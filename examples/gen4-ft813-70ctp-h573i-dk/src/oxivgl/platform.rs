//! Embassy + FT813-SPI platform glue for the gen4-FT813-70CTP OxivGL demo.
//!
//! The FT813 continuously scans its `RAM_G` framebuffer out to the panel, so
//! — exactly like the PIO-RGB gen4-RP2350 port — there is **no buffer swap
//! and no `present()`**: LVGL flushes dirty regions straight into the live
//! framebuffer (over SPI). The UI task owns LVGL, polls the FT813 touch
//! engine over the same SPI bus, and drives `lv_timer_handler` on a fixed
//! tick.
//!
//! The main loop mirrors `gen4-rp2350-70ct`: touch input triggers a
//! multi-tick LVGL batch so press/release animations finish before the next
//! idle tick, instead of hammering partial SPI flushes on every sample.

extern crate alloc;

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_time::{Duration, Instant, Timer};
use oxivgl::display::{DISPLAY_READY, LvglBuffers};
use oxivgl::driver::LvglDriver;
use oxivgl::view::{View, register_view_events};
use oxivgl::widgets::{Obj, Screen};
use oxivgl_sys::LV_DEF_REFR_PERIOD;
use static_cell::StaticCell;

use crate::board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};
use crate::ft81x::Ft81x;
use crate::oxivgl::display::{self, Ft813Display};
use crate::oxivgl::indev::{TouchInput, TouchSample};
use crate::oxivgl::touch_dbg;
use crate::oxivgl::widget_view::WidgetView;

/// LVGL timer tick — run the handler ~4× per LVGL refresh period.
const LVGL_TICK_MS: u64 = LV_DEF_REFR_PERIOD as u64 / 4;
pub(crate) const PRESENT_PERIOD_MS: u64 = 33;
pub(crate) const UI_TICK_MS: u64 = 5;
const PRESENT_LVGL_TICKS: usize = 4;

/// Number of display lines covered by each LVGL partial stripe buffer.
///
/// LVGL renders each dirty area into one of these SRAM stripe buffers, and
/// the flush callback pushes it into the FT813's `RAM_G` over SPI. Wide
/// stripes keep the SPI bursts long and sequential (full-width stripes go
/// out as a single burst — see [`Ft81x::blit`]).
///
/// `800 × 39 × RGB565 ≈ 62.4 KiB` per buffer; two of them fit comfortably in
/// the STM32H573's 640 KiB SRAM next to the 128 KiB LVGL pool.
pub const COLOR_BUF_LINES: usize = 39;
/// Byte size of one LVGL partial stripe buffer (RGB565).
pub const LVGL_BUF_BYTES: usize = DISPLAY_WIDTH * COLOR_BUF_LINES * 2;

static VIEW: StaticCell<WidgetView> = StaticCell::new();
static FIRST_TOUCH_UI_LOGGED: AtomicBool = AtomicBool::new(false);

/// Poll the FT813 capacitive touch engine once and feed the LVGL indev.
///
/// Returns `true` while pressed and on the press→release edge so the caller
/// can run a present batch (matching the queue-drain semantics of the other
/// gen4 ports).
pub(crate) fn poll_touch(touch: &TouchInput, last_pressed: &mut bool) -> bool {
    // SAFETY: called from the single UI task only (with_eve contract).
    let point = match unsafe { display::with_eve(|eve| eve.touch()) } {
        Some(Ok(point)) => point,
        Some(Err(e)) => {
            defmt::warn!("ft81x touch read failed: {}", e);
            return false;
        }
        None => return false,
    };

    let sample = TouchSample {
        x: point.x,
        y: point.y,
        pressed: point.pressed,
    };
    let had_touch = point.pressed || *last_pressed;
    *last_pressed = point.pressed;

    if point.pressed {
        touch_dbg::bump_queued();
        if !FIRST_TOUCH_UI_LOGGED.swap(true, Ordering::Relaxed) {
            defmt::info!(
                "oxivgl first touch sample x={} y={} pressed={}",
                sample.x,
                sample.y,
                sample.pressed
            );
        }
    }
    touch.feed(sample);
    had_touch
}

pub(crate) async fn lvgl_present_batch<V: View>(
    driver: &LvglDriver,
    view: &mut V,
    touch: &TouchInput,
    last_pressed: &mut bool,
) {
    for _ in 0..PRESENT_LVGL_TICKS {
        let _ = poll_touch(touch, last_pressed);
        driver.timer_handler();
        Timer::after(Duration::from_millis(LVGL_TICK_MS)).await;
    }
    let _ = view.update();
}

/// Run the OxivGL widget demo (LVGL UI task on the FT813 SPI framebuffer).
///
/// `eve` must be initialised and already scanning the `RAM_G` framebuffer
/// ([`Ft81x::init`] + [`Ft81x::show_framebuffer`]).
pub async fn run_widget_demo(eve: &'static mut Ft81x, bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>) -> ! {
    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32);
    let _display = Ft813Display::init(eve, bufs);

    DISPLAY_READY.wait().await;

    let view = VIEW.init(WidgetView::default());
    let screen = Screen::active().expect("LVGL screen must exist after display init");
    let container = Obj::from_raw_non_owning(screen.handle());
    if view.create(&container).is_err() {
        defmt::warn!("oxivgl widget create failed");
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }
    register_view_events(view, &container);

    let touch = TouchInput::register();
    let mut last_pressed = false;
    let mut heartbeat_ticks: u32 = 0;

    defmt::info!("oxivgl UI loop starting");

    // Paint the first full frame before entering the interactive loop.
    lvgl_present_batch(&driver, view, &touch, &mut last_pressed).await;

    let mut next_present = Instant::now() + Duration::from_millis(PRESENT_PERIOD_MS);

    loop {
        Timer::after(Duration::from_millis(UI_TICK_MS)).await;

        let had_touch = poll_touch(&touch, &mut last_pressed);

        if had_touch {
            // Let LVGL finish press/release transitions before going idle again.
            lvgl_present_batch(&driver, view, &touch, &mut last_pressed).await;
            next_present = Instant::now() + Duration::from_millis(PRESENT_PERIOD_MS);
        } else {
            driver.timer_handler();
            if Instant::now() >= next_present {
                lvgl_present_batch(&driver, view, &touch, &mut last_pressed).await;
                next_present = Instant::now() + Duration::from_millis(PRESENT_PERIOD_MS);
            }
        }

        heartbeat_ticks += 1;
        // ~1 s at UI_TICK_MS = 5 ms
        if heartbeat_ticks % 200 == 0 {
            defmt::info!(
                "oxivgl heartbeat queued={} fed={} read_cb={} clicks={} flushes={}",
                touch_dbg::QUEUED.load(core::sync::atomic::Ordering::Relaxed),
                touch_dbg::FED.load(core::sync::atomic::Ordering::Relaxed),
                touch_dbg::READ_CB.load(core::sync::atomic::Ordering::Relaxed),
                touch_dbg::LVGL_CLICKS.load(core::sync::atomic::Ordering::Relaxed),
                touch_dbg::LVGL_FLUSHES.load(core::sync::atomic::Ordering::Relaxed),
            );
        }
    }
}
