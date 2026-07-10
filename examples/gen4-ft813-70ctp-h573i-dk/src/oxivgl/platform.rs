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
#[cfg(not(feature = "eve"))]
use oxivgl::display::{DISPLAY_READY, LvglBuffers};
#[cfg(feature = "eve")]
use oxivgl::display::DISPLAY_READY;
use oxivgl::driver::LvglDriver;
use oxivgl::view::{View, register_view_events};
use oxivgl::widgets::{Obj, Screen};
use oxivgl_sys::LV_DEF_REFR_PERIOD;
use static_cell::StaticCell;

use crate::board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};
use crate::ft81x::Ft81x;
#[cfg(not(feature = "eve"))]
use crate::oxivgl::display::{self, Ft813Display};
#[cfg(feature = "eve")]
use crate::oxivgl::display::Ft813Display;
use crate::oxivgl::indev::{TouchInput, TouchSample};
use crate::oxivgl::touch_dbg;
use crate::oxivgl::widget_view::WidgetView;
use crate::oxivgl::stats;

/// LVGL timer tick — run the handler ~4× per LVGL refresh period.
const LVGL_TICK_MS: u64 = LV_DEF_REFR_PERIOD as u64 / 4;
pub(crate) const PRESENT_PERIOD_MS: u64 = 33;
pub(crate) const UI_TICK_MS: u64 = 5;
const PRESENT_LVGL_TICKS: usize = 4;

/// Number of display lines covered by each LVGL partial stripe buffer (framebuffer mode).
#[cfg(not(feature = "eve"))]
pub const COLOR_BUF_LINES: usize = 39;
/// Byte size of one LVGL partial stripe buffer (RGB565, framebuffer mode only).
#[cfg(not(feature = "eve"))]
pub const LVGL_BUF_BYTES: usize = DISPLAY_WIDTH * COLOR_BUF_LINES * 2;

static VIEW: StaticCell<WidgetView> = StaticCell::new();
static FIRST_TOUCH_UI_LOGGED: AtomicBool = AtomicBool::new(false);

/// Poll the FT813 capacitive touch engine once and feed the LVGL indev.
pub(crate) fn poll_touch(touch: &TouchInput, last_pressed: &mut bool) -> bool {
    #[cfg(feature = "eve")]
    let point = {
        let disp = crate::oxivgl::display::lvgl_display();
        crate::oxivgl::display::touch_sample(disp)
    };
    #[cfg(not(feature = "eve"))]
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

async fn run_widget_ui_loop(driver: LvglDriver) -> ! {
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

    defmt::info!(
        "oxivgl UI loop starting (mode={}, {})",
        crate::render_mode::NAME,
        crate::render_mode::DETAIL
    );

    lvgl_present_batch(&driver, view, &touch, &mut last_pressed).await;

    let mut next_present = Instant::now() + Duration::from_millis(PRESENT_PERIOD_MS);

    loop {
        Timer::after(Duration::from_millis(UI_TICK_MS)).await;

        let had_touch = poll_touch(&touch, &mut last_pressed);

        if had_touch {
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
        if heartbeat_ticks % 200 == 0 {
            stats::log_heartbeat();
        }
    }
}

/// Run the OxivGL widget demo (framebuffer / host-SPI flush mode).
#[cfg(not(feature = "eve"))]
pub async fn run_widget_demo(eve: &'static mut Ft81x, bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>) -> ! {
    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32);
    let _display = Ft813Display::init(eve, bufs);
    run_widget_ui_loop(driver).await
}

/// Run the OxivGL widget demo (LVGL EVE External GPU renderer).
#[cfg(feature = "eve")]
pub async fn run_widget_demo(eve: &'static mut Ft81x) -> ! {
    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32);
    let _display = Ft813Display::init(eve);
    run_widget_ui_loop(driver).await
}
