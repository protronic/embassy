//! STM32U5 + Embassy LTDC platform glue for OxivGL.
//!
//! The touch render loop mirrors [`examples/oxivgl-host`]: publish the latest
//! sample, then call `timer_handler()` so LVGL's TIMER-mode pointer indev reads
//! it (same mechanism as the SDL mouse indev on the host).

extern crate alloc;

use embassy_stm32::ltdc::{self, Ltdc, LtdcLayer, LtdcLayerConfig};
use embassy_stm32::peripherals;
use embassy_time::{Duration, Instant, Timer};
use oxivgl::display::{DISPLAY_READY, LvglBuffers};
use oxivgl::driver::LvglDriver;
use oxivgl::view::{View, register_view_events};
use oxivgl::widgets::{Obj, Screen};
use oxivgl_sys::LV_DEF_REFR_PERIOD;
use static_cell::StaticCell;

use crate::oxivgl::display::{
    LtdcDisplay, front_framebuffer, prefill_background, present_framebuffer, sync_back_from_front,
};
#[cfg(feature = "touch")]
use crate::oxivgl::indev::{TouchInput, TouchSample};
#[cfg(feature = "touch")]
use crate::oxivgl::touch_dbg;
use crate::oxivgl::widget_view::WidgetView;
use crate::rvt50_board::DISPLAY_WIDTH;

const LVGL_TICK_MS: u64 = LV_DEF_REFR_PERIOD as u64 / 4;
/// LVGL timer passes per frame — matches `run_app` / `examples/oxivgl-host`.
const TICKS_PER_FRAME: usize = 4;
/// LTDC refresh cadence (~30 fps, aligned with `LV_DEF_REFR_PERIOD`).
const PRESENT_PERIOD_MS: u64 = LV_DEF_REFR_PERIOD as u64;

/// OxivGL stripe buffer dimensions (lines × width × 2 bytes).
pub const COLOR_BUF_LINES: usize = 20;
pub const LVGL_BUF_BYTES: usize = crate::rvt50_board::DISPLAY_WIDTH * COLOR_BUF_LINES * 2;

static VIEW: StaticCell<WidgetView> = StaticCell::new();

// ---------------------------------------------------------------------------
// Touch poller (touch feature only)
// ---------------------------------------------------------------------------

/// Reads the I2C touch panel and tracks press/release transitions.
#[cfg(feature = "touch")]
struct TouchPoller {
    i2c: embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::Master>,
    was_pressed: bool,
    last_x: i32,
    last_y: i32,
}

#[cfg(feature = "touch")]
impl TouchPoller {
    fn new(i2c: embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::Master>) -> Self {
        Self {
            i2c,
            was_pressed: false,
            last_x: 0,
            last_y: 0,
        }
    }

    /// Read the current touch state from I2C and log transitions.
    fn poll(&mut self) -> TouchSample {
        use defmt::info;
        let t = crate::rvt50_board::read_touch(&mut self.i2c);

        if t.pressed {
            self.last_x = t.x as i32;
            self.last_y = t.y as i32;
        }
        // Idle reads park at the panel edge; keep releases on the last contact
        // point so LVGL can finish click hit-testing on the pressed widget.
        let sample = TouchSample {
            x: self.last_x,
            y: self.last_y,
            pressed: t.pressed,
        };

        touch_dbg::publish_touch(
            sample.x,
            sample.y,
            sample.pressed,
            t.i2c_ok,
            t.raw_status,
        );

        if t.pressed && !self.was_pressed {
            info!(
                "oxivgl touch down x={} y={} raw=0x{:02x}",
                sample.x, sample.y, t.raw_status
            );
        } else if !t.pressed && self.was_pressed {
            info!("oxivgl touch up x={} y={}", sample.x, sample.y);
        }
        self.was_pressed = t.pressed;
        sample
    }
}

#[cfg(feature = "touch")]
fn publish_touch_sample(touch: &TouchInput, poller: &mut TouchPoller, view: &WidgetView) {
    let sample = poller.poll();
    touch.publish(sample);
    if sample.pressed {
        let hit_btn = view.find_button_at(sample.x, sample.y);
        touch.log_debug(sample, hit_btn.map(|(idx, _)| idx));
    }
}

// ---------------------------------------------------------------------------
// LTDC helpers
// ---------------------------------------------------------------------------

async fn init_ltdc_layer(ltdc: &mut Ltdc<'static, peripherals::LTDC, ltdc::Rgb565>) {
    let layer_config = LtdcLayerConfig {
        pixel_format: ltdc::PixelFormat::Rgb565,
        layer: LtdcLayer::Layer1,
        window_x0: 0,
        window_x1: DISPLAY_WIDTH as _,
        window_y0: 0,
        window_y1: crate::rvt50_board::DISPLAY_HEIGHT as _,
    };
    ltdc.init_layer(&layer_config, None);
}

async fn present_to_ltdc(ltdc: &mut Ltdc<'static, peripherals::LTDC, ltdc::Rgb565>) {
    let fb_ptr = present_framebuffer();
    ltdc.init_buffer(LtdcLayer::Layer1, fb_ptr as *const _);
    let _ = ltdc.reload().await;
}

// ---------------------------------------------------------------------------
// Shared LVGL tick batch (host + board)
// ---------------------------------------------------------------------------

/// One simulator-style frame: optional touch publish, then `TICKS_PER_FRAME`
/// passes of `lv_timer_handler()`.
async fn lvgl_tick_batch(
    driver: &LvglDriver,
    #[cfg(feature = "touch")] touch: &TouchInput,
    #[cfg(feature = "touch")] poller: &mut TouchPoller,
    #[cfg(feature = "touch")] view: &WidgetView,
) {
    for _ in 0..TICKS_PER_FRAME {
        #[cfg(feature = "touch")]
        publish_touch_sample(touch, poller, view);
        driver.timer_handler();
        Timer::after(Duration::from_millis(LVGL_TICK_MS)).await;
    }
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

/// Run the OxivGL widget demo.
///
/// With the `touch` feature: `i2c` is the blocking I2C bus to the touch
/// controller. Touch samples are published before each `timer_handler()` call,
/// matching the SDL mouse path used by `examples/oxivgl-host`.
pub async fn run_widget_demo(
    mut ltdc: Ltdc<'static, peripherals::LTDC, ltdc::Rgb565>,
    bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>,
    #[cfg(feature = "touch")] i2c: embassy_stm32::i2c::I2c<
        'static,
        embassy_stm32::mode::Blocking,
        embassy_stm32::i2c::Master,
    >,
) -> ! {
    init_ltdc_layer(&mut ltdc).await;

    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, crate::rvt50_board::DISPLAY_HEIGHT as i32);
    let _display = LtdcDisplay::init(DISPLAY_WIDTH as i32, crate::rvt50_board::DISPLAY_HEIGHT as i32, bufs);

    DISPLAY_READY.wait().await;
    prefill_background();

    ltdc.init_buffer(LtdcLayer::Layer1, front_framebuffer() as *const _);
    let _ = ltdc.reload().await;

    let view = VIEW.init(WidgetView::default());
    let screen = Screen::active().expect("LVGL screen must exist after display init");
    let container = Obj::from_raw_non_owning(screen.handle());
    if view.create(&container).is_err() {
        defmt::warn!("oxivgl widget create failed");
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }
    register_view_events(view);
    view.log_layout();

    #[cfg(feature = "touch")]
    let touch = TouchInput::register();
    #[cfg(feature = "touch")]
    let mut poller = TouchPoller::new(i2c);

    // Initial flush: settle the display before entering the main loop.
    #[cfg(feature = "touch")]
    lvgl_tick_batch(&driver, &touch, &mut poller, view).await;
    #[cfg(not(feature = "touch"))]
    lvgl_tick_batch(&driver).await;
    sync_back_from_front();
    present_to_ltdc(&mut ltdc).await;

    let mut next_present = Instant::now() + Duration::from_millis(PRESENT_PERIOD_MS);

    loop {
        let _ = view.update();

        #[cfg(feature = "touch")]
        lvgl_tick_batch(&driver, &touch, &mut poller, view).await;
        #[cfg(not(feature = "touch"))]
        lvgl_tick_batch(&driver).await;

        if Instant::now() >= next_present {
            sync_back_from_front();
            present_to_ltdc(&mut ltdc).await;
            next_present = Instant::now() + Duration::from_millis(PRESENT_PERIOD_MS);
        }
    }
}
