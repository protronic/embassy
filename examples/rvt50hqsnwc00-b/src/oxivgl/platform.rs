//! STM32U5 + Embassy LTDC platform glue for OxivGL.
//!
//! Two independent async tasks:
//!
//! - [`run_widget_demo`] — UI task. Owns all LVGL calls: drives
//!   `lv_timer_handler` (which also services the pointer indev read timer)
//!   and presents frames to LTDC.
//! - [`run_touch`] — touch task (`touch` feature). Sleeps on the CTP_INT
//!   EXTI line and only polls the I2C touch controller while a touch is in
//!   progress, publishing samples for the LVGL indev read callback.

extern crate alloc;

use embassy_stm32::ltdc::{self, Ltdc, LtdcLayer, LtdcLayerConfig};
use embassy_stm32::peripherals;
use embassy_time::{Duration, Timer};
use oxivgl::display::{DISPLAY_READY, LvglBuffers};
use oxivgl::driver::LvglDriver;
use oxivgl::view::{View, register_view_events};
use oxivgl::widgets::{Obj, Screen};
use oxivgl_sys::LV_DEF_REFR_PERIOD;
use static_cell::StaticCell;

use crate::oxivgl::display::{
    LtdcDisplay, front_framebuffer, prefill_background, present_framebuffer, sync_back_from_front,
};
use crate::oxivgl::widget_view::WidgetView;
use crate::rvt50_board::DISPLAY_WIDTH;

/// LVGL timer pass cadence; 4 passes per LTDC present keeps animations smooth.
const LVGL_TICK_MS: u64 = LV_DEF_REFR_PERIOD as u64 / 4;
/// LVGL timer passes between LTDC presents (4 × 8 ms ≈ 32 ms ≈ 30 fps).
const PRESENT_LVGL_TICKS: usize = 4;

/// OxivGL stripe buffer dimensions (lines × width × 2 bytes).
pub const COLOR_BUF_LINES: usize = 20;
pub const LVGL_BUF_BYTES: usize = crate::rvt50_board::DISPLAY_WIDTH * COLOR_BUF_LINES * 2;

static VIEW: StaticCell<WidgetView> = StaticCell::new();

// ---------------------------------------------------------------------------
// Interrupt-driven touch task (touch feature only)
// ---------------------------------------------------------------------------

/// Poll cadence while a touch is in progress.
#[cfg(feature = "touch")]
const TOUCH_ACTIVE_POLL_MS: u64 = 10;
/// Consecutive idle reads after an interrupt before re-arming on CTP_INT.
/// Covers controllers that assert INT slightly before coordinates are valid
/// and ensures the release sample is published before going back to sleep.
#[cfg(feature = "touch")]
const TOUCH_IDLE_READS: u32 = 3;

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
    fn poll(&mut self) -> crate::oxivgl::indev::TouchSample {
        use defmt::info;
        let t = crate::rvt50_board::read_touch(&mut self.i2c);

        if t.pressed {
            self.last_x = t.x as i32;
            self.last_y = t.y as i32;
        }
        // Idle reads park at the panel edge; keep releases on the last contact
        // point so LVGL can finish click hit-testing on the pressed widget.
        let sample = crate::oxivgl::indev::TouchSample {
            x: self.last_x,
            y: self.last_y,
            pressed: t.pressed,
        };

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

/// Interrupt-driven touch sampling loop.
///
/// Waits for the touch controller to assert CTP_INT (active low), then polls
/// over I2C until the contact ends, publishing every sample for the LVGL
/// pointer indev. Between touches no I2C traffic occurs and the task is
/// suspended on the EXTI line.
#[cfg(feature = "touch")]
pub async fn run_touch(
    i2c: embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::Master>,
    mut touch_int: embassy_stm32::exti::ExtiInput<'static, embassy_stm32::mode::Async>,
) -> ! {
    let mut poller = TouchPoller::new(i2c);
    defmt::info!("oxivgl touch task waiting on CTP_INT");

    loop {
        // Active-low: returns immediately if a touch is already asserted,
        // otherwise suspends until the controller pulls the line down.
        touch_int.wait_for_low().await;

        let mut idle_reads = 0u32;
        loop {
            let sample = poller.poll();
            crate::oxivgl::indev::publish_touch_sample(sample);

            if sample.pressed {
                idle_reads = 0;
            } else {
                idle_reads += 1;
                if idle_reads >= TOUCH_IDLE_READS {
                    break;
                }
            }
            Timer::after(Duration::from_millis(TOUCH_ACTIVE_POLL_MS)).await;
        }
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
// UI task
// ---------------------------------------------------------------------------

/// Run the OxivGL widget demo render loop.
///
/// With the `touch` feature, [`run_touch`] must be spawned as a separate task;
/// the pointer indev registered here consumes its published samples from the
/// LVGL read timer inside `lv_timer_handler`.
pub async fn run_widget_demo(
    mut ltdc: Ltdc<'static, peripherals::LTDC, ltdc::Rgb565>,
    bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>,
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
    crate::oxivgl::indev::register_pointer_indev();

    loop {
        // Copy the visible buffer so partial LVGL flushes land on a complete
        // frame, run the LVGL timers (refresh + indev read), then present.
        sync_back_from_front();

        for _ in 0..PRESENT_LVGL_TICKS {
            driver.timer_handler();
            Timer::after(Duration::from_millis(LVGL_TICK_MS)).await;
        }

        let _ = view.update();
        present_to_ltdc(&mut ltdc).await;
    }
}
