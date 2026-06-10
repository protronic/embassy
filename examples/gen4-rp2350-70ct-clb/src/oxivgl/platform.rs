//! RP2350 + Embassy platform glue for OxivGL on the gen4-RP2350-70CT-CLB.
//!
//! The PIO/DMA scan-out (driven by a separate task via [`crate::dpi::Dpi`])
//! continuously streams the PSRAM framebuffer, so the UI loop only has to
//! pump LVGL timers and feed touch samples — flushed stripes become visible
//! on the next scan-out frame without an explicit present step.

extern crate alloc;

use embassy_time::{Duration, Timer};
use oxivgl::display::{DISPLAY_READY, LvglBuffers};
use oxivgl::driver::LvglDriver;
use oxivgl::view::{View, register_view_events};
use oxivgl::widgets::{Obj, Screen};
#[cfg(not(feature = "touch"))]
use oxivgl_sys::LV_DEF_REFR_PERIOD;
use static_cell::StaticCell;

use crate::gen4_board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};
use crate::oxivgl::display::PsramDisplay;
#[cfg(feature = "touch")]
use crate::oxivgl::indev::{TouchInput, TouchSample};
use crate::oxivgl::widget_view::WidgetView;

/// LVGL timer cadence for the UI loop.
#[cfg(not(feature = "touch"))]
const LVGL_TICK_MS: u64 = LV_DEF_REFR_PERIOD as u64 / 4;
/// Touch sample cadence; short presses are not missed at this rate.
#[cfg(feature = "touch")]
const TOUCH_POLL_MS: u64 = 5;

/// OxivGL stripe buffer dimensions (lines × width × 2 bytes).
pub const COLOR_BUF_LINES: usize = 40;
pub const LVGL_BUF_BYTES: usize = DISPLAY_WIDTH * COLOR_BUF_LINES * 2;

static VIEW: StaticCell<WidgetView> = StaticCell::new();

/// Reads the FT5446 over I2C and tracks press/release transitions.
#[cfg(feature = "touch")]
struct TouchPoller {
    i2c: embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Blocking>,
    was_pressed: bool,
    last_x: i32,
    last_y: i32,
}

#[cfg(feature = "touch")]
impl TouchPoller {
    fn new(i2c: embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Blocking>) -> Self {
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
        let t = crate::gen4_board::read_touch(&mut self.i2c);

        if t.pressed {
            self.last_x = t.x as i32;
            self.last_y = t.y as i32;
        }
        // Keep releases on the last contact point so LVGL can finish click
        // hit-testing on the pressed widget.
        let sample = TouchSample {
            x: self.last_x,
            y: self.last_y,
            pressed: t.pressed,
        };

        if t.pressed && !self.was_pressed {
            info!(
                "oxivgl touch down x={} y={} points={}",
                sample.x, sample.y, t.raw_status
            );
        } else if !t.pressed && self.was_pressed {
            info!("oxivgl touch up x={} y={}", sample.x, sample.y);
        }
        self.was_pressed = t.pressed;
        sample
    }
}

/// Run the OxivGL widget demo UI loop (never returns).
///
/// The scan-out must already be running (see [`crate::dpi::Dpi::present`]);
/// `fb` is the PSRAM framebuffer the scan-out streams from. With the `touch`
/// feature, `i2c` is the blocking I2C1 bus to the FT5446.
pub async fn run_widget_demo(
    fb: *mut u8,
    bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>,
    #[cfg(feature = "touch")] i2c: embassy_rp::i2c::I2c<
        'static,
        embassy_rp::peripherals::I2C1,
        embassy_rp::i2c::Blocking,
    >,
) -> ! {
    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32);
    let _display = PsramDisplay::init(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32, bufs, fb);

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
    register_view_events(view);
    view.log_layout();

    #[cfg(feature = "touch")]
    let touch = TouchInput::register();
    #[cfg(feature = "touch")]
    let mut poller = TouchPoller::new(i2c);

    loop {
        #[cfg(feature = "touch")]
        {
            // Sample touch *before* the LVGL pass and feed it in *after*, so
            // refresh state is settled before press events are dispatched.
            touch.publish(poller.poll());
            driver.timer_handler();
            touch.sync_read();
            let _ = view.update();
            Timer::after(Duration::from_millis(TOUCH_POLL_MS)).await;
        }

        #[cfg(not(feature = "touch"))]
        {
            driver.timer_handler();
            let _ = view.update();
            Timer::after(Duration::from_millis(LVGL_TICK_MS)).await;
        }
    }
}
