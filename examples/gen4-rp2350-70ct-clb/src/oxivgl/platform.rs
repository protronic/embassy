//! RP2350 + PSRAM RGB565 platform glue for OxivGL.

extern crate alloc;

#[cfg(feature = "touch")]
use embassy_rp::i2c::{Blocking, I2c};
#[cfg(feature = "touch")]
use embassy_rp::peripherals;
use embassy_time::{Duration, Instant, Timer};
use oxivgl::display::{DISPLAY_READY, LvglBuffers};
use oxivgl::driver::LvglDriver;
use oxivgl::view::{View, register_view_events};
use oxivgl::widgets::{Obj, Screen};
use oxivgl_sys::LV_DEF_REFR_PERIOD;
use static_cell::StaticCell;

use crate::gen4_board::{self, DISPLAY_WIDTH};
use crate::oxivgl::display::{prefill_background, present_framebuffer, sync_back_from_front, RgbDisplay};
#[cfg(feature = "touch")]
use crate::oxivgl::indev::{TouchInput, TouchSample};
use crate::oxivgl::widget_view::WidgetView;

const LVGL_TICK_MS: u64 = LV_DEF_REFR_PERIOD as u64 / 4;
const PRESENT_PERIOD_MS: u64 = 33;
#[cfg(feature = "touch")]
const TOUCH_POLL_MS: u64 = 5;
const PRESENT_LVGL_TICKS: usize = 3;

pub const COLOR_BUF_LINES: usize = 20;
pub const LVGL_BUF_BYTES: usize = DISPLAY_WIDTH * COLOR_BUF_LINES * 2;

static VIEW: StaticCell<WidgetView> = StaticCell::new();

#[cfg(feature = "touch")]
struct TouchPoller {
    i2c: I2c<'static, peripherals::I2C1, Blocking>,
    was_pressed: bool,
    last_x: i32,
    last_y: i32,
}

#[cfg(feature = "touch")]
impl TouchPoller {
    fn new(i2c: I2c<'static, peripherals::I2C1, Blocking>) -> Self {
        Self {
            i2c,
            was_pressed: false,
            last_x: 0,
            last_y: 0,
        }
    }

    fn poll(&mut self) -> TouchSample {
        use defmt::info;
        let t = gen4_board::read_touch(&mut self.i2c);

        if t.pressed {
            self.last_x = t.x as i32;
            self.last_y = t.y as i32;
        }
        let sample = TouchSample {
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

#[cfg(feature = "touch")]
fn pump_touch(driver: &LvglDriver, touch: &TouchInput, poller: &mut TouchPoller) {
    touch.publish(poller.poll());
    driver.timer_handler();
    touch.sync_read();
}

async fn lvgl_present_batch(
    driver: &LvglDriver,
    view: &mut WidgetView,
    #[cfg(feature = "touch")] touch: &TouchInput,
    #[cfg(feature = "touch")] poller: &mut TouchPoller,
) {
    sync_back_from_front();

    for _ in 0..PRESENT_LVGL_TICKS {
        #[cfg(feature = "touch")]
        touch.publish(poller.poll());
        driver.timer_handler();
        #[cfg(feature = "touch")]
        touch.sync_read();
        Timer::after(Duration::from_millis(LVGL_TICK_MS)).await;
    }

    let _ = view.update();
    present_framebuffer();
}

/// Run the OxivGL widget demo on the gen4-RP2350-70CT-CLB panel.
pub async fn run_widget_demo(
    resources: gen4_board::DisplayResources,
    bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>,
) -> ! {
    let gen4_board::DisplayResources {
        framebuffers,
        #[cfg(feature = "touch")]
        i2c,
        ..
    } = resources;

    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, gen4_board::DISPLAY_HEIGHT as i32);
    let _display = RgbDisplay::init(
        DISPLAY_WIDTH as i32,
        gen4_board::DISPLAY_HEIGHT as i32,
        bufs,
        framebuffers,
    );

    DISPLAY_READY.wait().await;
    prefill_background();
    present_framebuffer();

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

    #[cfg(feature = "touch")]
    lvgl_present_batch(&driver, view, &touch, &mut poller).await;
    #[cfg(not(feature = "touch"))]
    lvgl_present_batch(&driver, view).await;

    let mut next_present = Instant::now() + Duration::from_millis(PRESENT_PERIOD_MS);

    loop {
        #[cfg(feature = "touch")]
        Timer::after(Duration::from_millis(TOUCH_POLL_MS)).await;
        #[cfg(not(feature = "touch"))]
        Timer::at(next_present).await;

        #[cfg(feature = "touch")]
        pump_touch(&driver, &touch, &mut poller);
        #[cfg(not(feature = "touch"))]
        driver.timer_handler();

        if Instant::now() >= next_present {
            #[cfg(feature = "touch")]
            lvgl_present_batch(&driver, view, &touch, &mut poller).await;
            #[cfg(not(feature = "touch"))]
            lvgl_present_batch(&driver, view).await;
            next_present = Instant::now() + Duration::from_millis(PRESENT_PERIOD_MS);
        }
    }
}
