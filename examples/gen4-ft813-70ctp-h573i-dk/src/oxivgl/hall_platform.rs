//! FT813-SPI platform glue for the JSON-driven hall lighting CAN demo.
//!
//! Same pipeline as [`super::platform::run_widget_demo`] — LVGL partial
//! stripes flushed into the FT813's `RAM_G` over SPI, touch polled from the
//! FT813 touch engine on the same bus — but running [`HallView`], whose
//! button events feed the CAN press/hold/repeat tasks in
//! [`crate::touch_can`].

extern crate alloc;

use embassy_time::{Duration, Instant, Timer};
#[cfg(not(feature = "eve"))]
use oxivgl::display::{DISPLAY_READY, LvglBuffers};
#[cfg(feature = "eve")]
use oxivgl::display::DISPLAY_READY;
use oxivgl::driver::LvglDriver;
use oxivgl::view::{View, register_view_events};
use oxivgl::widgets::{Obj, Screen};
use static_cell::StaticCell;

use crate::board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};
use crate::ft81x::Ft81x;
use crate::oxivgl::display::Ft813Display;
use crate::oxivgl::hall_view::HallView;
use crate::oxivgl::indev::TouchInput;
#[cfg(not(feature = "eve"))]
use crate::oxivgl::platform::{LVGL_BUF_BYTES, PRESENT_PERIOD_MS, UI_TICK_MS, lvgl_present_batch, poll_touch};
#[cfg(feature = "eve")]
use crate::oxivgl::platform::{PRESENT_PERIOD_MS, UI_TICK_MS, lvgl_present_batch, poll_touch};

static VIEW: StaticCell<HallView> = StaticCell::new();

/// Run the JSON-driven hall lighting CAN demo (LVGL UI task on the FT813 SPI
/// framebuffer).
///
/// `eve` must be initialised and already scanning the `RAM_G` framebuffer
/// ([`Ft81x::init`] + [`Ft81x::co_show_framebuffer`]).
#[cfg(not(feature = "eve"))]
pub async fn run_hall_demo(eve: &'static mut Ft81x, bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>) -> ! {
    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32);
    let _display = Ft813Display::init(eve, bufs);
    run_hall_ui_loop(driver).await
}

#[cfg(feature = "eve")]
pub async fn run_hall_demo(eve: &'static mut Ft81x) -> ! {
    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32);
    let _display = Ft813Display::init(eve);
    run_hall_ui_loop(driver).await
}

async fn run_hall_ui_loop(driver: LvglDriver) -> ! {
    DISPLAY_READY.wait().await;

    let view = VIEW.init(HallView::default());
    let screen = Screen::active().expect("LVGL screen must exist after display init");
    let container = Obj::from_raw_non_owning(screen.handle());
    if view.create(&container).is_err() {
        defmt::warn!("oxivgl hall create failed");
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }
    register_view_events(view, &container);

    let touch = TouchInput::register();
    let mut last_pressed = false;

    defmt::info!("oxivgl hall UI loop starting");

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
    }
}
