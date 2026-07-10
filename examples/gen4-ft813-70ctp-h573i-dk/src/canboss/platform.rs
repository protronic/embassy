//! Navigator-UI-Loop des CANbossTouch-Rust-Ports auf der FT81x-Pipeline.

extern crate alloc;

use embassy_time::{Duration, Instant, Timer};
#[cfg(not(feature = "eve"))]
use oxivgl::display::{DISPLAY_READY, LvglBuffers};
#[cfg(feature = "eve")]
use oxivgl::display::DISPLAY_READY;
use oxivgl::driver::LvglDriver;
use oxivgl::navigator::Navigator;
use oxivgl::view::NavAction;
use oxivgl_sys::LV_DEF_REFR_PERIOD;

use crate::board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};
use crate::canboss::views::menu::MenuView;
use crate::ft81x::Ft81x;
use crate::oxivgl::display::Ft813Display;
use crate::oxivgl::indev::TouchInput;
#[cfg(not(feature = "eve"))]
use crate::oxivgl::platform::{LVGL_BUF_BYTES, poll_touch};
#[cfg(feature = "eve")]
use crate::oxivgl::platform::poll_touch;
use crate::oxivgl::stats;

const LVGL_TICK_MS: u64 = LV_DEF_REFR_PERIOD as u64 / 4;
const UI_TICK_MS: u64 = 5;
const SCAN_PERIOD_MS: u64 = 33;
const HEARTBEAT_TICKS: u32 = 400; // ~2 s at 5 ms UI tick

#[cfg(not(feature = "eve"))]
pub async fn run_canboss_app(eve: &'static mut Ft81x, bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>) -> ! {
    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32);
    let _display = Ft813Display::init(eve, bufs);
    run_canboss_ui_loop(driver).await
}

#[cfg(feature = "eve")]
pub async fn run_canboss_app(eve: &'static mut Ft81x) -> ! {
    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32);
    let _display = Ft813Display::init(eve);
    run_canboss_ui_loop(driver).await
}

async fn run_canboss_ui_loop(driver: LvglDriver) -> ! {
    DISPLAY_READY.wait().await;

    crate::canboss::views::apply_accent_theme();

    let mut nav = Navigator::new();
    nav.push_root(MenuView::default());

    let touch = TouchInput::register();
    let mut last_pressed = false;
    let mut next_scan = Instant::now();
    let mut next_lvgl_tick = Instant::now();
    let mut heartbeat_ticks: u32 = 0;

    defmt::info!(
        "canboss UI loop starting (mode={}, {})",
        crate::render_mode::NAME,
        crate::render_mode::DETAIL
    );

    loop {
        Timer::after(Duration::from_millis(UI_TICK_MS)).await;

        let had_touch = poll_touch(&touch, &mut last_pressed);
        let now = Instant::now();

        if had_touch || now >= next_lvgl_tick {
            driver.timer_handler();
            next_lvgl_tick = now + Duration::from_millis(LVGL_TICK_MS);
        }

        if now < next_scan {
            continue;
        }
        next_scan = now + Duration::from_millis(SCAN_PERIOD_MS);

        let action = nav
            .active_view_mut()
            .map(|v| v.update())
            .unwrap_or(Ok(NavAction::None))
            .unwrap_or_else(|e| {
                defmt::warn!("canboss view update failed: {}", defmt::Debug2Format(&e));
                NavAction::None
            });
        let modal_action = nav
            .active_modal_mut()
            .map(|m| m.update())
            .unwrap_or(Ok(NavAction::None))
            .unwrap_or(NavAction::None);

        let event_handled = nav.process_pending_event_action();
        if !event_handled {
            if !action.is_none() {
                nav.process_action(action);
            }
            if !modal_action.is_none() {
                nav.process_action(modal_action);
            }
        }

        nav.drain_toast_requests();
        nav.tick_toast();

        heartbeat_ticks += 1;
        if heartbeat_ticks % HEARTBEAT_TICKS == 0 {
            stats::log_heartbeat();
        }
    }
}
