//! Navigator-UI-Loop des CANbossTouch-Rust-Ports auf der FT81x-Pipeline.
//!
//! Kombiniert den Render-/Touch-Pfad aus [`crate::oxivgl::platform`]
//! (LVGL-PARTIAL-Streifen per SPI ins RAM_G, Touch-Poll der FT813-Engine)
//! mit dem [`Navigator`](oxivgl::navigator::Navigator)-Scan aus
//! `oxivgl::view::run_app_nav` (Push/Pop der Views, Event-Aktionen).

extern crate alloc;

use embassy_time::{Duration, Instant, Timer};
use oxivgl::display::{DISPLAY_READY, LvglBuffers};
use oxivgl::driver::LvglDriver;
use oxivgl::navigator::Navigator;
use oxivgl::view::NavAction;
use oxivgl_sys::LV_DEF_REFR_PERIOD;

use crate::board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};
use crate::canboss::views::menu::MenuView;
use crate::ft81x::Ft81x;
use crate::oxivgl::display::Ft813Display;
use crate::oxivgl::indev::TouchInput;
use crate::oxivgl::platform::{LVGL_BUF_BYTES, poll_touch};

const LVGL_TICK_MS: u64 = LV_DEF_REFR_PERIOD as u64 / 4;
const UI_TICK_MS: u64 = 5;
/// Abstand der Navigator-Scans (View::update + NavAction-Verarbeitung).
const SCAN_PERIOD_MS: u64 = 33;

/// CANbossTouch-App ausfuehren (Menue → Knoten-Screens / PoC-Hallenlicht).
///
/// `eve` muss initialisiert sein und den `RAM_G`-Framebuffer scannen
/// ([`Ft81x::init`] + [`Ft81x::show_framebuffer`]).
pub async fn run_canboss_app(eve: &'static mut Ft81x, bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>) -> ! {
    let driver = LvglDriver::init(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32);
    let _display = Ft813Display::init(eve, bufs);

    DISPLAY_READY.wait().await;

    // Widgets (Switch/Slider/Spinbox/Bar) im protronic-Akzent statt LVGL-Default.
    crate::canboss::views::apply_accent_theme();

    let mut nav = Navigator::new();
    nav.push_root(MenuView::default());

    let touch = TouchInput::register();
    let mut last_pressed = false;
    let mut next_scan = Instant::now();
    let mut next_lvgl_tick = Instant::now();

    defmt::info!("canboss UI loop starting (navigator)");

    loop {
        Timer::after(Duration::from_millis(UI_TICK_MS)).await;

        let had_touch = poll_touch(&touch, &mut last_pressed);
        let now = Instant::now();

        // LVGL-Timer regelmaessig treiben; bei Touch sofort, damit
        // Press/Release-Uebergaenge ohne Verzoegerung gerendert werden.
        if had_touch || now >= next_lvgl_tick {
            driver.timer_handler();
            next_lvgl_tick = now + Duration::from_millis(LVGL_TICK_MS);
        }

        if now < next_scan {
            continue;
        }
        next_scan = now + Duration::from_millis(SCAN_PERIOD_MS);

        // Navigator-Scan: aktive View/Modal updaten, Aktionen verarbeiten
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
    }
}
