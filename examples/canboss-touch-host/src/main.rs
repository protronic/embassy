//! Host (SDL2) port of the CANbossTouch Rust firmware.
//!
//! Rendert dieselben EDS-Screens wie das Binary `canboss_touch` auf dem
//! STM32H573I-DK — Menue (Knotenliste aus `eds/network.json`) und je Knoten
//! ein Screen mit einer Widget-Zeile pro Datenpunkt — am Entwicklungs-PC, um
//! Layout, Widget-Auswahl und SDO-Wertdarstellung ohne Hardware zu pruefen.
//!
//! Der SDO-Client ([`canboss::sdo`]) und die Views ([`canboss::views`]) sind
//! per `#[path]` direkt aus der Firmware geteilt; ein In-Memory-SDO-Server-Mock
//! ([`canboss::canbus`]) beantwortet die Client-Requests mit simulierten,
//! datentyp-gerechten Werten (Vorbild: `examples/oxivgl-host`).
//!
//! Die PoC-Hallenlichtsteuerung ist im Host-Port ausgeblendet (sie haengt an
//! `touch_can`/FDCAN) — dafuer gibt es `examples/oxivgl-host`.
//!
//! ```bash
//! cd examples/canboss-touch-host
//! cargo run --release            # SDL-Fenster, Maus bedienbar
//! ```
//!
//! Headless-Vorschau (CI): `CANBOSS_HOST_SCREENSHOT=<prefix>` rendert Menue
//! und alle Knoten-Screens als BMP und beendet sich (mit `SDL_VIDEODRIVER=dummy`).

// Lokales Modul `crate::oxivgl::fonts` (von den geteilten Views erwartet, siehe
// src/oxivgl/mod.rs) — kollidiert nicht mit der externen Crate `oxivgl`: in
// Edition 2024 stammt `use oxivgl::…` aus dem Extern-Prelude, `crate::oxivgl::…`
// aus diesem Modul.
mod canboss;
mod oxivgl;

use core::ffi::CStr;

// Am Crate-Root verdeckt das lokale Modul `oxivgl` die externe Crate; mit
// fuehrendem `::` adressieren wir die Crate (die Views in Submodulen sehen den
// lokalen Konflikt nicht und nutzen `oxivgl::…` direkt).
use ::oxivgl::driver::LvglDriver;
use ::oxivgl::navigator::Navigator;
use ::oxivgl::view::NavAction;
use canboss::sdo;
use canboss::views::menu::MenuView;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use log::info;
use oxivgl_sys::LV_DEF_REFR_PERIOD;

use crate::canboss::NODES;

const DISPLAY_WIDTH: i32 = 800;
const DISPLAY_HEIGHT: i32 = 480;
const LVGL_TICK_MS: u64 = LV_DEF_REFR_PERIOD as u64 / 4;
const SCAN_PERIOD_MS: u64 = 33;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    info!(
        "CANbossTouch host (SDL {DISPLAY_WIDTH}x{DISPLAY_HEIGHT}) — {} Knoten, Mock-SDO-Server",
        NODES.len()
    );

    let title = CStr::from_bytes_with_nul(b"CANbossTouch (host)\0").unwrap();
    let driver = LvglDriver::sdl(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        .title(title)
        .mouse(true)
        .build();

    // Geteilter SDO-Client-Worker (Antworten liefert der Mock in canboss::canbus).
    // Das `#[task]`-Makro dieser embassy-Version liefert Result<SpawnToken, _>.
    spawner.spawn(sdo::worker_task().unwrap());

    // Widgets im protronic-Akzent statt LVGL-Default (vor dem Bau der Views).
    canboss::views::apply_accent_theme();

    let mut nav = Navigator::new();
    nav.push_root(MenuView::default());

    let shot_prefix = std::env::var("CANBOSS_HOST_SCREENSHOT").ok();
    let mut shot_stage: i32 = 0;
    let mut stage_start = Instant::now();
    let mut next_scan = Instant::now();

    loop {
        let now = Instant::now();
        if now >= next_scan {
            next_scan = now + Duration::from_millis(SCAN_PERIOD_MS);
            let action = nav
                .active_view_mut()
                .map(|v| v.update())
                .unwrap_or(Ok(NavAction::None))
                .unwrap_or(NavAction::None);
            if !nav.process_pending_event_action() && !action.is_none() {
                nav.process_action(action);
            }
            nav.drain_toast_requests();
            nav.tick_toast();
        }

        for _ in 0..4 {
            driver.timer_handler();
            Timer::after(Duration::from_millis(LVGL_TICK_MS)).await;
        }

        // Headless-Screenshot-Ablauf: Menue, dann jeder Knoten-Screen (je 2 s).
        if let Some(prefix) = &shot_prefix {
            if stage_start.elapsed() > Duration::from_millis(2000) {
                let path = format!("{prefix}_{shot_stage}.bmp");
                save_screen_bmp(&path);
                info!("Screenshot: {path}");

                if (shot_stage as usize) < NODES.len() {
                    let node = &NODES[shot_stage as usize];
                    nav.process_action(NavAction::push(canboss::views::node::NodeView::new(node), None));
                } else {
                    std::process::exit(0);
                }
                shot_stage += 1;
                stage_start = Instant::now();
            }
        }
    }
}

/// Aktiven LVGL-Screen als 24-Bit-BMP speichern (lv_snapshot, ARGB8888 -> BGR).
fn save_screen_bmp(path: &str) {
    unsafe {
        let scr = oxivgl_sys::lv_screen_active();
        let snap = oxivgl_sys::lv_snapshot_take(scr, oxivgl_sys::lv_color_format_t_LV_COLOR_FORMAT_ARGB8888);
        if snap.is_null() {
            log::warn!("Snapshot fehlgeschlagen");
            return;
        }
        let header = &(*snap).header;
        // w/h/stride sind Bitfeld-Accessoren (bindgen), keine Felder.
        let (w, h, stride) = (header.w() as usize, header.h() as usize, header.stride() as usize);
        let data = std::slice::from_raw_parts((*snap).data, stride * h);

        let row_out = (w * 3 + 3) & !3;
        let data_size = row_out * h;
        let mut buf = Vec::with_capacity(54 + data_size);
        buf.extend_from_slice(b"BM");
        buf.extend_from_slice(&((54 + data_size) as u32).to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes());
        buf.extend_from_slice(&54u32.to_le_bytes());
        buf.extend_from_slice(&40u32.to_le_bytes());
        buf.extend_from_slice(&(w as u32).to_le_bytes());
        buf.extend_from_slice(&(h as u32).to_le_bytes());
        buf.extend_from_slice(&1u16.to_le_bytes());
        buf.extend_from_slice(&24u16.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes());
        buf.extend_from_slice(&(data_size as u32).to_le_bytes());
        buf.extend_from_slice(&[0u8; 16]);

        for y in (0..h).rev() {
            let row = &data[y * stride..y * stride + w * 4];
            for px in row.chunks_exact(4) {
                buf.extend_from_slice(&[px[0], px[1], px[2]]); // B,G,R
            }
            buf.extend(std::iter::repeat(0u8).take(row_out - w * 3));
        }
        oxivgl_sys::lv_draw_buf_destroy(snap);
        let _ = std::fs::write(path, buf);
    }
}
