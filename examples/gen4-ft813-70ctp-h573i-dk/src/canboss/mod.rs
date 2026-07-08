//! CANbossTouch-Rust-Port: CANopen-Bediengeraet auf dem gen4-FT813 + H573I-DK.
//!
//! Vollport des C-Projekts `protronic/CANbossTouch`: Die LVGL-Screens werden
//! aus den EDS-Dateien (`eds/network.json`) generiert (build.rs), Werte
//! werden zyklisch per SDO-Upload gelesen und aus den Widgets per
//! SDO-Download parametriert. Dazu kommt die PoC-Hallenlichtsteuerung
//! ([`crate::oxivgl::hall_view`] + [`crate::touch_can`]) als Menuepunkt.
//!
//! Aufbau:
//! - [`types`]   — plattformunabhaengige Datentypen (mit dem Host-Port geteilt)
//! - [`canbus`]  — geteilter FDCAN: TX-Queue + RX-Router (SDO vs. PoC-Frames)
//! - [`sdo`]     — CiA-301-SDO-Client (expedited + segmented), Slot-Pool
//! - [`views`]   — Menue, generierte Knoten-Screens, PoC-Hall-Wrapper
//! - [`platform`]— Navigator-Loop auf der FT81x-Pipeline

pub mod canbus;
pub mod platform;
pub mod sdo;
pub mod types;
pub mod views;

pub use types::*;

// Vom build.rs generierte Tabellen (NODES + je-Knoten-Datenpunkte).
// Referenziert Dp/NodeDesc/DType/Access/WidgetKind aus `types` (oben re-exportiert).
include!(concat!(env!("OUT_DIR"), "/canboss_gen.rs"));
