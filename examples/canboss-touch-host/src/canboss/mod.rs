//! Host-Modulbaum des CANbossTouch-Ports.
//!
//! Die interessanten, sonst ungetesteten Teile — der SDO-Client, die
//! generierten Datentypen und die Views (Menue + Knoten-Screens) — werden
//! per `#[path]` **direkt aus der Firmware geteilt** (nicht kopiert), damit
//! der Host-Build exakt denselben Code rendert wie das Geraet. Nur das
//! CAN-Backend ([`canbus`], hier ein Mock-SDO-Server) und `main.rs` sind
//! host-spezifisch.

#[path = "../../../gen4-ft813-70ctp-h573i-dk/src/canboss/types.rs"]
pub mod types;
pub use types::*;

pub mod canbus;

#[path = "../../../gen4-ft813-70ctp-h573i-dk/src/canboss/sdo.rs"]
pub mod sdo;

#[path = "../../../gen4-ft813-70ctp-h573i-dk/src/canboss/views/mod.rs"]
pub mod views;

// Vom build.rs (geteilter Codegen) generierte Tabellen — dieselben EDS-Dateien
// wie die Firmware. Referenziert Dp/NodeDesc/DType/Access/WidgetKind aus `types`.
include!(concat!(env!("OUT_DIR"), "/canboss_gen.rs"));
