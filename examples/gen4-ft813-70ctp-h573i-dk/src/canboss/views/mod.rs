//! LVGL-Views des CANbossTouch-Rust-Ports.
//!
//! Navigation ueber den oxivgl-[`Navigator`](oxivgl::navigator::Navigator):
//! [`menu::MenuView`] (Root) → [`node::NodeView`] je CANopen-Knoten bzw.
//! [`hall::HallScreenView`] (PoC-Hallenlicht).

pub mod hall;
pub mod menu;
pub mod node;

/// Farbpalette (an das C-Projekt/den PoC angelehnt).
pub const SCREEN_BG: u32 = 0xE7DCC8;
pub const SURFACE: u32 = 0xFFFDF8;
pub const CARD_BG: u32 = 0xFFFDF7;
pub const BORDER: u32 = 0xE4D8C3;
pub const TEXT: u32 = 0x151515;
pub const MUTED: u32 = 0x665F54;
pub const ACCENT: u32 = 0xA37418;
pub const OK_GREEN: u32 = 0x2E7D32;
pub const ERR_RED: u32 = 0xC62828;
