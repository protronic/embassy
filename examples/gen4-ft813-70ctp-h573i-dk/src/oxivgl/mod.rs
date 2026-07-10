//! OxivGL (C LVGL v9.5) integration for the gen4-FT813-70CTP-CLB SPI panel.
//!
//! Real LVGL compiled via [`oxivgl-sys`] (with `conf/lv_conf.h`).
//!
//! - **Default** (`oxivgl-demo`): framebuffer mode — software renderer, partial
//!   stripes uploaded over host SPI ([`crate::ft81x::Ft81x::blit`]).
//! - **`eve` feature**: EVE External GPU renderer — LVGL sends draw commands to
//!   the FT813 co-processor ([`eve_display`]).

pub mod display;
#[cfg(feature = "eve")]
pub mod eve_display;
pub mod fonts;
pub mod hall_platform;
pub mod hall_view;
pub mod indev;
pub mod platform;
pub mod stats;
pub mod touch_dbg;
pub mod widget_view;
