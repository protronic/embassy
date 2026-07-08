//! OxivGL (C LVGL v9.5) integration for the gen4-FT813-70CTP-CLB SPI panel.
//!
//! Real LVGL compiled via [`oxivgl-sys`] (with `conf/lv_conf.h`). Rendered
//! stripes are pushed into the FT813's `RAM_G` framebuffer over SPI
//! ([`crate::ft81x`]), and the FT813's built-in capacitive touch engine feeds
//! the LVGL pointer indev — display and touch share the one SPI bus, so the
//! whole pipeline runs inside a single UI task.

pub mod display;
pub mod fonts;
pub mod indev;
pub mod platform;
pub mod touch_dbg;
pub mod widget_view;
