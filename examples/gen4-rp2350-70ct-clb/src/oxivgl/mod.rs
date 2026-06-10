//! OxivGL (C LVGL v9.5) integration for the gen4-RP2350-70CT-CLB.
//!
//! LVGL renders into SRAM stripe buffers; the flush callback copies the dirty
//! stripes into the PSRAM framebuffer that the PIO/DMA scan-out streams to
//! the panel (see [`crate::dpi`]). LVGL sources are compiled by
//! [`oxivgl-sys`] with `conf/lv_conf.h`.

pub mod display;
pub mod indev;
pub mod platform;
pub mod widget_view;
