//! Shared heartbeat line for framebuffer vs EVE GPU A/B comparison.

use crate::oxivgl::touch_dbg;

/// Log a ~1 s heartbeat with mode tag and comparable counters.
pub fn log_heartbeat() {
    defmt::info!(
        "oxivgl mode={} queued={} fed={} read_cb={} clicks={} flushes={}",
        crate::render_mode::NAME,
        touch_dbg::QUEUED.load(core::sync::atomic::Ordering::Relaxed),
        touch_dbg::FED.load(core::sync::atomic::Ordering::Relaxed),
        touch_dbg::READ_CB.load(core::sync::atomic::Ordering::Relaxed),
        touch_dbg::LVGL_CLICKS.load(core::sync::atomic::Ordering::Relaxed),
        touch_dbg::LVGL_FLUSHES.load(core::sync::atomic::Ordering::Relaxed),
    );
}
