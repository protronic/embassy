//! Compile-time render path label (for logs and A/B comparison on hardware).

/// Human-readable mode name printed at boot and in heartbeat lines.
#[cfg(feature = "eve")]
pub const NAME: &str = "eve_gpu";

#[cfg(not(feature = "eve"))]
pub const NAME: &str = "framebuffer";

/// Short description for defmt boot lines.
#[cfg(feature = "eve")]
pub const DETAIL: &str = "LVGL draw_eve (co-processor commands, no RAM_G blit)";

#[cfg(not(feature = "eve"))]
pub const DETAIL: &str = "LVGL SW renderer + host SPI blit to RAM_G";
