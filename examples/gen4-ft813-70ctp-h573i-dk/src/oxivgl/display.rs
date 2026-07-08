//! OxivGL / LVGL v9.5 display glue for the gen4-FT813 (EVE2) SPI panel.
//!
//! Unlike an LTDC or PIO-RGB board there is no CPU-addressable framebuffer:
//! the FT813 scans its own 1 MiB `RAM_G` out to the panel. LVGL therefore
//! runs in `PARTIAL` render mode with small SRAM stripe buffers, and the
//! flush callback pushes each dirty rectangle into `RAM_G` over SPI
//! ([`Ft81x::blit`]). The static EVE display list set up by
//! [`Ft81x::show_framebuffer`] keeps scanning the bitmap, so no display-list
//! swap is needed per frame — pixels appear on the next panel refresh.
//!
//! Single-task constraint: the FT813 driver is stored as a raw pointer and
//! accessed from the LVGL flush callback *and* the touch poll. Both run
//! inside the one UI task (`platform::run_widget_demo`), never concurrently.

use core::ffi::c_void;
use core::sync::atomic::{AtomicPtr, Ordering};
use core::{ptr, slice};

use oxivgl::display::{DISPLAY_READY, LvglBuffers};
use oxivgl_sys::{
    lv_area_t, lv_color_format_t_LV_COLOR_FORMAT_RGB565, lv_display_create, lv_display_flush_ready,
    lv_display_render_mode_t_LV_DISPLAY_RENDER_MODE_PARTIAL, lv_display_set_buffers, lv_display_set_color_format,
    lv_display_set_default, lv_display_set_flush_cb, lv_display_t,
};

use crate::board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};
use crate::ft81x::Ft81x;
use crate::oxivgl::touch_dbg;

/// LVGL display handle, set once in [`Ft813Display::init`].
static LVGL_DISP: AtomicPtr<lv_display_t> = AtomicPtr::new(ptr::null_mut());

/// The FT813 driver the flush callback and touch poll write through.
static EVE: AtomicPtr<Ft81x> = AtomicPtr::new(ptr::null_mut());

/// LVGL display token — proves [`Ft813Display::init`] completed.
#[derive(Debug)]
pub struct Ft813Display;

impl Ft813Display {
    /// Register the LVGL display and wire the FT813 SPI flush callback.
    ///
    /// `eve` must already be initialised ([`Ft81x::init`] +
    /// [`Ft81x::show_framebuffer`]); `bufs` are the LVGL partial stripe
    /// buffers (small SRAM working buffers, not full screens).
    pub fn init<const BYTES: usize>(eve: &'static mut Ft81x, bufs: &'static mut LvglBuffers<BYTES>) -> Self {
        EVE.store(eve as *mut Ft81x, Ordering::Release);
        // SAFETY: `lv_init()` ran in `LvglDriver::init`; single init; `'static` bufs.
        unsafe {
            init_display(bufs);
        }
        Self
    }
}

/// Return the LVGL display created by [`Ft813Display::init`].
pub(crate) fn lvgl_display() -> *mut lv_display_t {
    LVGL_DISP.load(Ordering::Acquire)
}

/// Run `f` on the shared FT813 driver. Returns `None` before
/// [`Ft813Display::init`].
///
/// # Safety
/// UI-task only: the caller must be the single LVGL task, and `f` must not
/// re-enter `with_eve` (the flush callback and the touch poll never nest).
pub(crate) unsafe fn with_eve<R>(f: impl FnOnce(&mut Ft81x) -> R) -> Option<R> {
    let eve = EVE.load(Ordering::Acquire);
    if eve.is_null() {
        return None;
    }
    // SAFETY: single UI task (caller contract) — no aliasing at runtime.
    Some(f(unsafe { &mut *eve }))
}

/// # Safety
/// `lv_init()` must have been called and `bufs` must outlive the display.
unsafe fn init_display<const BYTES: usize>(bufs: &'static mut LvglBuffers<BYTES>) {
    // SAFETY: single init; buffer pointers come from static-mut LVGL stripes.
    unsafe {
        let buf1 = ptr::addr_of_mut!(bufs.buf1) as *mut c_void;
        let buf2 = ptr::addr_of_mut!(bufs.buf2) as *mut c_void;

        let disp = lv_display_create(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32);
        assert!(!disp.is_null(), "lv_display_create returned NULL");

        // Plain (little-endian) RGB565 — matches the FT81x RGB565 bitmap
        // format, so flushed stripes go out over SPI byte-for-byte.
        lv_display_set_color_format(disp, lv_color_format_t_LV_COLOR_FORMAT_RGB565);
        // PARTIAL mode: LVGL renders dirty areas into the stripe buffers,
        // then `flush_callback` pushes them into RAM_G over SPI.
        lv_display_set_buffers(
            disp,
            buf1,
            buf2,
            BYTES as u32,
            lv_display_render_mode_t_LV_DISPLAY_RENDER_MODE_PARTIAL,
        );
        lv_display_set_flush_cb(disp, Some(flush_callback));
        lv_display_set_default(disp);

        LVGL_DISP.store(disp, Ordering::Release);
        DISPLAY_READY.signal(());
    }
}

unsafe extern "C" fn flush_callback(disp: *mut lv_display_t, area_p: *const lv_area_t, px_map: *mut u8) {
    if disp.is_null() || area_p.is_null() || px_map.is_null() {
        return;
    }

    // SAFETY: LVGL guarantees `area` and `px_map` valid for this callback.
    let area = unsafe { &*area_p };
    if area.x2 < area.x1 || area.y2 < area.y1 {
        // SAFETY: signalling readiness on a valid display is always sound.
        unsafe { lv_display_flush_ready(disp) };
        return;
    }

    let x = area.x1 as usize;
    let y = area.y1 as usize;
    let w = (area.x2 - area.x1 + 1) as usize;
    let h = (area.y2 - area.y1 + 1) as usize;

    // SAFETY: `px_map` points at `w * h` RGB565 pixels supplied by LVGL;
    // flush runs on the UI task (see `with_eve` contract).
    let src = unsafe { slice::from_raw_parts(px_map, w * h * 2) };
    let res = unsafe { with_eve(|eve| eve.blit(x, y, w, h, src)) };
    match res {
        Some(Ok(())) => touch_dbg::bump_lvgl_flush(),
        Some(Err(e)) => defmt::warn!("ft81x flush failed: {}", e),
        None => defmt::warn!("ft81x flush before display init"),
    }

    // SAFETY: `disp` is the valid LVGL display created during init.
    unsafe { lv_display_flush_ready(disp) };
}
