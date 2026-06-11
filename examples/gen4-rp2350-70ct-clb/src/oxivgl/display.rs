//! LVGL display flush into the PSRAM framebuffer.
//!
//! LVGL runs in `PARTIAL` render mode with two SRAM stripe buffers. Each
//! flush copies the dirty stripe into the single PSRAM framebuffer that the
//! PIO scan-out streams continuously, so updates become visible on the next
//! frame without an explicit present step.

use core::ffi::c_void;
use core::sync::atomic::{AtomicU32, Ordering};
use core::{ptr, slice};

use oxivgl::display::{DISPLAY_READY, LvglBuffers};
use oxivgl_sys::{
    lv_area_t, lv_color_format_t_LV_COLOR_FORMAT_RGB565, lv_display_create, lv_display_flush_ready,
    lv_display_render_mode_t_LV_DISPLAY_RENDER_MODE_PARTIAL, lv_display_set_buffers, lv_display_set_color_format,
    lv_display_set_default, lv_display_set_flush_cb, lv_display_t,
};

use crate::dpi::{ROW_BYTES, ROW_PIXEL_OFFSET};
use crate::gen4_board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};

/// LVGL display handle (set once in [`PsramDisplay::init`]).
static mut LVGL_DISP: *mut lv_display_t = ptr::null_mut();

/// PSRAM framebuffer base (set once in [`PsramDisplay::init`]).
static mut FRAMEBUFFER: *mut u8 = ptr::null_mut();

/// Number of LVGL flushes copied into the framebuffer (debug statistics).
static FLUSH_COUNT: AtomicU32 = AtomicU32::new(0);

/// How many LVGL flushes have reached the PSRAM framebuffer so far.
///
/// If this stays at 0 the LVGL render path is broken; if it advances but the
/// panel stays dark, the problem is in the scan-out or panel wiring.
pub fn flush_count() -> u32 {
    FLUSH_COUNT.load(Ordering::Relaxed)
}

/// PSRAM-backed LVGL display token — proves display init completed.
#[derive(Debug)]
pub struct PsramDisplay;

impl PsramDisplay {
    /// Register the LVGL display, stripe buffers, and PSRAM flush callback.
    ///
    /// `fb` must point at a [`crate::dpi::FRAME_BYTES`]-sized framebuffer
    /// that stays valid for the rest of the program.
    pub fn init<const BYTES: usize>(w: i32, h: i32, bufs: &'static mut LvglBuffers<BYTES>, fb: *mut u8) -> Self {
        // SAFETY: `lv_init()` completed in `LvglDriver::init`; single init;
        // `'static` stripe buffers; caller guarantees `fb` validity.
        unsafe {
            FRAMEBUFFER = fb;
            let buf1_ptr = ptr::addr_of_mut!(bufs.buf1) as *mut c_void;
            let buf2_ptr = ptr::addr_of_mut!(bufs.buf2) as *mut c_void;

            let disp = lv_display_create(w, h);
            assert!(!disp.is_null(), "lv_display_create returned NULL");

            lv_display_set_color_format(disp, lv_color_format_t_LV_COLOR_FORMAT_RGB565);
            lv_display_set_buffers(
                disp,
                buf1_ptr,
                buf2_ptr,
                BYTES as u32,
                lv_display_render_mode_t_LV_DISPLAY_RENDER_MODE_PARTIAL,
            );
            lv_display_set_flush_cb(disp, Some(flush_callback));
            lv_display_set_default(disp);
            LVGL_DISP = disp;
            DISPLAY_READY.signal(());
        }
        Self
    }
}

/// Return the LVGL display created by [`PsramDisplay::init`].
pub(crate) fn lvgl_display() -> *mut lv_display_t {
    // SAFETY: written once during init before the UI loop runs.
    unsafe { LVGL_DISP }
}

unsafe extern "C" fn flush_callback(disp: *mut lv_display_t, area_p: *const lv_area_t, px_map: *mut u8) {
    if disp.is_null() || area_p.is_null() || px_map.is_null() {
        return;
    }

    // SAFETY: LVGL provides a valid area and pixel map for the duration of this callback.
    let area = unsafe { &*area_p };
    if area.x2 < area.x1 || area.y2 < area.y1 {
        return;
    }

    let w = (area.x2 - area.x1 + 1) as usize;
    let h = (area.y2 - area.y1 + 1) as usize;
    let row_bytes = w * 2;

    // SAFETY: px_map points at `w*h` RGB565 pixels supplied by LVGL.
    let src = unsafe { slice::from_raw_parts(px_map, row_bytes * h) };

    FLUSH_COUNT.fetch_add(1, Ordering::Relaxed);

    // SAFETY: FRAMEBUFFER was set in init and covers DISPLAY_HEIGHT rows.
    unsafe {
        let fb = FRAMEBUFFER;
        if !fb.is_null() {
            for row in 0..h {
                let y = area.y1 as usize + row;
                if y >= DISPLAY_HEIGHT {
                    break;
                }
                let x = (area.x1 as usize).min(DISPLAY_WIDTH);
                let copy = row_bytes.min((DISPLAY_WIDTH - x) * 2);
                let dst = fb.add(y * ROW_BYTES + ROW_PIXEL_OFFSET + x * 2);
                ptr::copy_nonoverlapping(src[row * row_bytes..].as_ptr(), dst, copy);
            }
        }
        lv_display_flush_ready(disp);
    }
}
