//! PSRAM RGB565 flush for OxivGL / LVGL v9.5 on the gen4-RP2350-70CT-CLB.
//!
//! Full-screen buffers live in external PSRAM (8 MiB APS6404L). LVGL partial
//! stripes stay in on-chip SRAM via [`LvglBuffers`].

use core::ffi::c_void;
use core::ptr;
use core::slice;

use oxivgl::display::{DISPLAY_READY, LvglBuffers};
use oxivgl_sys::{
    lv_area_t, lv_display_create, lv_display_flush_ready, lv_display_set_buffers,
    lv_display_set_color_format, lv_display_set_default, lv_display_set_flush_cb,
    lv_display_t, lv_color_format_t_LV_COLOR_FORMAT_RGB565,
    lv_display_render_mode_t_LV_DISPLAY_RENDER_MODE_PARTIAL,
};

use crate::gen4_board::{self, PsramFramebuffers};

static mut LVGL_DISP: *mut lv_display_t = core::ptr::null_mut();
static mut FRAME: Option<PsramFramebuffers> = None;
static mut SHOW_FRONT: bool = false;

/// LVGL display token — proves LVGL display init completed.
#[derive(Debug)]
pub struct RgbDisplay;

impl RgbDisplay {
    /// Register LVGL display buffers and wire the PSRAM flush callback.
    pub fn init<const BYTES: usize>(
        w: i32,
        h: i32,
        bufs: &'static mut LvglBuffers<BYTES>,
        frame: PsramFramebuffers,
    ) -> Self {
        // SAFETY: `lv_init()` completed in `LvglDriver::init`; single init.
        unsafe {
            FRAME = Some(frame);
            lvgl_disp_init_rgb(w, h, bufs);
        }
        Self
    }
}

pub(crate) fn lvgl_display() -> *mut lv_display_t {
    // SAFETY: written once during init before the UI loop runs.
    unsafe { LVGL_DISP }
}

fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3)
}

/// Fill both PSRAM framestores with the demo background colour.
pub fn prefill_background() {
    let px = rgb565(16, 32, 48).to_le_bytes();
    // SAFETY: only the UI task touches PSRAM framestores.
    unsafe {
        if let Some(frame) = FRAME.as_ref() {
            for fb in [frame.back, frame.front] {
                let base = fb;
                for i in (0..frame.bytes).step_by(2) {
                    ptr::copy_nonoverlapping(px.as_ptr(), base.add(i), 2);
                }
            }
        }
    }
}

/// Register LVGL display buffers and wire the PSRAM flush callback.
///
/// # Safety
/// `lv_init()` must have been called. `bufs` must remain valid for the
/// display lifetime.
pub unsafe fn lvgl_disp_init_rgb<const BYTES: usize>(
    w: i32,
    h: i32,
    bufs: &'static mut LvglBuffers<BYTES>,
) -> *mut lv_display_t {
    // SAFETY: single init; pointers come from PSRAM framestores.
    unsafe {
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
        disp
    }
}

/// Copy the visible PSRAM buffer to the back buffer before partial LVGL flushes.
pub fn sync_back_from_front() {
    // SAFETY: only the UI task touches PSRAM framestores.
    unsafe {
        if let Some(frame) = FRAME.as_ref() {
            let (src, dst) = if SHOW_FRONT {
                (frame.front, frame.back)
            } else {
                (frame.back, frame.front)
            };
            ptr::copy_nonoverlapping(src, dst, frame.bytes);
        }
    }
}

/// Pointer to the PSRAM buffer currently shown (before the next swap).
pub fn front_framebuffer() -> *const u16 {
    // SAFETY: only the UI task touches PSRAM framestores.
    unsafe {
        FRAME
            .as_ref()
            .map(|f| {
                if SHOW_FRONT {
                    f.front as *const u16
                } else {
                    f.back as *const u16
                }
            })
            .unwrap_or(core::ptr::null())
    }
}

/// Swap PSRAM buffers and scan the new front buffer out to the RGB panel.
pub fn present_framebuffer() {
    // SAFETY: only the UI task touches PSRAM framestores.
    unsafe {
        if let Some(frame) = FRAME.as_ref() {
            SHOW_FRONT = !SHOW_FRONT;
            let ptr = if SHOW_FRONT {
                frame.front as *const u16
            } else {
                frame.back as *const u16
            };
            gen4_board::gen4_lcd_present_rgb565(
                ptr,
                gen4_board::DISPLAY_WIDTH as u16,
                gen4_board::DISPLAY_HEIGHT as u16,
            );
        }
    }
}

unsafe extern "C" fn flush_callback(
    disp: *mut lv_display_t,
    area_p: *const lv_area_t,
    px_map: *mut u8,
) {
    if disp.is_null() || area_p.is_null() || px_map.is_null() {
        return;
    }

    // SAFETY: LVGL provides valid area and pixel map for the duration of this callback.
    let area = unsafe { &*area_p };
    if area.x2 < area.x1 || area.y2 < area.y1 {
        return;
    }

    let w = (area.x2 - area.x1 + 1) as usize;
    let h = (area.y2 - area.y1 + 1) as usize;
    let row_bytes = w * 2;
    let stride = gen4_board::DISPLAY_WIDTH * 2;

    // SAFETY: px_map points at `w*h` RGB565 pixels supplied by LVGL.
    let src = unsafe { slice::from_raw_parts(px_map, row_bytes * h) };

    // SAFETY: back PSRAM buffer is only written from this LVGL flush callback.
    unsafe {
        let Some(frame) = FRAME.as_ref() else {
            lv_display_flush_ready(disp);
            return;
        };
        let back = if SHOW_FRONT {
            frame.back
        } else {
            frame.front
        };
        for row in 0..h {
            let y = area.y1 as usize + row;
            if y >= gen4_board::DISPLAY_HEIGHT {
                break;
            }
            let dst_off = y * stride + area.x1 as usize * 2;
            let src_off = row * row_bytes;
            let end = dst_off + row_bytes;
            if end <= frame.bytes {
                ptr::copy_nonoverlapping(
                    src[src_off..].as_ptr(),
                    back.add(dst_off),
                    row_bytes,
                );
            }
        }
        lv_display_flush_ready(disp);
    }
}
