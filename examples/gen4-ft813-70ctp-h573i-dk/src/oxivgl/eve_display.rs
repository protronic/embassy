//! LVGL EVE External GPU renderer for the gen4-FT813-70CTP (FT813).
//!
//! Uses [`lv_draw_eve_display_create`] from LVGL 9.5 — the co-processor renders
//! widgets as high-level EVE commands instead of shipping RGB565 stripes over
//! SPI. See <https://lvgl.io/docs/open/integration/external_display_controllers/eve/gpu>.

use core::ffi::c_void;
use core::sync::atomic::{AtomicPtr, Ordering};
use core::{ptr, slice};

use oxivgl::display::DISPLAY_READY;
use oxivgl_sys::{
    lv_display_set_default, lv_display_t, lv_draw_eve_display_create, lv_draw_eve_display_get_user_data,
    lv_draw_eve_memread32, lv_draw_eve_operation_t,
    lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_CS_ASSERT,
    lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_CS_DEASSERT,
    lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_POWERDOWN_CLEAR,
    lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_POWERDOWN_SET,
    lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_SPI_RECEIVE,
    lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_SPI_SEND, lv_draw_eve_parameters_t,
};

use crate::board::{DISPLAY_HEIGHT, DISPLAY_WIDTH, SPI_RUN_HZ};
use crate::ft81x::{
    Ft81x, PANEL_CSPREAD, PANEL_H_CYCLE, PANEL_H_OFFSET, PANEL_H_SYNC0, PANEL_H_SYNC1, PANEL_PCLK_DIV,
    PANEL_PCLK_POL, PANEL_SWIZZLE, PANEL_V_CYCLE, PANEL_V_OFFSET, PANEL_V_SYNC0, PANEL_V_SYNC1, TouchPoint,
};

/// FT813 `REG_TOUCH_SCREEN_XY` (same address as [`crate::ft81x`]).
const REG_TOUCH_SCREEN_XY: u32 = 0x0030_2124;

/// LVGL display handle from [`Ft813EveDisplay::init`].
static LVGL_DISP: AtomicPtr<lv_display_t> = AtomicPtr::new(ptr::null_mut());

/// FT813 SPI bridge passed as `user_data` to the LVGL EVE op callback.
static EVE: AtomicPtr<Ft81x> = AtomicPtr::new(ptr::null_mut());

/// LVGL EVE GPU display token — proves [`Ft813EveDisplay::init`] completed.
#[derive(Debug)]
pub struct Ft813EveDisplay;

impl Ft813EveDisplay {
    /// Create the LVGL EVE GPU display and run `EVE_init()` (≤11 MHz SPI).
    ///
    /// `eve` must be constructed via [`crate::board::init_ft813`] at the init
    /// SPI clock; this bumps to [`SPI_RUN_HZ`] after chip bring-up.
    pub fn init(eve: &'static mut Ft81x) -> Self {
        EVE.store(eve as *mut Ft81x, Ordering::Release);
        // SAFETY: `lv_init()` ran in `LvglDriver::init`; single init.
        unsafe {
            init_eve_display();
        }
        Self
    }
}

/// Return the LVGL display created by [`Ft813EveDisplay::init`].
pub(crate) fn lvgl_display() -> *mut lv_display_t {
    LVGL_DISP.load(Ordering::Acquire)
}

/// Read one touch sample via LVGL's EVE register helpers (axis-correct for gen4).
pub fn touch_sample(disp: *mut lv_display_t) -> TouchPoint {
    if disp.is_null() {
        return TouchPoint::default();
    }
    // SAFETY: `disp` is the valid EVE display; UI-task only.
    let xy = unsafe { lv_draw_eve_memread32(disp, REG_TOUCH_SCREEN_XY) };
    let rx = (xy & 0xFFFF) as i16;
    let ry = (xy >> 16) as i16;
    if rx == i16::MIN && ry == i16::MIN {
        return TouchPoint::default();
    }
    TouchPoint {
        x: (rx as i32).clamp(0, DISPLAY_WIDTH as i32 - 1),
        y: (ry as i32).clamp(0, DISPLAY_HEIGHT as i32 - 1),
        pressed: true,
    }
}

/// # Safety
/// `lv_init()` must have been called and [`Ft813EveDisplay::init`] must store `EVE`.
unsafe fn init_eve_display() {
    unsafe {
        let params = lv_draw_eve_parameters_t {
            hor_res: DISPLAY_WIDTH as u16,
            ver_res: DISPLAY_HEIGHT as u16,
            hcycle: PANEL_H_CYCLE,
            hoffset: PANEL_H_OFFSET,
            hsync0: PANEL_H_SYNC0,
            hsync1: PANEL_H_SYNC1,
            vcycle: PANEL_V_CYCLE,
            voffset: PANEL_V_OFFSET,
            vsync0: PANEL_V_SYNC0,
            vsync1: PANEL_V_SYNC1,
            swizzle: PANEL_SWIZZLE,
            pclkpol: PANEL_PCLK_POL,
            cspread: PANEL_CSPREAD,
            pclk: PANEL_PCLK_DIV,
            has_crystal: true,
            has_gt911: false,
            backlight_pwm: 96,
            backlight_freq: 4000,
        };

        let eve = EVE.load(Ordering::Acquire);
        assert!(!eve.is_null(), "Ft813EveDisplay: EVE pointer not set");

        let disp = lv_draw_eve_display_create(&params, Some(op_cb), eve as *mut c_void);
        assert!(!disp.is_null(), "lv_draw_eve_display_create returned NULL");

        lv_display_set_default(disp);
        LVGL_DISP.store(disp, Ordering::Release);

        // EVE_init() ran at the ≤11 MHz init clock; raise SPI for rendering.
        let eve = EVE.load(Ordering::Acquire);
        if !eve.is_null() {
            unsafe { (*eve).set_spi_frequency(SPI_RUN_HZ) };
        }

        defmt::info!("oxivgl EVE GPU display ready ({}x{})", DISPLAY_WIDTH, DISPLAY_HEIGHT);
        DISPLAY_READY.signal(());
    }
}

unsafe extern "C" fn op_cb(
    disp: *mut lv_display_t,
    operation: lv_draw_eve_operation_t,
    data: *mut c_void,
    length: u32,
) {
    let user = lv_draw_eve_display_get_user_data(disp) as *mut Ft81x;
    if user.is_null() {
        return;
    }
    // SAFETY: single UI task; `user` is the static `Ft81x` from init.
    let eve = unsafe { &mut *user };
    match operation {
        lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_POWERDOWN_SET => eve.eve_pd_set(true),
        lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_POWERDOWN_CLEAR => eve.eve_pd_set(false),
        lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_CS_ASSERT => eve.eve_cs_assert(),
        lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_CS_DEASSERT => eve.eve_cs_deassert(),
        lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_SPI_SEND => {
            if data.is_null() || length == 0 {
                return;
            }
            // SAFETY: LVGL passes a valid TX buffer for `length` bytes.
            let buf = unsafe { slice::from_raw_parts(data as *const u8, length as usize) };
            if let Err(e) = eve.eve_spi_send(buf) {
                defmt::warn!("eve op_cb SPI send failed: {}", e);
            }
        }
        lv_draw_eve_operation_t_LV_DRAW_EVE_OPERATION_SPI_RECEIVE => {
            if data.is_null() || length == 0 {
                return;
            }
            // SAFETY: LVGL passes a valid RX buffer for `length` bytes.
            let buf = unsafe { slice::from_raw_parts_mut(data as *mut u8, length as usize) };
            if let Err(e) = eve.eve_spi_receive(buf) {
                defmt::warn!("eve op_cb SPI receive failed: {}", e);
            }
        }
        _ => {}
    }
}
