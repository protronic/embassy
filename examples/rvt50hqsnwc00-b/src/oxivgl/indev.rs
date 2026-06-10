//! Capacitive touch input for OxivGL on the Riverdi RVT50.
//!
//! Uses LVGL's standard **timer-mode** pointer indev: the touch task publishes
//! samples with [`publish_touch_sample`] and LVGL pulls the latest one from
//! its periodic indev read timer (driven by `lv_timer_handler`). This is the
//! canonical LVGL integration — no manual `lv_indev_read` calls and no
//! event-mode read-timer pausing, which previously kept press/release events
//! from reaching widgets.
//!
//! All LVGL FFI stays inside this module.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use oxivgl_sys::{
    lv_indev_create, lv_indev_data_t, lv_indev_get_read_timer, lv_indev_set_display, lv_indev_set_read_cb,
    lv_indev_set_type, lv_indev_state_t_LV_INDEV_STATE_PRESSED, lv_indev_state_t_LV_INDEV_STATE_RELEASED, lv_indev_t,
    lv_indev_type_t_LV_INDEV_TYPE_POINTER, lv_timer_set_period,
};

use crate::oxivgl::display::lvgl_display;

/// Period of the LVGL indev read timer in ms (default is `LV_DEF_REFR_PERIOD`,
/// 32 ms). Shorter so press/release edges are picked up promptly.
const INDEV_READ_PERIOD_MS: u32 = 10;

/// One touch sample as published by the touch task.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TouchSample {
    /// Horizontal coordinate in display pixels.
    pub x: i32,
    /// Vertical coordinate in display pixels.
    pub y: i32,
    /// `true` while the panel reports active contact.
    pub pressed: bool,
}

#[derive(Clone, Copy)]
struct TouchState {
    /// Most recent sample from the touch task.
    latest: TouchSample,
    /// Pressed sample not yet consumed by the LVGL read callback. Guarantees
    /// LVGL observes at least one pressed read even for very short taps.
    pending_press: Option<TouchSample>,
}

static TOUCH_STATE: Mutex<CriticalSectionRawMutex, Cell<TouchState>> = Mutex::new(Cell::new(TouchState {
    latest: TouchSample {
        x: 0,
        y: 0,
        pressed: false,
    },
    pending_press: None,
}));

/// `true` once [`register_pointer_indev`] has run.
static INDEV_REGISTERED: AtomicBool = AtomicBool::new(false);

/// Last state reported to LVGL, for edge logging in the read callback.
static LAST_REPORTED_PRESSED: AtomicBool = AtomicBool::new(false);

/// Publish the latest board touch sample (called from the touch task).
///
/// Safe to call before [`register_pointer_indev`]; samples are simply stored
/// until LVGL starts reading them.
pub fn publish_touch_sample(sample: TouchSample) {
    TOUCH_STATE.lock(|cell| {
        let mut state = cell.get();
        if sample.pressed {
            state.pending_press = Some(sample);
        }
        state.latest = sample;
        cell.set(state);
    });
}

/// Create the LVGL pointer indev in default timer mode and bind it to the
/// LTDC display. Call once from the UI task after `LtdcDisplay::init`.
pub fn register_pointer_indev() {
    assert!(
        !INDEV_REGISTERED.swap(true, Ordering::Relaxed),
        "register_pointer_indev called twice"
    );

    let disp = lvgl_display();
    assert!(!disp.is_null(), "LVGL display must be initialised first");

    // SAFETY: `lv_init()` completed in `LvglDriver::init`; display is valid.
    unsafe {
        let indev = lv_indev_create();
        assert!(!indev.is_null(), "lv_indev_create returned NULL");
        lv_indev_set_type(indev, lv_indev_type_t_LV_INDEV_TYPE_POINTER);
        lv_indev_set_display(indev, disp);
        lv_indev_set_read_cb(indev, Some(pointer_read_cb));

        let read_timer = lv_indev_get_read_timer(indev);
        assert!(!read_timer.is_null(), "indev read timer missing");
        lv_timer_set_period(read_timer, INDEV_READ_PERIOD_MS);
    }

    info!(
        "oxivgl pointer indev registered (timer mode, {} ms read period)",
        INDEV_READ_PERIOD_MS
    );
}

/// LVGL read callback — runs inside `lv_timer_handler` on the UI task.
unsafe extern "C" fn pointer_read_cb(_indev: *mut lv_indev_t, data: *mut lv_indev_data_t) {
    if data.is_null() {
        return;
    }

    let sample = TOUCH_STATE.lock(|cell| {
        let mut state = cell.get();
        let sample = state.pending_press.take().unwrap_or(state.latest);
        cell.set(state);
        sample
    });

    let was_pressed = LAST_REPORTED_PRESSED.swap(sample.pressed, Ordering::Relaxed);
    if was_pressed != sample.pressed {
        info!(
            "oxivgl indev -> LVGL {} x={} y={}",
            if sample.pressed { "press" } else { "release" },
            sample.x,
            sample.y
        );
    }

    // SAFETY: `data` is a valid out-parameter from LVGL for the duration of this callback.
    let out = unsafe { &mut *data };
    out.point.x = sample.x;
    out.point.y = sample.y;
    out.state = if sample.pressed {
        lv_indev_state_t_LV_INDEV_STATE_PRESSED
    } else {
        lv_indev_state_t_LV_INDEV_STATE_RELEASED
    };
    out.continue_reading = false;
}
