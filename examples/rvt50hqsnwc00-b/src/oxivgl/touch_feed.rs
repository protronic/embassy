//! Async touch sampling task — feeds the UI task via an Embassy [`Watch`].
//!
//! I2C polling runs in its own task so blocking reads do not stall LVGL /
//! LTDC. The UI task (sole LVGL owner) pulls the latest sample and calls
//! [`super::indev::TouchInput::feed`].

use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Timer};

use crate::oxivgl::indev::TouchSample;
use crate::oxivgl::touch_dbg;
use crate::rvt50_board;

/// Latest raw touch sample from the board (includes I2C metadata for debug).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TouchBoardSample {
    pub x: i32,
    pub y: i32,
    pub pressed: bool,
    pub i2c_ok: bool,
    pub raw_status: u8,
}

impl From<TouchBoardSample> for TouchSample {
    fn from(s: TouchBoardSample) -> Self {
        Self {
            x: s.x,
            y: s.y,
            pressed: s.pressed,
        }
    }
}

/// Touch sample cadence (matches former in-task poller).
const TOUCH_POLL_MS: u64 = 5;

static TOUCH_WATCH: Watch<CriticalSectionRawMutex, TouchBoardSample, 1> = Watch::new();

/// Latest touch sample published by [`run_touch_poll_task`].
pub fn latest() -> TouchSample {
    TOUCH_WATCH
        .anon_receiver()
        .try_get()
        .map(TouchSample::from)
        .unwrap_or_default()
}

/// Embassy task: poll the capacitive panel over I2C and publish samples.
#[embassy_executor::task]
pub async fn run_touch_poll_task(
    mut i2c: embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::Master>,
) -> ! {
    let sender = TOUCH_WATCH.sender();
    let mut was_pressed = false;
    let mut last_x = 0i32;
    let mut last_y = 0i32;

    sender.send(TouchBoardSample::default());

    loop {
        let t = rvt50_board::read_touch(&mut i2c);

        if t.pressed {
            last_x = t.x as i32;
            last_y = t.y as i32;
        }

        let sample = TouchBoardSample {
            x: last_x,
            y: last_y,
            pressed: t.pressed,
            i2c_ok: t.i2c_ok,
            raw_status: t.raw_status,
        };

        touch_dbg::publish_touch(
            sample.x,
            sample.y,
            sample.pressed,
            sample.i2c_ok,
            sample.raw_status,
        );

        if t.pressed && !was_pressed {
            info!(
                "oxivgl touch down x={} y={} raw=0x{:02x}",
                sample.x, sample.y, t.raw_status
            );
        } else if !t.pressed && was_pressed {
            info!("oxivgl touch up x={} y={}", sample.x, sample.y);
        }
        was_pressed = t.pressed;

        sender.send(sample);
        Timer::after(Duration::from_millis(TOUCH_POLL_MS)).await;
    }
}
