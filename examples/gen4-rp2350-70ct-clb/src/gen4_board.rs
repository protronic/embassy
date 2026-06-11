//! Board support for the 4D Systems **gen4-RP2350-70CT-CLB** intelligent display.
//!
//! - **MCU**: Raspberry Pi RP2350B
//! - **Panel**: 7.0" 800×480 RGB565 (landscape)
//! - **Touch**: FocalTech FT5446 (capacitive), I2C1
//! - **PSRAM**: APS6404L 8 MiB on QMI CS1 / GPIO0
//!
//! Auxiliary pin assignments follow the Pico SDK `gen4_rp2350_*ct` board headers
//! (backlight, panel reset, touch controller).

use defmt::info;
use embassy_executor::Spawner;
use log::{info as uinfo, warn as uwarn};
use embassy_rp::gpio::{Level, Output};
#[cfg(feature = "touch")]
use embassy_rp::gpio::{Input, Pull};
#[cfg(feature = "touch")]
use embassy_rp::i2c::{Blocking, Config as I2cConfig, I2c};
use embassy_rp::peripherals::{self, PIN_17, PIN_37, PWM_SLICE0};
#[cfg(feature = "touch")]
use embassy_rp::peripherals::PIN_47;
use embassy_rp::psram::{self, Psram};
use embassy_rp::pwm::{Config as PwmConfig, Pwm, SetDutyCycle};
use embassy_rp::qmi_cs1::QmiCs1;
use embassy_rp::{Peri, Peripherals};

use embassy_time::{Duration, Timer};

pub const DISPLAY_WIDTH: usize = 800;
pub const DISPLAY_HEIGHT: usize = 480;

/// FT5446 default 7-bit I2C address on gen4-RP2350 CT panels.
#[cfg(feature = "touch")]
pub const TOUCH_I2C_ADDR: u8 = 0x38;

const FB_BYTES: usize = DISPLAY_WIDTH * DISPLAY_HEIGHT * 2;

/// PSRAM-backed RGB565 framestores for LVGL / panel scanout.
pub struct PsramFramebuffers {
    pub back: *mut u8,
    pub front: *mut u8,
    pub bytes: usize,
}

impl PsramFramebuffers {
    /// Reserve two full-screen RGB565 buffers at the base of PSRAM.
    ///
    /// # Safety
    /// `psram` must be initialised and `base_address()` must stay valid.
    pub unsafe fn new(psram: &Psram<'_>) -> Option<Self> {
        let base = psram.base_address();
        let total = psram.size() as usize;
        if total < FB_BYTES * 2 {
            return None;
        }
        Some(Self {
            back: base.cast::<u8>(),
            front: base.wrapping_add(FB_BYTES).cast::<u8>(),
            bytes: FB_BYTES,
        })
    }
}

pub struct DisplayResources {
    pub psram: Psram<'static>,
    pub framebuffers: PsramFramebuffers,
    #[cfg(feature = "touch")]
    pub i2c: I2c<'static, peripherals::I2C1, Blocking>,
}

/// Log whether the Graphics4D panel driver is linked (needs `GEN4_GRAPHICS4D_SDK`).
pub fn log_panel_driver_status() {
    #[cfg(gen4_graphics4d)]
    uinfo!("panel: Graphics4D SDK linked — RGB scanout active");
    #[cfg(not(gen4_graphics4d))]
    uwarn!(
        "panel: GEN4_GRAPHICS4D_SDK not set — gen4_lcd_present_rgb565() is a STUB; display stays blank"
    );
}

/// Initialise PSRAM, panel, backlight, USB logging, and optional FT5446 touch.
pub async fn init(spawner: &Spawner, p: Peripherals) -> Option<DisplayResources> {
    let Peripherals {
        USB,
        QMI_CS1,
        PIN_0,
        PIN_17,
        PWM_SLICE0,
        PIN_37,
        #[cfg(feature = "touch")]
        PIN_38,
        #[cfg(feature = "touch")]
        PIN_39,
        #[cfg(feature = "touch")]
        PIN_46,
        #[cfg(feature = "touch")]
        PIN_47,
        #[cfg(feature = "touch")]
        I2C1,
        ..
    } = p;

    crate::usb_log::spawn(spawner, USB);
    Timer::after(Duration::from_millis(100)).await;
    log_panel_driver_status();
    uinfo!(
        "gen4-RP2350-70CT-CLB OxivGL demo ({}x{})",
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT
    );
    uinfo!("board init: starting");

    let psram = Psram::new(
        QmiCs1::new(QMI_CS1, PIN_0),
        psram::Config::aps6404l(),
    )
    .ok()?;
    uinfo!("board init: PSRAM OK ({} KiB)", psram.size() / 1024);

    // SAFETY: PSRAM is mapped and sized by the driver.
    let framebuffers = unsafe { PsramFramebuffers::new(&psram)? };
    uinfo!(
        "board init: framebuffers {}x{} RGB565 x2 ({} KiB each)",
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
        FB_BYTES / 1024
    );

    info!(
        "gen4 PSRAM ready: {} KiB, framebuffers {}x{} RGB565 x2",
        psram.size() / 1024,
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT
    );

    reset_panel(PIN_37).await;
    init_backlight(PWM_SLICE0, PIN_17);
    uinfo!("board init: panel reset + backlight PWM done");

    // SAFETY: C shim may call into Graphics4D when `GEN4_GRAPHICS4D_SDK` is set.
    unsafe {
        gen4_lcd_init();
        gen4_lcd_backlight_enable();
    }
    uinfo!("board init: gen4_lcd_init() returned");

    #[cfg(feature = "touch")]
    {
        reset_touch(PIN_47).await;
        let i2c = I2c::new_blocking(I2C1, PIN_39, PIN_46, I2cConfig::default());
        let _touch_int = Input::new(PIN_38, Pull::Up);
        info!("FT5446 touch on I2C1 (SCL=GPIO39, SDA=GPIO46)");
        uinfo!("board init: FT5446 touch on I2C1");
        return Some(DisplayResources {
            psram,
            framebuffers,
            i2c,
        });
    }

    #[cfg(not(feature = "touch"))]
    {
        uinfo!("board init: complete (no touch feature)");
        Some(DisplayResources {
            psram,
            framebuffers,
        })
    }
}

async fn reset_panel(rst: Peri<'static, PIN_37>) {
    let mut panel_reset = Output::new(rst, Level::Low);
    panel_reset.set_low();
    Timer::after(Duration::from_millis(20)).await;
    panel_reset.set_high();
    Timer::after(Duration::from_millis(120)).await;
    info!("gen4 LCD reset sequence done");
}

#[cfg(feature = "touch")]
async fn reset_touch(rst: Peri<'static, PIN_47>) {
    let mut touch_reset = Output::new(rst, Level::Low);
    touch_reset.set_low();
    Timer::after(Duration::from_millis(10)).await;
    touch_reset.set_high();
    Timer::after(Duration::from_millis(50)).await;
}

fn init_backlight(slice: Peri<'static, PWM_SLICE0>, pin: Peri<'static, PIN_17>) {
    let mut pwm_config = PwmConfig::default();
    pwm_config.top = 255;
    pwm_config.compare_b = 220;
    let mut backlight = Pwm::new_output_b(slice, pin, pwm_config);
    backlight.set_duty_cycle(220).unwrap();
    info!("backlight PWM enabled on GPIO17");
}

#[derive(Clone, Copy, Default)]
pub struct TouchPoint {
    pub x: u16,
    pub y: u16,
    pub pressed: bool,
    pub raw_status: u8,
}

/// Read the first contact from an FT5446 (operating mode registers).
#[cfg(feature = "touch")]
pub fn read_touch(i2c: &mut I2c<'static, peripherals::I2C1, Blocking>) -> TouchPoint {
    let mut status = [0u8; 1];
    if i2c
        .blocking_write_read(TOUCH_I2C_ADDR, &[0x02], &mut status)
        .is_err()
    {
        return TouchPoint::default();
    }

    let raw_status = status[0];
    let count = raw_status & 0x0F;
    if count == 0 {
        return TouchPoint {
            raw_status,
            ..Default::default()
        };
    }

    let mut coords = [0u8; 4];
    if i2c
        .blocking_write_read(TOUCH_I2C_ADDR, &[0x03], &mut coords)
        .is_err()
    {
        return TouchPoint::default();
    }

    let mut x = u16::from(coords[0] & 0x0F) << 8 | u16::from(coords[1]);
    let mut y = u16::from(coords[2] & 0x0F) << 8 | u16::from(coords[3]);

    // Workshop / Linux EDT driver defaults for 800×480 gen4 RGB panels.
    x = DISPLAY_WIDTH as u16 - 1 - x.min(DISPLAY_WIDTH as u16 - 1);
    y = DISPLAY_HEIGHT as u16 - 1 - y.min(DISPLAY_HEIGHT as u16 - 1);

    TouchPoint {
        x,
        y,
        pressed: true,
        raw_status,
    }
}

unsafe extern "C" {
    fn gen4_lcd_init();
    fn gen4_lcd_backlight_enable();
    pub fn gen4_lcd_present_rgb565(pixels: *const u16, width: u16, height: u16);
}
