//! Board support for the 4D Systems **gen4-RP2350-70CT-CLB** intelligent display.
//!
//! - **MCU**: Raspberry Pi RP2350B
//! - **Panel**: 7.0" 800×480 RGB565 parallel RGB (`GEN4_RP2350_RGB`)
//! - **Touch**: FocalTech FT5446 (capacitive), I2C1
//! - **PSRAM**: APS6404L 8 MiB on QMI CS1 / GPIO0
//!
//! Pin assignments follow the Pico SDK board file `board/gen4_rp2350_70ct.h`
//! (CLB variants use the same GPIO map). When Graphics4D is linked it owns
//! RGB scan-out; otherwise Embassy PIO+DPI drives the panel (see [`crate::dpi`]).

use defmt::info;
use embassy_executor::Spawner;
use log::info as uinfo;
use embassy_rp::gpio::{Level, Output};
#[cfg(feature = "touch")]
use embassy_rp::gpio::{Input, Pull};
#[cfg(feature = "touch")]
use embassy_rp::i2c::{Blocking, Config as I2cConfig, I2c};
use embassy_rp::peripherals::{self as peripherals, PIN_17, PWM_SLICE0};
#[cfg(feature = "touch")]
use embassy_rp::peripherals::PIN_47;
use embassy_rp::psram::{self, Psram};
use embassy_rp::pwm::{Config as PwmConfig, Pwm, SetDutyCycle};
use embassy_rp::qmi_cs1::QmiCs1;
use embassy_rp::{Peri, Peripherals};

use embassy_time::{Duration, Timer};

/// Pin map from `board/gen4_rp2350_70ct.h` (Pico SDK / Workshop5).
pub mod pins {
    pub const LCD_BACKLIGHT: u8 = 17;
    pub const LCD_DE: u8 = 18;
    pub const LCD_VSYNC: u8 = 19;
    pub const LCD_HSYNC: u8 = 20;
    pub const LCD_PCLK: u8 = 21;
    /// First of 16 consecutive RGB565 data lines (`LCD_DATA0_PIN` … `GPIO37`).
    pub const LCD_DATA0: u8 = 22;
    pub const LCD_DATA15: u8 = 37;
    pub const LCD_CLK_FREQ_HZ: u32 = 25_000_000;

    pub const TOUCH_INT: u8 = 38;
    pub const TOUCH_SCL: u8 = 39;
    pub const TOUCH_SDA: u8 = 46;
    pub const TOUCH_RST: u8 = 47;

    pub const PSRAM_CS: u8 = 0;
}

pub const DISPLAY_WIDTH: usize = 800;
pub const DISPLAY_HEIGHT: usize = 480;

/// FT5446 default 7-bit I2C address on gen4-RP2350 CT panels.
#[cfg(feature = "touch")]
pub const TOUCH_I2C_ADDR: u8 = 0x38;

#[cfg(gen4_graphics4d)]
const FB_BYTES: usize = DISPLAY_WIDTH * DISPLAY_HEIGHT * 2;
#[cfg(not(gen4_graphics4d))]
const FB_BYTES: usize = crate::dpi::FRAME_BYTES;

/// PSRAM-backed framestores for LVGL / panel scanout.
pub struct PsramFramebuffers {
    pub back: *mut u8,
    #[cfg(gen4_graphics4d)]
    pub front: *mut u8,
    pub bytes: usize,
}

impl PsramFramebuffers {
    /// Reserve framebuffer(s) at the base of PSRAM (Embassy DPI path only).
    ///
    /// # Safety
    /// `psram` must be initialised and `base_address()` must stay valid.
    #[cfg(not(gen4_graphics4d))]
    pub unsafe fn new(psram: &Psram<'_>) -> Option<Self> {
        let base = psram.base_address();
        let total = psram.size() as usize;
        if total < FB_BYTES {
            return None;
        }
        Some(Self {
            back: base.cast::<u8>(),
            bytes: FB_BYTES,
        })
    }

    /// LVGL draw buffers in PSRAM after Graphics4D has claimed the low region.
    ///
    /// Graphics4D `Initialize()` sets up the QMI PSRAM heap at `0x1100_0000` and
    /// allocates its own scan-out framestores there. Placing LVGL buffers at the
    /// same base corrupts that heap and typically hard-faults during panel init.
    ///
    /// # Safety
    /// `gen4_lcd_init()` / `gfx.Initialize()` must have returned successfully.
    #[cfg(gen4_graphics4d)]
    pub unsafe fn new_after_graphics4d() -> Option<Self> {
        const PSRAM_BASE: usize = 0x1100_0000;
        /// Room for Graphics4D TLSF heap + internal RGB/aux FBs + DMA bounce (~1.6 MiB).
        const GFX4D_PSRAM_RESERVE: usize = 2 * 1024 * 1024;
        const PSRAM_SIZE: usize = 8 * 1024 * 1024;

        let fb_base = PSRAM_BASE + GFX4D_PSRAM_RESERVE;
        if fb_base + FB_BYTES * 2 > PSRAM_BASE + PSRAM_SIZE {
            return None;
        }
        let back = fb_base as *mut u8;
        Some(Self {
            back,
            front: back.add(FB_BYTES),
            bytes: FB_BYTES,
        })
    }
}

/// PIO/DMA peripherals for Embassy DPI scan-out (when Graphics4D is not linked).
#[cfg(not(gen4_graphics4d))]
pub struct DpiPeripherals {
    pub pio0: Peri<'static, peripherals::PIO0>,
    pub dma_ch0: Peri<'static, peripherals::DMA_CH0>,
    pub de: Peri<'static, peripherals::PIN_18>,
    pub pclk: Peri<'static, peripherals::PIN_21>,
    pub d0: Peri<'static, peripherals::PIN_22>,
    pub d1: Peri<'static, peripherals::PIN_23>,
    pub d2: Peri<'static, peripherals::PIN_24>,
    pub d3: Peri<'static, peripherals::PIN_25>,
    pub d4: Peri<'static, peripherals::PIN_26>,
    pub d5: Peri<'static, peripherals::PIN_27>,
    pub d6: Peri<'static, peripherals::PIN_28>,
    pub d7: Peri<'static, peripherals::PIN_29>,
    pub d8: Peri<'static, peripherals::PIN_30>,
    pub d9: Peri<'static, peripherals::PIN_31>,
    pub d10: Peri<'static, peripherals::PIN_32>,
    pub d11: Peri<'static, peripherals::PIN_33>,
    pub d12: Peri<'static, peripherals::PIN_34>,
    pub d13: Peri<'static, peripherals::PIN_35>,
    pub d14: Peri<'static, peripherals::PIN_36>,
    pub d15: Peri<'static, peripherals::PIN_37>,
}

pub struct DisplayResources {
    #[cfg(not(gen4_graphics4d))]
    pub psram: Psram<'static>,
    pub framebuffers: PsramFramebuffers,
    #[cfg(not(gen4_graphics4d))]
    pub dpi: Option<DpiPeripherals>,
    #[cfg(feature = "touch")]
    pub i2c: I2c<'static, peripherals::I2C1, Blocking>,
}

/// Log whether the Graphics4D panel driver is linked.
pub fn log_panel_driver_status() {
    #[cfg(gen4_graphics4d)]
    uinfo!("panel: Graphics4D linked — RGB scanout active");
    #[cfg(not(gen4_graphics4d))]
    uinfo!("panel: Embassy PIO+DPI scanout (GPIO22..37 data, DE=18, PCLK=21)");
}

/// Initialise PSRAM, panel, backlight, USB logging, and optional FT5446 touch.
pub async fn init(spawner: &Spawner, p: Peripherals) -> Option<DisplayResources> {
    let Peripherals {
        USB,
        #[cfg(not(gen4_graphics4d))]
        QMI_CS1,
        #[cfg(not(gen4_graphics4d))]
        PIN_0,
        #[cfg(not(gen4_graphics4d))]
        PIN_17,
        #[cfg(not(gen4_graphics4d))]
        PWM_SLICE0,
        #[cfg(not(gen4_graphics4d))]
        PIO0,
        #[cfg(not(gen4_graphics4d))]
        DMA_CH0,
        #[cfg(not(gen4_graphics4d))]
        PIN_18,
        #[cfg(not(gen4_graphics4d))]
        PIN_19,
        #[cfg(not(gen4_graphics4d))]
        PIN_20,
        #[cfg(not(gen4_graphics4d))]
        PIN_21,
        #[cfg(not(gen4_graphics4d))]
        PIN_22,
        #[cfg(not(gen4_graphics4d))]
        PIN_23,
        #[cfg(not(gen4_graphics4d))]
        PIN_24,
        #[cfg(not(gen4_graphics4d))]
        PIN_25,
        #[cfg(not(gen4_graphics4d))]
        PIN_26,
        #[cfg(not(gen4_graphics4d))]
        PIN_27,
        #[cfg(not(gen4_graphics4d))]
        PIN_28,
        #[cfg(not(gen4_graphics4d))]
        PIN_29,
        #[cfg(not(gen4_graphics4d))]
        PIN_30,
        #[cfg(not(gen4_graphics4d))]
        PIN_31,
        #[cfg(not(gen4_graphics4d))]
        PIN_32,
        #[cfg(not(gen4_graphics4d))]
        PIN_33,
        #[cfg(not(gen4_graphics4d))]
        PIN_34,
        #[cfg(not(gen4_graphics4d))]
        PIN_35,
        #[cfg(not(gen4_graphics4d))]
        PIN_36,
        #[cfg(not(gen4_graphics4d))]
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

    #[cfg(gen4_graphics4d)]
    let framebuffers = {
        // Graphics4D owns QMI/PSRAM setup and panel PIO. Do not call embassy Psram::new()
        // first — double QMI programming is a common hard-fault source.
        // SAFETY: C shim calls into the linked Graphics4D library.
        unsafe {
            gen4_lcd_init();
            gen4_lcd_backlight_enable();
        }
        uinfo!("board init: gen4_lcd_init() returned (Graphics4D PSRAM + panel)");

        // SAFETY: Graphics4D finished PSRAM/panel init.
        let framebuffers = unsafe { PsramFramebuffers::new_after_graphics4d()? };
        uinfo!(
            "board init: LVGL framebuffers {}x{} RGB565 x2 @ PSRAM+2MiB ({} KiB each)",
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT,
            FB_BYTES / 1024
        );
        info!(
            "gen4 Graphics4D ready: LVGL framebuffers {}x{} RGB565 x2",
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT
        );
        framebuffers
    };

    #[cfg(not(gen4_graphics4d))]
    let psram = Psram::new(
        QmiCs1::new(QMI_CS1, PIN_0),
        psram::Config::aps6404l(),
    )
    .ok()?;
    #[cfg(not(gen4_graphics4d))]
    uinfo!("board init: PSRAM OK ({} KiB)", psram.size() / 1024);

    #[cfg(not(gen4_graphics4d))]
    let framebuffers = {
        // SAFETY: PSRAM is mapped and sized by the driver.
        let framebuffers = unsafe { PsramFramebuffers::new(&psram)? };
        uinfo!(
            "board init: framebuffer {}x{} DPI ({} KiB)",
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT,
            FB_BYTES / 1024
        );
        info!(
            "gen4 PSRAM ready: {} KiB, framebuffers {}x{} DPI",
            psram.size() / 1024,
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT
        );
        init_backlight(PWM_SLICE0, PIN_17);
        uinfo!("board init: backlight PWM on GPIO{}", pins::LCD_BACKLIGHT);
        let _sync = park_sync_pins(PIN_20, PIN_19);
        uinfo!("board init: HSYNC/VSYNC parked (DE-only mode)");
        framebuffers
    };

    #[cfg(feature = "touch")]
    {
        reset_touch(PIN_47).await;
        let i2c = I2c::new_blocking(I2C1, PIN_39, PIN_46, I2cConfig::default());
        let _touch_int = Input::new(PIN_38, Pull::Up);
        info!(
            "FT5446 touch on I2C1 (SCL=GPIO{}, SDA=GPIO{})",
            pins::TOUCH_SCL,
            pins::TOUCH_SDA
        );
        uinfo!("board init: FT5446 touch on I2C1");
        return Some(DisplayResources {
            #[cfg(not(gen4_graphics4d))]
            psram,
            framebuffers,
            #[cfg(not(gen4_graphics4d))]
            dpi: Some(make_dpi_peripherals(
                PIO0, DMA_CH0, PIN_18, PIN_21, PIN_22, PIN_23, PIN_24, PIN_25, PIN_26, PIN_27,
                PIN_28, PIN_29, PIN_30, PIN_31, PIN_32, PIN_33, PIN_34, PIN_35, PIN_36, PIN_37,
            )),
            i2c,
        });
    }

    #[cfg(not(feature = "touch"))]
    {
        uinfo!("board init: complete (no touch feature)");
        Some(DisplayResources {
            #[cfg(not(gen4_graphics4d))]
            psram,
            framebuffers,
            #[cfg(not(gen4_graphics4d))]
            dpi: Some(make_dpi_peripherals(
                PIO0, DMA_CH0, PIN_18, PIN_21, PIN_22, PIN_23, PIN_24, PIN_25, PIN_26, PIN_27,
                PIN_28, PIN_29, PIN_30, PIN_31, PIN_32, PIN_33, PIN_34, PIN_35, PIN_36, PIN_37,
            )),
        })
    }
}

#[cfg(not(gen4_graphics4d))]
fn make_dpi_peripherals(
    pio0: Peri<'static, peripherals::PIO0>,
    dma_ch0: Peri<'static, peripherals::DMA_CH0>,
    de: Peri<'static, peripherals::PIN_18>,
    pclk: Peri<'static, peripherals::PIN_21>,
    d0: Peri<'static, peripherals::PIN_22>,
    d1: Peri<'static, peripherals::PIN_23>,
    d2: Peri<'static, peripherals::PIN_24>,
    d3: Peri<'static, peripherals::PIN_25>,
    d4: Peri<'static, peripherals::PIN_26>,
    d5: Peri<'static, peripherals::PIN_27>,
    d6: Peri<'static, peripherals::PIN_28>,
    d7: Peri<'static, peripherals::PIN_29>,
    d8: Peri<'static, peripherals::PIN_30>,
    d9: Peri<'static, peripherals::PIN_31>,
    d10: Peri<'static, peripherals::PIN_32>,
    d11: Peri<'static, peripherals::PIN_33>,
    d12: Peri<'static, peripherals::PIN_34>,
    d13: Peri<'static, peripherals::PIN_35>,
    d14: Peri<'static, peripherals::PIN_36>,
    d15: Peri<'static, peripherals::PIN_37>,
) -> DpiPeripherals {
    DpiPeripherals {
        pio0,
        dma_ch0,
        de,
        pclk,
        d0,
        d1,
        d2,
        d3,
        d4,
        d5,
        d6,
        d7,
        d8,
        d9,
        d10,
        d11,
        d12,
        d13,
        d14,
        d15,
    }
}

/// Park HSYNC/VSYNC at inactive (high) — DE-only sync mode.
#[cfg(not(gen4_graphics4d))]
pub fn park_sync_pins(
    hsync: Peri<'static, peripherals::PIN_20>,
    vsync: Peri<'static, peripherals::PIN_19>,
) -> (Output<'static>, Output<'static>) {
    (Output::new(hsync, Level::High), Output::new(vsync, Level::High))
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
    info!("backlight PWM enabled on GPIO{}", pins::LCD_BACKLIGHT);
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

    // `LCD_TOUCH_SWAP_XY` in gen4_rp2350_70ct.h
    core::mem::swap(&mut x, &mut y);
    x = x.min(DISPLAY_WIDTH as u16 - 1);
    y = y.min(DISPLAY_HEIGHT as u16 - 1);

    TouchPoint {
        x,
        y,
        pressed: true,
        raw_status,
    }
}

#[cfg(gen4_graphics4d)]
unsafe extern "C" {
    fn gen4_lcd_init();
    fn gen4_lcd_backlight_enable();
}

unsafe extern "C" {
    pub fn gen4_lcd_present_rgb565(pixels: *const u16, width: u16, height: u16);
}
