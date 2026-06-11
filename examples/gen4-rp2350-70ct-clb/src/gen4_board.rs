//! Board support for the 4D Systems **gen4-RP2350-70CT-CLB** display module.
//!
//! - 7.0" 800×480 TN TFT panel with a parallel RGB565 (DPI) interface driven
//!   from the RP2350B via PIO + DMA (see [`crate::dpi`]).
//! - FocalTech FT5446 capacitive touch controller on `I2C1`.
//! - 16 MiB QSPI flash (W25Q128JVSIQ) on QMI CS0, 8 MiB QSPI PSRAM
//!   (APS6404L-3SQR) on QMI CS1 / `GPIO0` — the framebuffer lives in PSRAM.
//!
//! # Pin map
//!
//! The 30-way FFC user pins (`GPIO1..GPIO9`, `GPIO16`, `GPIO40..GPIO45`) and
//! the PSRAM CS (`GPIO0`), touch (`GPIO38/39/46/47` on `I2C1`) and SD-card
//! pins follow 4D Systems' published gen4-RP2350 board files. The internal
//! LCD pin assignment of the RGB series is not published in the datasheet;
//! the mapping in [`pins`] follows the same family layout (backlight on
//! `GPIO17`, LCD block on `GPIO18..GPIO37`) and is kept in one place so it is
//! easy to adjust against the Graphics4D board file or module schematic.

use defmt::info;
use embassy_rp::gpio::{Level, Output};
#[cfg(feature = "touch")]
use embassy_rp::i2c::{Blocking, Config as I2cConfig, I2c};
use embassy_rp::psram::{Config as PsramConfig, Psram};
use embassy_rp::qmi_cs1::QmiCs1;
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{Peri, Peripherals, bind_interrupts, peripherals};
use embassy_time::{Duration, Timer};

/// Active pixels per line.
pub const DISPLAY_WIDTH: usize = 800;
/// Active lines per frame.
pub const DISPLAY_HEIGHT: usize = 480;

/// FT5446 capacitive touch controller, 7-bit I2C address (FT5x06 family).
#[cfg(feature = "touch")]
pub const TOUCH_I2C_ADDR: u8 = 0x38;

/// gen4-RP2350-70CT-CLB pin assignments.
pub mod pins {
    //! RP2350B GPIO map for the gen4-RP2350-70CT-CLB.
    //!
    //! ```text
    //!              +---------- RP2350B ----------+
    //!              |                             |
    //!  PSRAM CS1 --+ GPIO0 (QMI XIP CS1)         |
    //!  FFC user ---+ GPIO1..9, 16, 40..45        |
    //!  microSD ----+ GPIO10..15 (SDIO via PIO)   |
    //!  LCD --------+ GPIO17..37 (see below)      |
    //!  Touch ------+ GPIO38/39/46/47 (I2C1)      |
    //!              +-----------------------------+
    //! ```

    // --- LCD parallel RGB565 interface (PIO-driven) ---
    /// Backlight enable (family layout: `LCD_BACKLIGHT`).
    pub const LCD_BACKLIGHT: u8 = 17;
    /// First of 16 consecutive RGB565 data pins:
    /// `B0..B4` = GPIO18..22, `G0..G5` = GPIO23..28, `R0..R4` = GPIO29..33.
    pub const LCD_DATA0: u8 = 18;
    /// Data enable — PIO side-set bit 0.
    pub const LCD_DE: u8 = 34;
    /// Pixel clock — PIO side-set bit 1.
    pub const LCD_PCLK: u8 = 35;
    /// HSYNC, parked inactive (the panel runs in DE-only sync mode).
    pub const LCD_HSYNC: u8 = 36;
    /// VSYNC, parked inactive (the panel runs in DE-only sync mode).
    pub const LCD_VSYNC: u8 = 37;

    // --- Capacitive touch (FT5446 on I2C1, gen4 family layout) ---
    pub const TOUCH_INT: u8 = 38;
    pub const TOUCH_SCL: u8 = 39;
    pub const TOUCH_SDA: u8 = 46;
    pub const TOUCH_RST: u8 = 47;

    // --- PSRAM ---
    /// APS6404L chip-select on QMI XIP CS1.
    pub const PSRAM_CS: u8 = 0;

    // --- FFC defaults (Pico SDK board configuration) ---
    pub const UART_TX: u8 = 4; // UART1
    pub const UART_RX: u8 = 5; // UART1
    pub const I2C0_SDA: u8 = 8;
    pub const I2C0_SCL: u8 = 9;
}

bind_interrupts!(pub struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<peripherals::USB>;
});

/// Initialize the RP2350 with the default clock tree (150 MHz `clk_sys`).
pub fn init() -> Peripherals {
    embassy_rp::init(embassy_rp::config::Config::default())
}

/// Create the USB driver for the CDC-ACM debug logger (module USB-C port).
pub fn usb_log_driver(usb: Peri<'static, peripherals::USB>) -> UsbDriver<'static, peripherals::USB> {
    UsbDriver::new(usb, UsbIrqs)
}

/// Expose `log::info!`/`log::warn!`-style messages on the module's USB-C
/// connector as a CDC-ACM serial port (e.g. `/dev/ttyACM0`, 115200 8N1).
///
/// Messages logged before the host opens the port are dropped, so the
/// examples re-log key bring-up facts periodically.
#[embassy_executor::task]
pub async fn usb_logger_task(driver: UsbDriver<'static, peripherals::USB>) {
    embassy_usb_logger::run!(2048, log::LevelFilter::Info, driver);
}

/// Map the on-module APS6404L PSRAM at `0x1100_0000` and return the driver.
///
/// The 800×480 RGB565 framebuffer (~752 KiB plus row prefixes) does not fit
/// in the 520 KiB of SRAM, so it is placed at the start of PSRAM.
///
/// On failure this never returns: it keeps reporting the error on RTT and
/// the USB logger (if running) instead of panicking invisibly.
pub async fn init_psram(
    qmi_cs1: Peri<'static, peripherals::QMI_CS1>,
    cs_pin: Peri<'static, peripherals::PIN_0>,
) -> Psram<'static> {
    match Psram::new(QmiCs1::new(qmi_cs1, cs_pin), PsramConfig::aps6404l()) {
        Ok(psram) => {
            info!("PSRAM mapped: {} bytes at {:#x}", psram.size(), psram.base_address());
            log::info!("PSRAM mapped: {} bytes at {:p}", psram.size(), psram.base_address());
            psram
        }
        Err(_) => loop {
            defmt::error!("APS6404L PSRAM not found (is this a gen4-RP2350 module?)");
            log::error!("APS6404L PSRAM not found (is this a gen4-RP2350 module?)");
            Timer::after(Duration::from_secs(1)).await;
        },
    }
}

/// Write/read-back self-test on a PSRAM window (returns the first failing
/// byte offset on error).
///
/// Spans more than the 16 KiB XIP cache so most read-backs really hit the
/// PSRAM chip. Pick `offset`/`len` outside the framebuffer region.
pub fn psram_self_test(psram: &Psram<'static>, offset: usize, len: usize) -> Result<(), usize> {
    assert!(offset + len <= psram.size());
    let base = psram.base_address().wrapping_add(offset);
    // SAFETY: PSRAM is memory-mapped and the window is inside its size.
    unsafe {
        for i in 0..len {
            core::ptr::write_volatile(base.add(i), (i as u8).wrapping_mul(31).wrapping_add(7));
        }
        for i in 0..len {
            let expected = (i as u8).wrapping_mul(31).wrapping_add(7);
            if core::ptr::read_volatile(base.add(i)) != expected {
                return Err(i);
            }
        }
    }
    Ok(())
}

/// Turn the LCD backlight on (plain GPIO; the module dims via PWM if needed).
pub fn init_backlight(pin: Peri<'static, peripherals::PIN_17>) -> Output<'static> {
    Output::new(pin, Level::High)
}

/// Park HSYNC/VSYNC at their inactive (high) level — DE-only sync mode.
pub fn park_sync_pins(
    hsync: Peri<'static, peripherals::PIN_36>,
    vsync: Peri<'static, peripherals::PIN_37>,
) -> (Output<'static>, Output<'static>) {
    (Output::new(hsync, Level::High), Output::new(vsync, Level::High))
}

/// Resources used by the touch controller.
#[cfg(feature = "touch")]
pub struct TouchResources {
    pub i2c: I2c<'static, peripherals::I2C1, Blocking>,
    /// FT5446 interrupt line (active low while a contact is present).
    pub int: embassy_rp::gpio::Input<'static>,
}

/// Reset the FT5446 and bring up its I2C bus (`I2C1` on GPIO46/GPIO39).
#[cfg(feature = "touch")]
pub async fn init_touch(
    i2c1: Peri<'static, peripherals::I2C1>,
    sda: Peri<'static, peripherals::PIN_46>,
    scl: Peri<'static, peripherals::PIN_39>,
    rst: Peri<'static, peripherals::PIN_47>,
    int: Peri<'static, peripherals::PIN_38>,
) -> TouchResources {
    // FT5x06 reset pulse: ≥5 ms low, ≥300 ms to first valid report.
    let mut touch_reset = Output::new(rst, Level::High);
    Timer::after(Duration::from_millis(10)).await;
    touch_reset.set_low();
    Timer::after(Duration::from_millis(10)).await;
    touch_reset.set_high();
    Timer::after(Duration::from_millis(300)).await;
    core::mem::forget(touch_reset); // keep the pin driven high forever

    let mut config = I2cConfig::default();
    config.frequency = 400_000;
    let i2c = I2c::new_blocking(i2c1, scl, sda, config);
    let int = embassy_rp::gpio::Input::new(int, embassy_rp::gpio::Pull::Up);
    info!("FT5446 touch reset done, I2C1 up");
    TouchResources { i2c, int }
}

/// One decoded touch sample.
#[derive(Clone, Copy, Default)]
pub struct TouchPoint {
    pub x: u16,
    pub y: u16,
    pub pressed: bool,
    /// `true` when the last I2C read succeeded.
    pub i2c_ok: bool,
    /// `TD_STATUS` register (active touch point count), for debug logging.
    pub raw_status: u8,
}

/// Read the first touch point from the FT5446 (FT5x06 register layout).
///
/// Registers `0x00..0x06`: device mode, gesture, `TD_STATUS` (point count),
/// then P1 `XH/XL/YH/YL` with 12-bit coordinates.
#[cfg(feature = "touch")]
pub fn read_touch(i2c: &mut I2c<'static, peripherals::I2C1, Blocking>) -> TouchPoint {
    let mut data = [0u8; 7];
    match i2c.blocking_write_read(TOUCH_I2C_ADDR as u16, &[0x00], &mut data) {
        Ok(()) => {
            let raw_status = data[2] & 0x0F;
            let x = (u16::from(data[3] & 0x0F) << 8) | u16::from(data[4]);
            let y = (u16::from(data[5] & 0x0F) << 8) | u16::from(data[6]);
            let pressed = raw_status > 0 && raw_status < 6;
            TouchPoint {
                x: x.min(DISPLAY_WIDTH as u16 - 1),
                y: y.min(DISPLAY_HEIGHT as u16 - 1),
                pressed,
                i2c_ok: true,
                raw_status,
            }
        }
        Err(_) => TouchPoint::default(),
    }
}
