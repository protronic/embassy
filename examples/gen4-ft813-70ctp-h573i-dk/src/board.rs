//! STM32H573I-DK bring-up for the gen4-FT813-70CTP-CLB on the Arduino header.
//!
//! Wiring (gen4-PA / gen4-IB breakout → Arduino R3 header, matching the
//! Zephyr `stm32h573i_dk` device tree):
//!
//! | gen4 FFC signal | Arduino | MCU pin | Function      |
//! |-----------------|---------|---------|---------------|
//! | SCK             | D13     | PI1     | SPI2_SCK      |
//! | MISO (SDO)      | D12     | PI2     | SPI2_MISO     |
//! | MOSI (SDI)      | D11     | PB15    | SPI2_MOSI     |
//! | /CS             | D10     | PA3     | GPIO out      |
//! | PD (power-down) | D9      | PA8     | GPIO out      |
//! | INT             | D8      | PG8     | unused (poll) |
//!
//! Power: the gen4 module needs 5 V (backlight boost); logic is 3.3 V.

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{PA3, PA8, PB15, PI1, PI2, SPI2};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, Peri};

use crate::ft81x::Ft81x;
pub use crate::ft81x::{DISPLAY_HEIGHT, DISPLAY_WIDTH};

/// SPI clock during FT81x power-up (must stay ≤ 11 MHz until the EVE core
/// runs; 250 MHz kernel clock / 32 = 7.8125 MHz).
pub const SPI_INIT_HZ: u32 = 8_000_000;
/// SPI clock after init. FT81x allows up to 30 MHz; with the 250 MHz SPI2
/// kernel clock the divider lands on 250/16 = 15.625 MHz on the wire.
pub const SPI_RUN_HZ: u32 = 30_000_000;

/// RCC setup: HSE 25 MHz crystal → PLL1 → 250 MHz sysclk (voltage scale 0).
/// PLL1Q also runs at 250 MHz and feeds SPI2 (reset kernel-clock mux).
pub fn config() -> Config {
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale,
    };

    let mut config = Config::default();
    config.rcc.hsi = None;
    config.rcc.hse = Some(Hse {
        freq: Hertz(25_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll1 = Some(Pll {
        source: PllSource::Hse,
        prediv: PllPreDiv::Div5,  // 25 MHz / 5 = 5 MHz ref
        mul: PllMul::Mul100,      // 500 MHz VCO
        divp: Some(PllDiv::Div2), // 250 MHz sysclk
        divq: Some(PllDiv::Div2), // 250 MHz SPI2 kernel clock
        divr: None,
    });
    config.rcc.ahb_pre = AHBPrescaler::Div1;
    config.rcc.apb1_pre = APBPrescaler::Div1;
    config.rcc.apb2_pre = APBPrescaler::Div1;
    config.rcc.apb3_pre = APBPrescaler::Div1;
    config.rcc.sys = Sysclk::Pll1P;
    config.rcc.voltage_scale = VoltageScale::Scale0;
    config
}

/// Set up SPI2 + CS/PD pins and return the (not yet initialised) FT813
/// driver. Call [`Ft81x::init`] next.
pub fn init_ft813(
    spi: Peri<'static, SPI2>,
    sck: Peri<'static, PI1>,
    mosi: Peri<'static, PB15>,
    miso: Peri<'static, PI2>,
    cs: Peri<'static, PA3>,
    pd: Peri<'static, PA8>,
) -> Ft81x {
    let mut cfg = spi::Config::default();
    cfg.frequency = Hertz(SPI_INIT_HZ);

    let spi = Spi::new_blocking(spi, sck, mosi, miso, cfg);
    let cs = Output::new(cs, Level::High, Speed::VeryHigh);
    let pd = Output::new(pd, Level::Low, Speed::Low);

    Ft81x::new(spi, cs, pd, cfg)
}
