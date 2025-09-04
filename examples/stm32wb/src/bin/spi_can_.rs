#![no_std]
#![no_main]
extern crate alloc;

use alloc::vec;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi;
use embassy_stm32::time::mhz;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use mcp25xxfd::registers::*;
use mcp25xxfd::{Frame, Instruction, MCP25xxFD};

use embedded_can::nb::Can;
use embedded_can::{Frame, Id, StandardId};
use mcp25xxfd::config::Config;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");
    // CN4 Digital D7 (ARD_D7)
    let mut led = Output::new(p.PB5, Level::High, Speed::Low);

    let mut spi_config = spi::Config::default();
    spi_config.frequency = mhz(1);

    let spi = spi::Spi::new_blocking(p.SPI3, p.PB3, p.PB5, p.PB4, spi_config);

    let mut mcp25xx = MCP25xxFD::new(spi);

    let config = Config::default()
        //.mode(OperationMode::Normal)
        //.bitrate(CNF_500K_BPS)
        //.receive_buffer_0(RXB0CTRL::default().with_rxm(RXM::ReceiveAny))
    ;


    mcp25xx.apply_config(&config).unwrap();

    // Send a frame
    let can_id = StandardId::new(123).unwrap();
    let data = [1, 2, 3, 4, 5, 6, 7, 8];
    let frame = Frame::new(can_id, &data).unwrap();
    mcp25xx.transmit(&frame).unwrap();

    // Receive a frame
    if let Ok(frame) = mcp25xx.try_receive() {
        let _can_id = frame.id();
        let _data = frame.data();
    }

    loop {
        info!("high");
        led.set_high();
        Timer::after_millis(500).await;

        info!("low");
        led.set_low();
        Timer::after_millis(500).await;
    }
}
