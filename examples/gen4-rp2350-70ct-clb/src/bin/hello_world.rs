//! Hello world for the 4D Systems gen4-RP2350-70CT-CLB (RP2350B).
//!
//! Prints once per second over **USB CDC serial** (visible in `picocom` /
//! `minicom` on `/dev/ttyACM*`) and verifies the on-module APS6404L PSRAM
//! (8 MiB on QMI CS1 / GPIO0).
//!
//! Flash via USB bootloader (BOOTSEL → `2e8a:000f`):
//!
//! ```bash
//! cargo run --bin hello_world
//! ```
//!
//! Requires `picotool` as the cargo runner (see `.cargo/config.toml`).

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_gen4_rp2350_70ct_clb_examples::gen4_board;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::Timer;
use log::info;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = gen4_board::init();
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver).unwrap());

    info!("gen4-RP2350-70CT-CLB hello world (USB serial)");

    let psram = gen4_board::init_psram(p.QMI_CS1, p.PIN_0);
    info!("PSRAM size: {} bytes", psram.size());

    let mut counter: u32 = 0;
    loop {
        info!("hello from the gen4-RP2350-70CT-CLB! counter={}", counter);
        counter = counter.wrapping_add(1);
        Timer::after_secs(1).await;
    }
}
