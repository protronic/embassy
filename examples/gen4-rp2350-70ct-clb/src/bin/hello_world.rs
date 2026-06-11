//! Hello world for the 4D Systems gen4-RP2350-70CT-CLB (RP2350B).
//!
//! Prints once per second over **USB CDC serial** (visible in `picocom` /
//! `minicom` on `/dev/ttyACM*`) and over RTT (defmt), and verifies the
//! on-module APS6404L PSRAM (8 MiB on QMI CS1 / GPIO0) with a
//! write/read-back self-test.
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

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_rp2350_70ct_clb_examples::gen4_board;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = gen4_board::init();
    info!("gen4-RP2350-70CT-CLB hello world");

    // USB CDC logger on the module's USB-C port.
    spawner.spawn(unwrap!(gen4_board::usb_logger_task(gen4_board::usb_log_driver(p.USB))));
    log::info!("gen4-RP2350-70CT-CLB hello world (USB serial)");

    let psram = gen4_board::init_psram(p.QMI_CS1, p.PIN_0).await;
    let psram_test = gen4_board::psram_self_test(&psram, 4 * 1024 * 1024, 64 * 1024);
    match psram_test {
        Ok(()) => info!("PSRAM self-test OK"),
        Err(at) => defmt::error!("PSRAM self-test FAILED at byte {}", at),
    }

    let mut counter: u32 = 0;
    loop {
        info!("hello from the gen4-RP2350-70CT-CLB! counter={}", counter);
        log::info!(
            "hello from the gen4-RP2350-70CT-CLB! counter={} psram={}B selftest={}",
            counter,
            psram.size(),
            if psram_test.is_ok() { "OK" } else { "FAILED" }
        );
        counter = counter.wrapping_add(1);
        Timer::after_secs(1).await;
    }
}
