//! Hello world for the 4D Systems gen4-RP2350-70CT-CLB (RP2350B).
//!
//! Prints a message over RTT once per second and verifies the on-module
//! APS6404L PSRAM (8 MiB on QMI CS1 / GPIO0).
//!
//! ```bash
//! cargo run --bin hello_world
//! ```

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_gen4_rp2350_70ct_clb_examples::gen4_board;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = gen4_board::init();
    info!("gen4-RP2350-70CT-CLB hello world");

    let psram = gen4_board::init_psram(p.QMI_CS1, p.PIN_0);
    info!("PSRAM size: {} bytes", psram.size());

    let mut counter: u32 = 0;
    loop {
        info!("hello from the gen4-RP2350-70CT-CLB! counter={}", counter);
        counter = counter.wrapping_add(1);
        Timer::after_secs(1).await;
    }
}
