#![no_std]
#![no_main]

//! FT813 bring-up smoke test — no LVGL, builds with default features.
//!
//! Initialises the gen4-FT813-70CTP-CLB over SPI, paints colour bars into the
//! `RAM_G` framebuffer and then logs capacitive touch samples. If the panel
//! shows eight vertical colour bars and touches appear in the defmt log, the
//! wiring and the FT813 init are good — the OxivGL demo will work on top.
//!
//! ```bash
//! cargo run --release --bin ft813_selftest
//! ```

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::board;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::firmware_id::FIRMWARE_ID;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::ft81x::{DISPLAY_HEIGHT, DISPLAY_WIDTH, LINE_STRIDE, RAM_G};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

/// Eight vertical bars: white, yellow, cyan, green, magenta, red, blue, black.
const BAR_COLORS: [u16; 8] = [0xFFFF, 0xFFE0, 0x07FF, 0x07E0, 0xF81F, 0xF800, 0x001F, 0x0000];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(board::config());

    info!("gen4-FT813 self-test on STM32H573I-DK firmware={}", FIRMWARE_ID);

    let mut eve = board::init_ft813(p.SPI2, p.PI1, p.PB15, p.PI2, p.PA3, p.PA8);
    unwrap!(eve.init().await);
    eve.set_spi_frequency(board::SPI_RUN_HZ);

    // One line of colour bars, repeated for every row.
    let mut line = [0u8; LINE_STRIDE];
    let bar_w = DISPLAY_WIDTH / BAR_COLORS.len();
    for (x, px) in line.chunks_exact_mut(2).enumerate() {
        let color = BAR_COLORS[(x / bar_w).min(BAR_COLORS.len() - 1)];
        px.copy_from_slice(&color.to_le_bytes());
    }
    for y in 0..DISPLAY_HEIGHT {
        unwrap!(eve.wr_bytes(RAM_G + (y * LINE_STRIDE) as u32, &line));
    }

    unwrap!(eve.show_framebuffer());
    unwrap!(eve.set_backlight(96));
    info!("colour bars up — touch the panel");

    let mut was_pressed = false;
    loop {
        let t = unwrap!(eve.touch());
        if t.pressed {
            info!("touch x={} y={}", t.x, t.y);
        } else if was_pressed {
            info!("touch released");
        }
        was_pressed = t.pressed;
        Timer::after_millis(50).await;
    }
}
