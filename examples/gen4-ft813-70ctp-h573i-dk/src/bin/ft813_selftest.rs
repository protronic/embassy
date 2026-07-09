#![no_std]
#![no_main]

//! FT813 co-processor bring-up smoke test — no LVGL, builds with default
//! features.
//!
//! Drives the FT813 as a real EVE **co-processor** (RAM_CMD FIFO): builds a
//! display list with `CMD_DLSTART … CMD_SWAP`, clears the screen, paints eight
//! colour bars with the `RECTS` primitive and prints a line of text with
//! `CMD_TEXT`, then logs capacitive touch.
//!
//! Bring-up order matters on this board: enable the pixel clock / DISP
//! ([`Ft81x::enable_display`]) as the *last* step, after the final SPI speed
//! and the first frame — otherwise `REG_PCLK` gets cleared again.
//!
//! Note: with the panel actively scanning, the capacitive touch is sensitive
//! to display noise coupling over the flywire hookup (see
//! `gen4-ft813-touch-vs-display-emi` note) — solid grounding matters.
//!
//! ```bash
//! cargo run --release --bin ft813_selftest
//! ```

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::board;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::firmware_id::FIRMWARE_ID;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::ft81x::{
    DISPLAY_HEIGHT, DISPLAY_WIDTH, OPT_CENTER, RECTS, dl_begin, dl_clear, dl_clear_color_rgb, dl_color_rgb, dl_display,
    dl_end, dl_vertex2f, dl_vertex_format,
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

/// Eight vertical bars, 24-bit RGB: white, yellow, cyan, green, magenta, red,
/// blue, black.
const BARS: [(u8, u8, u8); 8] = [
    (0xFF, 0xFF, 0xFF),
    (0xFF, 0xFF, 0x00),
    (0x00, 0xFF, 0xFF),
    (0x00, 0xFF, 0x00),
    (0xFF, 0x00, 0xFF),
    (0xFF, 0x00, 0x00),
    (0x00, 0x00, 0xFF),
    (0x00, 0x00, 0x00),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(board::config());

    info!("gen4-FT813 co-processor self-test on STM32H573I-DK firmware={}", FIRMWARE_ID);

    let mut eve = board::init_ft813(p.SPI2, p.PI1, p.PB15, p.PI2, p.PA3, p.PA8);
    unwrap!(eve.init().await);
    eve.set_spi_frequency(board::SPI_RUN_HZ);

    // Build one frame with the co-processor.
    unwrap!(eve.co_start()); // CMD_DLSTART
    unwrap!(eve.co_cmd(dl_clear_color_rgb(0, 0, 0)));
    unwrap!(eve.co_cmd(dl_clear()));
    unwrap!(eve.co_cmd(dl_vertex_format(0))); // VERTEX2F in whole pixels

    let bar_w = (DISPLAY_WIDTH / BARS.len()) as i16;
    let bottom = (DISPLAY_HEIGHT - 1) as i16;
    unwrap!(eve.co_cmd(dl_begin(RECTS)));
    for (i, &(r, g, b)) in BARS.iter().enumerate() {
        let x0 = i as i16 * bar_w;
        let x1 = if i == BARS.len() - 1 {
            (DISPLAY_WIDTH - 1) as i16
        } else {
            x0 + bar_w - 1
        };
        unwrap!(eve.co_cmd(dl_color_rgb(r, g, b)));
        unwrap!(eve.co_cmd(dl_vertex2f(x0, 0)));
        unwrap!(eve.co_cmd(dl_vertex2f(x1, bottom)));
    }
    unwrap!(eve.co_cmd(dl_end()));

    unwrap!(eve.co_cmd(dl_color_rgb(0xFF, 0xFF, 0xFF)));
    unwrap!(eve.co_text(
        (DISPLAY_WIDTH / 2) as i16,
        (DISPLAY_HEIGHT / 2) as i16,
        30,
        OPT_CENTER,
        "FT813 co-processor OK",
    ));

    unwrap!(eve.co_cmd(dl_display()));
    unwrap!(eve.co_swap()); // CMD_SWAP
    unwrap!(eve.co_run().await); // submit + wait for the FIFO to drain

    // Light up the panel (DISP + pixel clock) and turn on the backlight.
    unwrap!(eve.enable_display());
    unwrap!(eve.set_backlight(96));
    info!("co-processor frame up — touch the panel");

    // Report a press after 2 consecutive touched samples (drops isolated noise
    // spikes), a release after 4 consecutive not-touched samples (a held touch
    // briefly drops out).
    const PRESS_SAMPLES: u8 = 2;
    const RELEASE_SAMPLES: u8 = 4;
    let mut pressed = false;
    let mut press_streak = 0u8;
    let mut release_streak = 0u8;
    loop {
        let t = unwrap!(eve.touch());
        if t.pressed {
            release_streak = 0;
            press_streak = press_streak.saturating_add(1);
            if !pressed && press_streak >= PRESS_SAMPLES {
                pressed = true;
                info!("touch x={} y={}", t.x, t.y);
            }
        } else {
            press_streak = 0;
            release_streak = release_streak.saturating_add(1);
            if pressed && release_streak >= RELEASE_SAMPLES {
                pressed = false;
                info!("touch released");
            }
        }
        Timer::after_millis(10).await;
    }
}
