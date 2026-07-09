#![no_std]
#![no_main]

//! FT813 co-processor bring-up smoke test — no LVGL, builds with default
//! features.
//!
//! Drives the FT813 as a real EVE **co-processor** (RAM_CMD FIFO): each frame
//! is a display list built with `CMD_DLSTART … CMD_SWAP` that clears the
//! screen, paints eight colour bars (`RECTS`), prints a line of text
//! (`CMD_TEXT`) and — while the panel is touched — draws a marker (`POINTS`)
//! at the touch position, so you can see the touch land on screen.
//!
//! ```bash
//! cargo run --release --bin ft813_selftest
//! ```
//!
//! Note: touch coordinates are uncalibrated (raw), so the marker tracks the
//! finger only roughly; and with the panel scanning, capacitive touch is
//! sensitive to display-noise coupling over the flywire hookup (solid ground
//! matters). See the `gen4-ft813-touch-vs-display-emi` note.

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::board;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::firmware_id::FIRMWARE_ID;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::ft81x::{
    DISPLAY_HEIGHT, DISPLAY_WIDTH, Error, Ft81x, OPT_CENTER, POINTS, RECTS, dl_begin, dl_clear, dl_clear_color_rgb,
    dl_color_rgb, dl_display, dl_end, dl_point_size, dl_vertex2f, dl_vertex_format,
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

/// Build and swap one co-processor frame: colour bars + text, plus a touch
/// marker at `mark` (panel pixels) when `Some`.
async fn draw_frame(eve: &mut Ft81x, mark: Option<(i16, i16)>) -> Result<(), Error> {
    eve.co_start()?; // CMD_DLSTART
    eve.co_cmd(dl_clear_color_rgb(0, 0, 0))?;
    eve.co_cmd(dl_clear())?;
    eve.co_cmd(dl_vertex_format(0))?; // VERTEX2F in whole pixels

    let bar_w = (DISPLAY_WIDTH / BARS.len()) as i16;
    let bottom = (DISPLAY_HEIGHT - 1) as i16;
    eve.co_cmd(dl_begin(RECTS))?;
    for (i, &(r, g, b)) in BARS.iter().enumerate() {
        let x0 = i as i16 * bar_w;
        let x1 = if i == BARS.len() - 1 {
            (DISPLAY_WIDTH - 1) as i16
        } else {
            x0 + bar_w - 1
        };
        eve.co_cmd(dl_color_rgb(r, g, b))?;
        eve.co_cmd(dl_vertex2f(x0, 0))?;
        eve.co_cmd(dl_vertex2f(x1, bottom))?;
    }
    eve.co_cmd(dl_end())?;

    eve.co_cmd(dl_color_rgb(0xFF, 0xFF, 0xFF))?;
    eve.co_text(
        (DISPLAY_WIDTH / 2) as i16,
        (DISPLAY_HEIGHT / 2) as i16,
        30,
        OPT_CENTER,
        "FT813 co-processor OK",
    )?;

    // Touch marker: white ring with a red centre so it shows on any bar.
    if let Some((x, y)) = mark {
        eve.co_cmd(dl_begin(POINTS))?;
        eve.co_cmd(dl_color_rgb(0xFF, 0xFF, 0xFF))?;
        eve.co_cmd(dl_point_size(224))?; // ~14 px
        eve.co_cmd(dl_vertex2f(x, y))?;
        eve.co_cmd(dl_color_rgb(0xFF, 0x00, 0x00))?;
        eve.co_cmd(dl_point_size(112))?; // ~7 px centre
        eve.co_cmd(dl_vertex2f(x, y))?;
        eve.co_cmd(dl_end())?;
    }

    eve.co_cmd(dl_display())?;
    eve.co_swap()?; // CMD_SWAP
    eve.co_run().await // submit + wait for the FIFO to drain
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(board::config());

    info!("gen4-FT813 co-processor self-test on STM32H573I-DK firmware={}", FIRMWARE_ID);

    let mut eve = board::init_ft813(p.SPI2, p.PI1, p.PB15, p.PI2, p.PA3, p.PA8);
    unwrap!(eve.init().await);
    eve.set_spi_frequency(board::SPI_RUN_HZ);

    // First frame (no marker), then light up the panel and backlight.
    unwrap!(draw_frame(&mut eve, None).await);
    unwrap!(eve.enable_display());
    unwrap!(eve.set_backlight(96));
    unwrap!(eve.log_timing()); // verify the timings held through bring-up
    let (mode, ext) = unwrap!(eve.touch_config());
    info!("touch cfg: REG_TOUCH_MODE={} REG_CTOUCH_EXTENDED={} (expect 3, 1)", mode, ext);
    info!("co-processor frame up — touch the panel");

    // Debounce press/release; while pressed, draw the marker at the latest
    // valid touch position and redraw only when it moves.
    const PRESS_SAMPLES: u8 = 2;
    const RELEASE_SAMPLES: u8 = 4;
    let mut pressed = false;
    let mut press_streak = 0u8;
    let mut release_streak = 0u8;
    let mut last_pos = (0i16, 0i16);
    let mut shown: Option<(i16, i16)> = None;
    loop {
        let t = unwrap!(eve.touch());
        if t.pressed {
            last_pos = (t.x as i16, t.y as i16);
            release_streak = 0;
            press_streak = press_streak.saturating_add(1);
            if !pressed && press_streak >= PRESS_SAMPLES {
                pressed = true;
            }
        } else {
            press_streak = 0;
            release_streak = release_streak.saturating_add(1);
            if pressed && release_streak >= RELEASE_SAMPLES {
                pressed = false;
            }
        }

        let want = if pressed { Some(last_pos) } else { None };
        if want != shown {
            unwrap!(draw_frame(&mut eve, want).await);
            match want {
                Some((x, y)) => info!("touch x={} y={}", x, y),
                None => info!("touch released"),
            }
            shown = want;
        }
        Timer::after_millis(15).await;
    }
}
