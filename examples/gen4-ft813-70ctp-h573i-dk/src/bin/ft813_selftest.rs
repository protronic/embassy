#![no_std]
#![no_main]

//! FT813 co-processor bring-up smoke test — no LVGL, builds with default
//! features.
//!
//! ```bash
//! cargo run --release --bin ft813_selftest
//! ```

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

async fn draw_frame(eve: &mut Ft81x, mark: Option<(i16, i16)>) -> Result<(), Error> {
    eve.co_start()?;
    eve.co_cmd(dl_clear_color_rgb(0, 0, 0))?;
    eve.co_cmd(dl_clear())?;
    eve.co_cmd(dl_vertex_format(0))?;

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

    if let Some((x, y)) = mark {
        eve.co_cmd(dl_begin(POINTS))?;
        eve.co_cmd(dl_color_rgb(0xFF, 0xFF, 0xFF))?;
        eve.co_cmd(dl_point_size(224))?;
        eve.co_cmd(dl_vertex2f(x, y))?;
        eve.co_cmd(dl_color_rgb(0xFF, 0x00, 0x00))?;
        eve.co_cmd(dl_point_size(112))?;
        eve.co_cmd(dl_vertex2f(x, y))?;
        eve.co_cmd(dl_end())?;
    }

    eve.co_cmd(dl_display())?;
    eve.co_swap()?;
    eve.co_run().await
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(board::config());

    info!("gen4-FT813 co-processor self-test on STM32H573I-DK firmware={}", FIRMWARE_ID);

    let mut eve = board::init_ft813(p.SPI2, p.PI1, p.PB15, p.PI2, p.PA3, p.PA8);
    unwrap!(eve.init().await);
    eve.set_spi_frequency(board::SPI_RUN_HZ);

    unwrap!(draw_frame(&mut eve, None).await);
    unwrap!(eve.apply_panel_timings());
    unwrap!(eve.enable_display());
    unwrap!(eve.set_backlight(96));
    info!("co-processor frame up — touch the panel");

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
