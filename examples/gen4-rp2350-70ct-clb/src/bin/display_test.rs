//! Display bring-up test for the gen4-RP2350-70CT-CLB — no UI library.
//!
//! Draws the classic 8-bar color test pattern plus a moving white column,
//! and reports detailed status on the module's **USB-C serial port**
//! (CDC-ACM, e.g. `/dev/ttyACM0`) and RTT:
//!
//! - PSRAM detection and a write/read-back self-test
//! - scan-out DMA/PIO statistics every second (busy flag, read position,
//!   PIO program counter, TX FIFO level/stall)
//!
//! What to look for:
//!
//! - **Color bars + moving column** → scan-out and pin mapping are good.
//! - **Backlight on, screen black, stats advancing** → check the RGB
//!   data/DE/PCLK pin mapping in `src/gen4_board.rs::pins`.
//! - **Backlight off** → check `LCD_BACKLIGHT` (GPIO17).
//! - **`read_offset` frozen while `busy`** → PSRAM streaming issue.
//!
//! ```bash
//! cargo run --bin display_test
//! ```

#![no_std]
#![no_main]

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_rp2350_70ct_clb_examples::dpi::{self, Dpi, LcdPins};
use embassy_gen4_rp2350_70ct_clb_examples::gen4_board::{self, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

/// Scan-out cadence (~20 fps; the DPI stream needs ~42 ms per frame).
const FRAME_PERIOD_MS: u64 = 50;
/// Log a status line every N frames (≈ once per second).
const LOG_EVERY_FRAMES: u32 = 20;
/// PSRAM self-test window: 64 KiB at 4 MiB, far away from the framebuffer.
const SELF_TEST_OFFSET: usize = 4 * 1024 * 1024;
const SELF_TEST_LEN: usize = 64 * 1024;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = gen4_board::init();

    // USB CDC logger first, so everything below is visible on the USB-C port.
    spawner.spawn(unwrap!(gen4_board::usb_logger_task(gen4_board::usb_log_driver(p.USB))));

    info!("gen4-RP2350-70CT-CLB display test");

    let psram = gen4_board::init_psram(p.QMI_CS1, p.PIN_0).await;
    let psram_test = gen4_board::psram_self_test(&psram, SELF_TEST_OFFSET, SELF_TEST_LEN);
    match psram_test {
        Ok(()) => info!("PSRAM self-test OK ({} bytes)", SELF_TEST_LEN),
        Err(at) => defmt::error!("PSRAM self-test FAILED at byte {}", at),
    }

    let fb = psram.base_address();
    // SAFETY: PSRAM is mapped and at least FRAME_BYTES large.
    unsafe {
        dpi::fill_test_pattern(fb);
    }

    let _backlight = gen4_board::init_backlight(p.PIN_17);
    let _sync = gen4_board::park_sync_pins(p.PIN_36, p.PIN_37);

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let frame_dma = dma::Channel::new(p.DMA_CH0, Irqs);
    let lcd_pins = LcdPins {
        d0: p.PIN_18,
        d1: p.PIN_19,
        d2: p.PIN_20,
        d3: p.PIN_21,
        d4: p.PIN_22,
        d5: p.PIN_23,
        d6: p.PIN_24,
        d7: p.PIN_25,
        d8: p.PIN_26,
        d9: p.PIN_27,
        d10: p.PIN_28,
        d11: p.PIN_29,
        d12: p.PIN_30,
        d13: p.PIN_31,
        d14: p.PIN_32,
        d15: p.PIN_33,
        de: p.PIN_34,
        pclk: p.PIN_35,
    };
    let mut dpi = Dpi::new(&mut common, sm0, frame_dma, lcd_pins, fb as u32);
    info!(
        "scan-out started: {}x{} @ {} Hz pclk",
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
        dpi::PCLK_HZ
    );

    let mut frame: u32 = 0;
    let mut column: usize = 0;
    loop {
        // Move a white column across the test pattern so a live (not frozen)
        // scan-out is obvious on the panel.
        // SAFETY: single writer; the DMA only reads.
        unsafe {
            draw_column(fb, column, false);
            column = (column + 4) % DISPLAY_WIDTH;
            draw_column(fb, column, true);
        }

        dpi.present().await;
        frame = frame.wrapping_add(1);

        if frame % LOG_EVERY_FRAMES == 0 {
            let s = dpi.stats();
            let up = Instant::now().as_millis();
            info!(
                "frame={} uptime={}ms dpi: busy={} read_offset={} remaining={} sm_pc={} fifo={} stalled={}",
                frame, up, s.busy, s.read_offset, s.remaining_words, s.sm_pc, s.tx_fifo_level, s.tx_stalled
            );
            log::info!(
                "frame={} uptime={}ms psram={}B selftest={} dpi: busy={} read_offset={} remaining={} sm_pc={} fifo={} stalled={}",
                frame,
                up,
                psram.size(),
                if psram_test.is_ok() { "OK" } else { "FAILED" },
                s.busy,
                s.read_offset,
                s.remaining_words,
                s.sm_pc,
                s.tx_fifo_level,
                s.tx_stalled
            );
        }

        Timer::after(Duration::from_millis(FRAME_PERIOD_MS)).await;
    }
}

/// Draw or erase a 4-pixel-wide vertical column at `x`.
///
/// # Safety
/// `fb` must be the initialized framebuffer.
unsafe fn draw_column(fb: *mut u8, x: usize, white: bool) {
    for row in 0..DISPLAY_HEIGHT {
        for dx in 0..4 {
            let xx = (x + dx) % DISPLAY_WIDTH;
            let color = if white { 0xFFFFu16 } else { dpi::test_pattern_color(xx) };
            let px = color.to_le_bytes();
            // SAFETY: coordinates are clamped to the framebuffer dimensions.
            unsafe {
                let ptr = fb.add(row * dpi::ROW_BYTES + dpi::ROW_PIXEL_OFFSET + xx * 2);
                core::ptr::copy_nonoverlapping(px.as_ptr(), ptr, 2);
            }
        }
    }
}
