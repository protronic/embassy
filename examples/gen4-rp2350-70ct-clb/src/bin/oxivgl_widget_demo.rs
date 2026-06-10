#![no_std]
#![no_main]
#![allow(static_mut_refs)]

//! OxivGL (C LVGL v9.5) multi-widget demo on the gen4-RP2350-70CT-CLB.
//!
//! Real LVGL compiled via [`oxivgl-sys`] with `conf/lv_conf.h`, rendering to
//! a PSRAM framebuffer that a PIO state machine scans out to the 800×480
//! parallel RGB panel (see `src/dpi.rs`).
//!
//! **Requires nightly Rust** (see `rust-toolchain.toml` in this crate) and
//! `arm-none-eabi-gcc` for the LVGL C sources.
//!
//! ```bash
//! cargo run --bin oxivgl_widget_demo --features oxivgl
//! cargo run --bin oxivgl_widget_demo --features oxivgl,touch
//! ```

extern crate alloc;

use core::mem::MaybeUninit;

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_rp2350_70ct_clb_examples::dpi::{self, Dpi, LcdPins};
use embassy_gen4_rp2350_70ct_clb_examples::gen4_board::{self, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use embassy_gen4_rp2350_70ct_clb_examples::oxivgl::platform::{self, LVGL_BUF_BYTES};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Duration, Timer};
use embedded_alloc::LlffHeap as Heap;
use oxivgl::display::LvglBuffers;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

/// Rust-side heap (LVGL itself uses its builtin allocator from `lv_conf.h`).
const HEAP_SIZE: usize = 64 * 1024;

/// Scan-out cadence. The DPI stream needs ~42 ms per frame at 10 MHz PCLK,
/// so 50 ms gives a steady ~20 fps with a little vertical blanking.
const FRAME_PERIOD_MS: u64 = 50;

#[global_allocator]
static HEAP: Heap = Heap::empty();

static mut HEAP_MEM: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();
static mut LVGL_BUFS: LvglBuffers<{ LVGL_BUF_BYTES }> = LvglBuffers::new();
static PSRAM: StaticCell<embassy_rp::psram::Psram<'static>> = StaticCell::new();

fn init_heap() {
    // SAFETY: called once before any allocation.
    unsafe {
        HEAP.init(HEAP_MEM.as_mut_ptr() as usize, HEAP_SIZE);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    init_heap();

    info!(
        "gen4-RP2350-70CT-CLB OxivGL widget demo ({}x{})",
        DISPLAY_WIDTH, DISPLAY_HEIGHT
    );

    let p = gen4_board::init();

    // Framebuffer lives at the start of the 8 MiB PSRAM.
    let psram = PSRAM.init(gen4_board::init_psram(p.QMI_CS1, p.PIN_0));
    assert!(psram.size() >= dpi::FRAME_BYTES);
    let fb = psram.base_address();
    // SAFETY: PSRAM is mapped and at least FRAME_BYTES large.
    unsafe {
        dpi::init_framebuffer(fb, rgb565(16, 32, 48));
    }

    // Panel control pins: backlight on, HSYNC/VSYNC parked (DE-only mode).
    let _backlight = gen4_board::init_backlight(p.PIN_17);
    let _sync = gen4_board::park_sync_pins(p.PIN_36, p.PIN_37);

    // PIO scan-out: data GPIO18..33, DE GPIO34, PCLK GPIO35.
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
    let dpi = Dpi::new(&mut common, sm0, frame_dma, lcd_pins, fb as u32);

    // SAFETY: static LVGL stripe buffers are only used from the UI task.
    let bufs = unsafe { &mut LVGL_BUFS };

    spawner.spawn(unwrap!(scanout_task(dpi)));
    spawner.spawn(unwrap!(heartbeat_info_task()));

    #[cfg(feature = "touch")]
    {
        let touch = gen4_board::init_touch(p.I2C1, p.PIN_46, p.PIN_39, p.PIN_47, p.PIN_38).await;
        spawner.spawn(unwrap!(ui_touch_task(fb, bufs, touch.i2c)));
    }

    #[cfg(not(feature = "touch"))]
    spawner.spawn(unwrap!(ui_task(fb, bufs)));

    loop {
        Timer::after_secs(60).await;
    }
}

fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3)
}

/// Stream the PSRAM framebuffer to the panel at a fixed cadence.
#[embassy_executor::task]
async fn scanout_task(mut dpi: Dpi<'static, PIO0, 0>) -> ! {
    loop {
        dpi.present().await;
        Timer::after(Duration::from_millis(FRAME_PERIOD_MS)).await;
    }
}

#[embassy_executor::task]
async fn heartbeat_info_task() -> ! {
    loop {
        info!("oxivgl widget demo heartbeat");
        Timer::after_secs(5).await;
    }
}

#[cfg(not(feature = "touch"))]
#[embassy_executor::task]
async fn ui_task(fb: *mut u8, bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>) -> ! {
    platform::run_widget_demo(fb, bufs).await
}

#[cfg(feature = "touch")]
#[embassy_executor::task]
async fn ui_touch_task(
    fb: *mut u8,
    bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>,
    i2c: embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C1, embassy_rp::i2c::Blocking>,
) -> ! {
    platform::run_widget_demo(fb, bufs, i2c).await
}
