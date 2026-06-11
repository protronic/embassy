#![no_std]
#![no_main]
#![allow(static_mut_refs)]

//! OxivGL (C LVGL v9.5) multi-widget demo on the 4D Systems gen4-RP2350-70CT-CLB.
//!
//! Logs over **USB CDC serial** (`picocom /dev/ttyACM0`). Flash via BOOTSEL + picotool.
//!
//! Without Graphics4D, Embassy PIO+DPI scan-out drives the panel (GPIO22..37 data).
//!
//! ```bash
//! cargo run --bin oxivgl_widget_demo --features oxivgl
//! cargo run --bin oxivgl_widget_demo --features oxivgl,touch
//! ```

extern crate alloc;

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_rp2350_70ct_clb_examples::gen4_board::{self, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use embassy_gen4_rp2350_70ct_clb_examples::oxivgl::display::scanout_stats;
use embassy_gen4_rp2350_70ct_clb_examples::oxivgl::platform::{self, LVGL_BUF_BYTES};
use embassy_rp::config::Config;
use embassy_time::{Duration, Timer};
use embedded_alloc::LlffHeap as Heap;
use log::info as uinfo;
use oxivgl::display::LvglBuffers;
use {defmt_rtt as _, panic_probe as _};

#[cfg(not(gen4_graphics4d))]
use embassy_gen4_rp2350_70ct_clb_examples::dpi::{self, Dpi, LcdPins};
#[cfg(not(gen4_graphics4d))]
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
#[cfg(not(gen4_graphics4d))]
use embassy_rp::{bind_interrupts, dma};
#[cfg(not(gen4_graphics4d))]
use embassy_rp::peripherals::{DMA_CH0, PIO0};

#[cfg(not(gen4_graphics4d))]
bind_interrupts!(struct DpiIrqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

const HEAP_SIZE: usize = 256 * 1024;

#[cfg(not(gen4_graphics4d))]
const FRAME_PERIOD_MS: u64 = 50;

#[global_allocator]
static HEAP: Heap = Heap::empty();

static mut HEAP_MEM: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();
static mut LVGL_BUFS: LvglBuffers<{ LVGL_BUF_BYTES }> = LvglBuffers::new();

#[cfg(not(gen4_graphics4d))]
static FRAME_COUNT: AtomicU32 = AtomicU32::new(0);

fn init_heap() {
    // SAFETY: called once before any allocation.
    unsafe {
        HEAP.init(HEAP_MEM.as_mut_ptr() as usize, HEAP_SIZE);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    init_heap();

    let p = embassy_rp::init(Config::default());

    info!(
        "gen4-RP2350-70CT-CLB OxivGL widget demo ({}x{})",
        DISPLAY_WIDTH, DISPLAY_HEIGHT
    );

    let Some(mut display) = gen4_board::init(&spawner, p).await else {
        uinfo!("FATAL: board init failed (PSRAM or panel setup)");
        defmt::error!("Board init failed (PSRAM or panel setup)");
        loop {
            Timer::after_secs(5).await;
        }
    };

    uinfo!("board init OK — starting UI task");

    #[cfg(not(gen4_graphics4d))]
    {
        let fb = display.framebuffers.back;
        // SAFETY: PSRAM framebuffer is mapped and sized by the driver.
        unsafe {
            dpi::init_framebuffer(fb, 0x1020); // dark blue-grey while LVGL loads
        }
        uinfo!(
            "DPI scan-out: {}x{} @ {} Hz pclk, framebuffer {} KiB",
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT,
            dpi::PCLK_HZ,
            dpi::FRAME_BYTES / 1024
        );

        let dpi_pins = display.dpi.take().expect("DPI peripherals");
        let Pio { mut common, sm0, .. } = Pio::new(dpi_pins.pio0, DpiIrqs);
        let frame_dma = dma::Channel::new(dpi_pins.dma_ch0, DpiIrqs);
        let lcd_pins = LcdPins {
            de: dpi_pins.de,
            pclk: dpi_pins.pclk,
            d0: dpi_pins.d0,
            d1: dpi_pins.d1,
            d2: dpi_pins.d2,
            d3: dpi_pins.d3,
            d4: dpi_pins.d4,
            d5: dpi_pins.d5,
            d6: dpi_pins.d6,
            d7: dpi_pins.d7,
            d8: dpi_pins.d8,
            d9: dpi_pins.d9,
            d10: dpi_pins.d10,
            d11: dpi_pins.d11,
            d12: dpi_pins.d12,
            d13: dpi_pins.d13,
            d14: dpi_pins.d14,
            d15: dpi_pins.d15,
        };
        let dpi_drv = Dpi::new(&mut common, sm0, frame_dma, lcd_pins, fb as u32);
        spawner.spawn(unwrap!(dpi_scanout_task(dpi_drv)));
    }

    let bufs = unsafe { &mut LVGL_BUFS };

    spawner.spawn(unwrap!(heartbeat_info_task()));
    spawner.spawn(unwrap!(ui_task(display, bufs)));

    loop {
        Timer::after_secs(60).await;
    }
}

#[cfg(not(gen4_graphics4d))]
#[embassy_executor::task]
async fn dpi_scanout_task(mut dpi: Dpi<'static, PIO0, 0>) -> ! {
    loop {
        dpi.present().await;
        let frame = FRAME_COUNT.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
        if frame % 100 == 0 {
            let s = dpi.stats();
            info!(
                "dpi frame={} busy={} read_offset={} remaining={} sm_pc={} fifo={} stalled={}",
                frame, s.busy, s.read_offset, s.remaining_words, s.sm_pc, s.tx_fifo_level, s.tx_stalled
            );
            uinfo!(
                "dpi frame={} busy={} read_offset={} remaining={} sm_pc={} fifo={} stalled={}",
                frame,
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

#[embassy_executor::task]
async fn heartbeat_info_task() -> ! {
    loop {
        let (flushes, presents) = scanout_stats();
        #[cfg(not(gen4_graphics4d))]
        let frames = FRAME_COUNT.load(Ordering::Relaxed);
        uinfo!(
            "oxivgl heartbeat: lvgl_flushes={} panel_presents={}",
            flushes,
            presents
        );
        #[cfg(not(gen4_graphics4d))]
        uinfo!("oxivgl heartbeat: dpi_frames={}", frames);
        info!("oxivgl widget demo heartbeat");
        Timer::after_secs(5).await;
    }
}

#[embassy_executor::task]
async fn ui_task(
    resources: gen4_board::DisplayResources,
    bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>,
) -> ! {
    platform::run_widget_demo(resources, bufs).await
}
