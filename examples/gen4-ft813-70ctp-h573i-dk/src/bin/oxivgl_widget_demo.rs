#![no_std]
#![no_main]
#![allow(static_mut_refs)]

//! OxivGL (C LVGL v9.5) multi-widget demo on the gen4-FT813-70CTP-CLB.
//!
//! Real LVGL compiled via [`oxivgl-sys`] (`conf/lv_conf.h`), rendered into
//! SRAM stripe buffers and flushed over SPI into the FT813's 1 MiB `RAM_G`
//! framebuffer, which the EVE display list scans out to the 800×480 panel.
//! This is the FT81x/SPI counterpart to the `gen4-rp2350-70ct` and
//! `rvt50hqsnwc00-b` OxivGL demos.
//!
//! Touch comes from the FT813's built-in capacitive touch engine, polled over
//! the same SPI bus inside the UI task ([`platform::run_widget_demo`]).
//!
//! ```bash
//! cargo run --release --bin oxivgl_widget_demo --features oxivgl-demo
//! cargo run --release --bin oxivgl_widget_demo --features oxivgl-demo,eve
//! ```

extern crate alloc;

use core::mem::MaybeUninit;

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::board::{self, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use embassy_gen4_ft813_70ctp_h573i_dk_examples::firmware_id::FIRMWARE_ID;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::ft81x::Ft81x;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::oxivgl::platform;
#[cfg(not(feature = "eve"))]
use embassy_gen4_ft813_70ctp_h573i_dk_examples::oxivgl::platform::LVGL_BUF_BYTES;
use embassy_time::Timer;
use embedded_alloc::LlffHeap as Heap;
#[cfg(not(feature = "eve"))]
use oxivgl::display::LvglBuffers;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// LVGL has its own builtin pool (`LV_MEM_SIZE`); the Rust global allocator only
// backs the small widget Vecs in the view, so a modest heap is plenty.
const HEAP_SIZE: usize = 32 * 1024;

#[global_allocator]
static HEAP: Heap = Heap::empty();

static mut HEAP_MEM: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();
#[cfg(not(feature = "eve"))]
static mut LVGL_BUFS: LvglBuffers<{ LVGL_BUF_BYTES }> = LvglBuffers::new();
static EVE: StaticCell<Ft81x> = StaticCell::new();

fn init_heap() {
    // SAFETY: called once before any allocation.
    unsafe {
        HEAP.init(HEAP_MEM.as_mut_ptr() as usize, HEAP_SIZE);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    init_heap();

    let p = embassy_stm32::init(board::config());

    info!(
        "gen4-FT813-70CTP-CLB OxivGL demo ({}x{}) on STM32H573I-DK firmware={}",
        DISPLAY_WIDTH, DISPLAY_HEIGHT, FIRMWARE_ID
    );

    let mut eve = board::init_ft813(p.SPI2, p.PI1, p.PB15, p.PI2, p.PA3, p.PA8);

    #[cfg(not(feature = "eve"))]
    {
        unwrap!(eve.init().await);
        unwrap!(eve.co_clear_framebuffer(0x0000));
        unwrap!(eve.co_show_framebuffer().await);
        eve.set_spi_frequency(board::SPI_RUN_HZ);
        unwrap!(eve.apply_panel_timings());
        unwrap!(eve.enable_display());
        unwrap!(eve.set_backlight(96));
    }

    let eve = EVE.init(eve);
    #[cfg(not(feature = "eve"))]
    {
        let bufs = unsafe { &mut LVGL_BUFS };
        spawner.spawn(unwrap!(ui_task(eve, bufs)));
    }
    #[cfg(feature = "eve")]
    {
        spawner.spawn(unwrap!(ui_task(eve)));
    }

    loop {
        Timer::after_secs(60).await;
    }
}

#[cfg(not(feature = "eve"))]
#[embassy_executor::task]
async fn ui_task(eve: &'static mut Ft81x, bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>) -> ! {
    platform::run_widget_demo(eve, bufs).await
}

#[cfg(feature = "eve")]
#[embassy_executor::task]
async fn ui_task(eve: &'static mut Ft81x) -> ! {
    platform::run_widget_demo(eve).await
}
