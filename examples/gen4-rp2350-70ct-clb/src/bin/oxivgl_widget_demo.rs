#![no_std]
#![no_main]
#![allow(static_mut_refs)]

//! OxivGL (C LVGL v9.5) multi-widget demo on the 4D Systems gen4-RP2350-70CT-CLB.
//!
//! ```bash
//! cargo run --bin oxivgl_widget_demo --features oxivgl
//! cargo run --bin oxivgl_widget_demo --features oxivgl,touch
//! ```

extern crate alloc;

use core::mem::MaybeUninit;

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_rp2350_70ct_clb_examples::gen4_board::{self, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use embassy_gen4_rp2350_70ct_clb_examples::oxivgl::platform::{self, LVGL_BUF_BYTES};
use embassy_rp::config::Config;
use embassy_time::Timer;
use embedded_alloc::LlffHeap as Heap;
use oxivgl::display::LvglBuffers;
use {defmt_rtt as _, panic_probe as _};

const HEAP_SIZE: usize = 256 * 1024;

#[global_allocator]
static HEAP: Heap = Heap::empty();

static mut HEAP_MEM: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();
static mut LVGL_BUFS: LvglBuffers<{ LVGL_BUF_BYTES }> = LvglBuffers::new();

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

    let p = embassy_rp::init(Config::default());

    let Some(display) = gen4_board::init(p).await else {
        defmt::error!("Board init failed (PSRAM or panel setup)");
        loop {
            Timer::after_secs(5).await;
        }
    };

    let bufs = unsafe { &mut LVGL_BUFS };

    spawner.spawn(unwrap!(heartbeat_info_task()));
    spawner.spawn(unwrap!(ui_task(display, bufs)));

    loop {
        Timer::after_secs(60).await;
    }
}

#[embassy_executor::task]
async fn heartbeat_info_task() -> ! {
    loop {
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
