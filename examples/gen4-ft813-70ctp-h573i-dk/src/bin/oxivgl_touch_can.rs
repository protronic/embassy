#![no_std]
#![no_main]
#![allow(static_mut_refs)]

//! CANbossTouch: JSON-driven OxivGL hall lighting UI with CAN
//! press/hold/repeat on the gen4-FT813-70CTP-CLB + STM32H573I-DK.
//!
//! Configuration is generated at build time from
//! `examples/touch-projects/Demo/{hall,can}_config.json` (select another
//! project via the `TOUCH_PROJECT` env var).
//!
//! Button highlight state is driven by an optional Rhai script (`state_script`
//! in `can_config.json`). Edit `state.rhai` in the touch project to use logic
//! expressions over incoming CAN data (`can_bit`, `can_byte`, `minp_*`
//! helpers).
//!
//! CAN runs on FDCAN2 (Arduino D3 = PB5 = RX, D15 = PB6 = TX) — the DK needs
//! an external 3.3 V CAN transceiver, see [`board::init_can`].
//!
//! ```bash
//! cargo run --release --bin oxivgl_touch_can --features oxivgl-demo
//! ```

extern crate alloc;

use core::mem::MaybeUninit;

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::board::{self, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use embassy_gen4_ft813_70ctp_h573i_dk_examples::canboss::canbus;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::firmware_id::FIRMWARE_ID;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::ft81x::Ft81x;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::oxivgl::hall_platform;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::oxivgl::platform::LVGL_BUF_BYTES;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::touch_can;
use embassy_stm32::can::{CanRx, CanTx};
use embassy_time::Timer;
use embedded_alloc::LlffHeap as Heap;
use oxivgl::display::LvglBuffers;
use static_cell::StaticCell;
use touch_hall_common::{CAN_BAUD, CAN_ENABLED, HALL_NAME};
use {defmt_rtt as _, panic_probe as _};

// The Rhai PLC scan cycle and the hall view's config strings live on the Rust
// heap; LVGL widgets use the separate 128 KiB LVGL builtin pool. Together with
// the two 62 KiB stripe buffers this still fits the H573's 640 KiB SRAM.
const HEAP_SIZE: usize = 256 * 1024;

#[global_allocator]
static HEAP: Heap = Heap::empty();

static mut HEAP_MEM: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();
static mut LVGL_BUFS: LvglBuffers<{ LVGL_BUF_BYTES }> = LvglBuffers::new();
static EVE: StaticCell<Ft81x> = StaticCell::new();

static CAN_TX: StaticCell<CanTx<'static>> = StaticCell::new();
static CAN_RX: StaticCell<CanRx<'static>> = StaticCell::new();

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
        "gen4-FT813-70CTP-CLB CANbossTouch ({}x{}) on STM32H573I-DK — {} firmware={}",
        DISPLAY_WIDTH, DISPLAY_HEIGHT, HALL_NAME, FIRMWARE_ID
    );

    let mut eve = board::init_ft813(p.SPI2, p.PI1, p.PB15, p.PI2, p.PA3, p.PA8);
    unwrap!(eve.init().await);

    // Black framebuffer + static scan-out display list, then full SPI speed
    // and backlight on: the panel shows black until LVGL paints the first
    // frame a few ticks later.
    unwrap!(eve.clear_framebuffer(0x0000));
    unwrap!(eve.show_framebuffer());
    eve.set_spi_frequency(board::SPI_RUN_HZ);
    unwrap!(eve.apply_panel_timings());
    unwrap!(eve.enable_display());
    unwrap!(eve.set_backlight(96));

    if CAN_ENABLED {
        let mut can = board::init_can(p.FDCAN2, p.PB5, p.PB6);
        can.set_bitrate(CAN_BAUD);
        let (tx, rx, _) = can.into_normal_mode().split();
        spawner.spawn(unwrap!(canbus::tx_pump_task(CAN_TX.init(tx))));
        spawner.spawn(unwrap!(canbus::rx_router_task(CAN_RX.init(rx))));
        spawner.spawn(unwrap!(touch_can::tx_task()));
        info!("FDCAN2 enabled at {} bit/s (PB5/PB6, Arduino D3/D15)", CAN_BAUD);
    } else {
        info!("CAN disabled in config — UI only");
    }

    let eve = EVE.init(eve);
    // SAFETY: static LVGL stripe buffers are only used from the UI task.
    let bufs = unsafe { &mut LVGL_BUFS };
    spawner.spawn(unwrap!(ui_task(eve, bufs)));

    loop {
        Timer::after_secs(60).await;
    }
}

#[embassy_executor::task]
async fn ui_task(eve: &'static mut Ft81x, bufs: &'static mut LvglBuffers<{ LVGL_BUF_BYTES }>) -> ! {
    hall_platform::run_hall_demo(eve, bufs).await
}
