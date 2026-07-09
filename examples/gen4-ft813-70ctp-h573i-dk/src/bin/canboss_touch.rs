#![no_std]
#![no_main]
#![allow(static_mut_refs)]

//! CANbossTouch (Rust-Port): CANopen-Bediengeraet auf dem STM32H573I-DK
//! mit gen4-FT813-70CTP-CLB.
//!
//! Vollport des C-Projekts `protronic/CANbossTouch`:
//!
//! - **EDS-Screens**: LVGL-Widget-Zeilen je Datenpunkt, zur Buildzeit aus
//!   `eds/network.json` + EDS-Dateien generiert (build.rs); Werte zyklisch
//!   per **SDO-Upload**, Parametrieren per **SDO-Download** (eigener
//!   CiA-301-SDO-Client, expedited + segmented)
//! - **PoC-Hallenlicht**: der JSON-getriebene Hallenscreen aus
//!   `touch-projects` als Menuepunkt (Bitmasken-Protokoll + Rhai-PLC via
//!   `touch-hall-common`)
//! - SDO- und PoC-Frames teilen sich **FDCAN2** (TX-Queue + RX-Router)
//!
//! Anders als das C-Original (CANopenNode) ist das Panel hier kein
//! vollstaendiger CANopen-Knoten: NMT/Heartbeat/SDO-Server entfallen,
//! implementiert ist die Bediengeraete-Seite (SDO-Client).
//!
//! ```bash
//! cargo run --release --bin canboss_touch --features oxivgl-demo
//! ```

extern crate alloc;

use core::mem::MaybeUninit;

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::board::{self, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use embassy_gen4_ft813_70ctp_h573i_dk_examples::canboss::{self, canbus, sdo};
use embassy_gen4_ft813_70ctp_h573i_dk_examples::firmware_id::FIRMWARE_ID;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::ft81x::Ft81x;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::oxivgl::platform::LVGL_BUF_BYTES;
use embassy_gen4_ft813_70ctp_h573i_dk_examples::touch_can;
use embassy_stm32::can::{CanRx, CanTx};
use embassy_time::Timer;
use embedded_alloc::LlffHeap as Heap;
use oxivgl::display::LvglBuffers;
use static_cell::StaticCell;
use touch_hall_common::{CAN_BAUD, CAN_ENABLED};
use {defmt_rtt as _, panic_probe as _};

// Navigator-Views (Box<dyn AnyView>), Zeilen-Vecs und der Rhai-PLC des
// PoC laufen auf dem Rust-Heap; LVGL nutzt seinen eigenen 128-KiB-Pool.
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
        "CANbossTouch Rust-Port ({}x{}) on STM32H573I-DK — {} Knoten, firmware={}",
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
        canboss::NODES.len(),
        FIRMWARE_ID
    );

    let mut eve = board::init_ft813(p.SPI2, p.PI1, p.PB15, p.PI2, p.PA3, p.PA8);
    unwrap!(eve.init().await);

    unwrap!(eve.clear_framebuffer(0x0000));
    unwrap!(eve.show_framebuffer());
    eve.set_spi_frequency(board::SPI_RUN_HZ);
    unwrap!(eve.set_backlight(96));

    // FDCAN2: gemeinsamer Bus fuer SDO-Client und PoC-Hallenlicht
    let mut can = board::init_can(p.FDCAN2, p.PB5, p.PB6);
    can.set_bitrate(CAN_BAUD);
    let (tx, rx, _) = can.into_normal_mode().split();
    spawner.spawn(unwrap!(canbus::tx_pump_task(CAN_TX.init(tx))));
    spawner.spawn(unwrap!(canbus::rx_router_task(CAN_RX.init(rx))));
    spawner.spawn(unwrap!(sdo::worker_task()));
    if CAN_ENABLED {
        spawner.spawn(unwrap!(touch_can::tx_task()));
    }
    info!(
        "FDCAN2 at {} bit/s: SDO-Client + PoC-Frames (PB5/PB6, Arduino D3/D15)",
        CAN_BAUD
    );

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
    canboss::platform::run_canboss_app(eve, bufs).await
}
