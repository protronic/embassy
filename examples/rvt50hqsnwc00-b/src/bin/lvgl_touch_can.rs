#![no_std]
#![no_main]
#![allow(static_mut_refs)]

//! JSON-driven LVGL hall lighting UI with CAN press/hold/repeat for Riverdi RVT50.
//!
//! Layout and labels come from `touch-projects/SporthalleLudwigsfelde/hall_config.json`.
//! CAN TX/RX protocol comes from `can_config.json` (one-hot TX on 0x200, minp RX on 0x285).
//!
//! ```bash
//! cargo run --bin lvgl_touch_can --features lvgl,touch
//! ```

#[cfg(not(all(feature = "lvgl", feature = "touch")))]
compile_error!("lvgl_touch_can requires --features lvgl,touch");

use core::sync::atomic::{AtomicU8, Ordering};

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rvt50hqsnwc00_b_examples::can_bridge::{
    self, frame_standard_id, handle_minp_frame, send_command, send_release, set_active_button,
};
use embassy_rvt50hqsnwc00_b_examples::rvt50_board::{self, CAN_BITRATE};
use embassy_rvt50hqsnwc00_b_examples::touch_config::{self, BUTTON_COUNT, CAN_ENABLED, MINP_RX_ID};
use embassy_stm32::can::{CanRx, CanTx};
use embassy_stm32::i2c::I2c;
use embassy_stm32::ltdc::{self, Ltdc, LtdcLayer, LtdcLayerConfig};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use lvgl_sys as _;

const FB_PIXELS: usize =
    touch_config::DISPLAY_WIDTH as usize * touch_config::DISPLAY_HEIGHT as usize;
const MAX_BUTTONS: usize = 64;

static FB1: StaticCell<[u16; FB_PIXELS]> = StaticCell::new();
static CAN_TX: StaticCell<CanTx<'static>> = StaticCell::new();
static CAN_RX: StaticCell<CanRx<'static>> = StaticCell::new();

static BUTTON_STATUS: [AtomicU8; MAX_BUTTONS] = [const { AtomicU8::new(0) }; MAX_BUTTONS];

enum CanAction {
    Press(u8),
    Release,
}

static CAN_ACTIONS: Channel<CriticalSectionRawMutex, CanAction, 8> = Channel::new();

unsafe extern "C" {
    static g_hall_ui_layout: HallUiLayout;
    fn hall_ui_init(framebuffer: *mut u16, width: u16, height: u16, layout: *const HallUiLayout);
    fn hall_ui_set_touch(x: u16, y: u16, pressed: bool);
    fn hall_ui_tick(ms: u32);
    fn hall_ui_handler();
    fn hall_ui_set_button_active(button_index: u8, active: bool);
    fn hall_ui_set_callbacks(on_press: Option<extern "C" fn(u8)>, on_release: Option<extern "C" fn()>);
}

#[repr(C)]
struct HallUiLayout {
    _opaque: [u8; 0],
}

#[unsafe(no_mangle)]
extern "C" fn rvt50_hall_on_press(button_index: u8) {
    let _ = CAN_ACTIONS.try_send(CanAction::Press(button_index));
}

#[unsafe(no_mangle)]
extern "C" fn rvt50_hall_on_release() {
    let _ = CAN_ACTIONS.try_send(CanAction::Release);
}

#[embassy_executor::task]
async fn can_tx_task(tx: &'static mut CanTx<'static>) {
    if !CAN_ENABLED {
        info!("CAN disabled in config — UI only");
        loop {
            Timer::after_secs(60).await;
        }
    }

    can_bridge::log_button_order();
    info!(
        "CAN TX: id=0x{:03x}, repeat={}ms",
        touch_config::CAN_TX_ID,
        touch_config::CAN_COMMAND_REPEAT_MS,
    );

    let repeat = Duration::from_millis(touch_config::CAN_COMMAND_REPEAT_MS);

    loop {
        match CAN_ACTIONS.receive().await {
            CanAction::Press(index) => {
                set_active_button(Some(index));
                if !send_command(tx, index).await {
                    warn!("CAN send_command failed for button {}", index);
                }

                let mut held = index;
                loop {
                    match select(CAN_ACTIONS.receive(), Timer::after(repeat)).await {
                        Either::First(CanAction::Release) => {
                            set_active_button(None);
                            if !send_release(tx).await {
                                warn!("CAN send_release failed");
                            }
                            break;
                        }
                        Either::First(CanAction::Press(new_index)) => {
                            held = new_index;
                            set_active_button(Some(new_index));
                            if !send_command(tx, new_index).await {
                                warn!("CAN send_command failed for button {}", new_index);
                            }
                        }
                        Either::Second(()) => {
                            if !send_command(tx, held).await {
                                warn!("CAN repeat failed for button {}", held);
                            }
                        }
                    }
                }
            }
            CanAction::Release => {
                set_active_button(None);
                if !send_release(tx).await {
                    warn!("CAN send_release failed");
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn can_rx_task(rx: &'static mut CanRx<'static>) {
    if !CAN_ENABLED {
        loop {
            Timer::after_secs(60).await;
        }
    }

    info!("CAN RX: minp id=0x{:03x}", MINP_RX_ID);

    let mut scratch = [0u8; MAX_BUTTONS];

    loop {
        if let Ok(envelope) = rx.read().await {
            let (frame, _) = envelope.parts();
            if let Some(id) = frame_standard_id(&frame) {
                if id == MINP_RX_ID {
                    for (i, atom) in BUTTON_STATUS.iter().enumerate().take(BUTTON_COUNT) {
                        scratch[i] = atom.load(Ordering::Relaxed);
                    }
                    handle_minp_frame(id, frame.data(), &mut scratch[..BUTTON_COUNT]);
                    for (i, value) in scratch.iter().enumerate().take(BUTTON_COUNT) {
                        BUTTON_STATUS[i].store(*value, Ordering::Relaxed);
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn lvgl_touch_task(
    mut ltdc: Ltdc<'static, peripherals::LTDC, ltdc::Rgb565>,
    mut i2c: I2c<'static, Blocking, embassy_stm32::i2c::Master>,
) {
    info!(
        "LVGL hall UI {}x{} — {}",
        touch_config::DISPLAY_WIDTH,
        touch_config::DISPLAY_HEIGHT,
        touch_config::HALL_NAME,
    );

    let layer_config = LtdcLayerConfig {
        pixel_format: ltdc::PixelFormat::Rgb565,
        layer: LtdcLayer::Layer1,
        window_x0: 0,
        window_x1: touch_config::DISPLAY_WIDTH as _,
        window_y0: 0,
        window_y1: touch_config::DISPLAY_HEIGHT as _,
    };
    ltdc.init_layer(&layer_config, None);

    let fb = FB1.init([0; FB_PIXELS]);

    unsafe {
        hall_ui_set_callbacks(Some(rvt50_hall_on_press), Some(rvt50_hall_on_release));
        hall_ui_init(
            fb.as_mut_ptr(),
            touch_config::DISPLAY_WIDTH,
            touch_config::DISPLAY_HEIGHT,
            &raw const g_hall_ui_layout,
        );
    }

    ltdc.set_buffer(LtdcLayer::Layer1, fb.as_ptr() as *const _)
        .await
        .unwrap();

    loop {
        let touch = rvt50_board::read_touch(&mut i2c);
        unsafe {
            hall_ui_set_touch(touch.x, touch.y, touch.pressed);
            hall_ui_tick(5);
            hall_ui_handler();
        }

        for i in 0..BUTTON_COUNT {
            let active = BUTTON_STATUS[i].load(Ordering::Relaxed) != 0;
            unsafe {
                hall_ui_set_button_active(i as u8, active);
            }
        }

        ltdc.set_buffer(LtdcLayer::Layer1, fb.as_ptr() as *const _)
            .await
            .unwrap();

        Timer::after(Duration::from_millis(5)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    info!("Riverdi RVT50 - LVGL touch CAN");

    let p = rvt50_board::init_clocks();
    rvt50_board::enable_icache();

    let rvt50_board::TouchCanResources {
        ltdc,
        i2c,
        fdcan,
        can_rx_pin,
        can_tx_pin,
        can_stb,
    } = rvt50_board::init_touch_can(p).await;

    let mut can = rvt50_board::init_can(fdcan, can_rx_pin, can_tx_pin, can_stb);
    can.set_bitrate(CAN_BITRATE);
    let can = can.into_normal_mode();
    let (tx, rx, _) = can.split();

    let tx = CAN_TX.init(tx);
    let rx = CAN_RX.init(rx);

    spawner.spawn(unwrap!(lvgl_touch_task(ltdc, i2c)));
    spawner.spawn(unwrap!(can_tx_task(tx)));
    spawner.spawn(unwrap!(can_rx_task(rx)));

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
