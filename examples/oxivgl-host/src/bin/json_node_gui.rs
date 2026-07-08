//! JSON-driven CANopen node with OxivGL GUI on the host (SDL2 + SocketCAN).
//!
//! Puts the CANbossTouch PoC pieces together on the PC:
//! - object dictionary from JSON (inline datapoints and/or a CANopenNode /
//!   CANopenEditor JSON export via `od_file`)
//! - Rhai scripts (`once` / `cyclic`) working on the OD
//! - SocketCAN "PDO" TX/RX + CANopen bootup/heartbeat
//! - OxivGL (LVGL) node screen: live OD rows with edit controls, virtual
//!   LEDs, momentary push button, script/CAN status
//!
//! ```bash
//! sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
//! cd examples/oxivgl-host
//! cargo run --bin json_node_gui                        # json_node.json (inline OD)
//! cargo run --bin json_node_gui json_node_ds301.json   # imported CANopenNode OD
//! candump vcan0                                        # watch TPDO/heartbeat
//! ```

use core::ffi::CStr;
use std::path::{Path, PathBuf};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use log::info;
use oxivgl::driver::LvglDriver;
use oxivgl::view::{View, register_view_events};
use oxivgl::widgets::{Obj, Screen};
use oxivgl_sys::LV_DEF_REFR_PERIOD;
use static_cell::StaticCell;

use embassy_oxivgl_host_examples::od_node;
use embassy_oxivgl_host_examples::od_view::OdView;

const DISPLAY_WIDTH: i32 = 800;
const DISPLAY_HEIGHT: i32 = 480;
const LVGL_TICK_MS: u64 = LV_DEF_REFR_PERIOD as u64 / 4;
const TICKS_PER_FRAME: usize = 4;

static VIEW: StaticCell<OdView> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    env_logger::builder()
        .filter_level(log::LevelFilter::Debug)
        .format_timestamp_millis()
        .init();

    // Config: CLI argument > JSON_NODE_CONFIG env > json_node.json in the crate.
    let config_path: PathBuf = std::env::args()
        .nth(1)
        .or_else(|| std::env::var("JSON_NODE_CONFIG").ok())
        .map(PathBuf::from)
        .map(|p| {
            if p.is_relative() && !p.exists() {
                Path::new(env!("CARGO_MANIFEST_DIR")).join(p)
            } else {
                p
            }
        })
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("json_node.json"));

    info!("json_node_gui: loading {}", config_path.display());
    let cfg = od_node::load_config(&config_path).unwrap_or_else(|e| panic!("config: {e}"));
    let config_dir = config_path.parent().unwrap_or(Path::new(".")).to_path_buf();
    od_node::init_node(&cfg, &config_dir).unwrap_or_else(|e| panic!("object dictionary: {e}"));
    od_node::start_can(&cfg);

    // Rhai: runs the `once` scripts, compiles the `cyclic` ones.
    let mut runtime = od_node::NodeRuntime::new(&cfg);

    let title = CStr::from_bytes_with_nul(b"CANopen JSON Node\0").unwrap();
    let driver = LvglDriver::sdl(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        .title(title)
        .mouse(true)
        .build();

    let view = VIEW.init(OdView::default());
    let screen = Screen::active().expect("LVGL screen must exist after SDL init");
    let container = Obj::from_raw_non_owning(screen.handle());
    view.create(&container).expect("od view create failed");
    register_view_events(view);
    view.log_layout();

    info!("SDL window active — edit rw entries with -/+/wechseln, hold the Taster for 0x6000");

    // Single-threaded main loop: scripts (PLC-style) + LVGL, like the
    // embedded json_node scheduler; CAN runs on background threads.
    loop {
        runtime.poll();
        let _ = view.update();
        for _ in 0..TICKS_PER_FRAME {
            driver.timer_handler();
            Timer::after(Duration::from_millis(LVGL_TICK_MS)).await;
        }
    }
}
