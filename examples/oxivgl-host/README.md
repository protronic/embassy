# OxivGL host demo (protronic widget scene)

SDL2 port of the Riverdi RVT50 **protronic** lighting-scene demo from
`examples/rvt50hqsnwc00-b`. Use it on a development PC to verify OxivGL widget
layout, event bubbling, and mouse click handling without hardware.

## Prerequisites

- **Nightly Rust** (see `rust-toolchain.toml`)
- **SDL2** development libraries, e.g. on Debian/Ubuntu:
  ```bash
  sudo apt-get install libsdl2-dev pkg-config
  ```

## Run

### Widget scene demo (no CAN)

```bash
cd examples/oxivgl-host
cargo run
```

### Hall lighting UI with SocketCAN

Same **5-column shell layout** as `oxivgl_widget_demo`, with UI strings from JSON.
Default touch project: [`DemoHost`](../touch-projects/DemoHost/) (`hall_name`: **Sporthalle Demo**,
SocketCAN on **`vcan0`**).

```bash
cd examples/oxivgl-host
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
cargo run --bin oxivgl_touch_can
```

Hold scene buttons in the SDL window to send the one-hot CAN command bitmask; release
to send the all-zero frame. Button highlight state follows `minp` feedback on CAN ID `0x285`.

To use a real CAN interface instead, override the project at build time:

```bash
TOUCH_PROJECT=Demo cargo run --bin oxivgl_touch_can
# then bring up can0 at 500 kbit/s before running
```

Latin-1 Montserrat fonts (ä ö ü ß, …) are compiled from
`examples/rvt50hqsnwc00-b/fonts/` via `LVGL_FONTS_DIR` in `.cargo/config.toml`.
Regenerate with `examples/rvt50hqsnwc00-b/fonts/generate.sh` if glyph coverage changes.

Click scene buttons in the SDL window. Terminal output shows:

- Widget layout bounds at startup (compare touch coordinates on the board)
- LVGL pointer events (`PRESSED`, `CLICKED`, …) with target handle and button index

## Relation to the board demo

| Board (`oxivgl_widget_demo`) | Host (this crate) |
|------------------------------|-------------------|
| LTDC RGB565 flush | SDL window |
| I2C touch via `TouchInput` (TIMER mode) | SDL mouse indev (TIMER mode) |
| `defmt` RTT logging | `log` / stderr |
| `touch_dbg` heartbeat task | Event lines printed directly |

Both targets use the same render cadence: `view.update()`, then four
`timer_handler()` passes per frame. On the board, `TouchInput::publish()` feeds
the I2C sample immediately before each tick — the same slot where SDL injects
mouse coordinates on the host.

If clicks work here but not on the RVT50, the widget tree and event wiring are
fine — focus on I2C sampling and coordinate mapping. If clicks fail here too,
inspect `WidgetView` flags (`CLICKABLE`, `EVENT_BUBBLE`) and event registration.

## CANopen JSON node with GUI (`json_node_gui`)

Puts the CANbossTouch PoC pieces together on the PC: **object dictionary from
JSON**, **Rhai scripts**, **SocketCAN** and an **OxivGL (LVGL) node screen** —
the host twin of the `json_node` examples for the STM32H573I-DK / STM32H5F5J-DK.

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
cd examples/oxivgl-host
cargo run --bin json_node_gui                        # json_node.json (inline OD)
cargo run --bin json_node_gui json_node_ds301.json   # OD imported from CANopenNode JSON export
```

Without `vcan0` the app runs UI-only (scripts still work); the config file can
also be passed via `JSON_NODE_CONFIG`.

### What the JSON describes

- **Object dictionary** — inline `datapoints` (index/sub/type/access/value/step)
  and/or `od_file`: a **CANopenNode / CANopenEditor JSON export**
  (`od/DS301_profile.json` here was exported from the CANopenNode repo's DS301
  example profile). `include` filters index ranges (CANbossTouch
  `network.json` semantics); `$NODEID+0x…` defaults resolve against `node_id`.
- **Rhai scripts** — `once` at boot, `cyclic` with a period; API:
  `od_read(i, s)`, `od_write(i, s, v)`, `get("name")`, `set("name", v)`,
  `od_dump()`, `node_id()`, `node_name()`, `uptime_ms()`, plus virtual process
  data `leds(mask)` / `led(n, on)` / `button()` rendered by the GUI.
- **CAN** — `{"mode": "socketcan", "channel": "vcan0", "tpdo": "0x180",
  "rpdo": "0x200", "heartbeat": true}`. Every OD change is sent as a "PDO"
  frame on `tpdo + node_id` (payload: index u16 LE, sub u8, dtype u8, value
  4 B LE — same framing as the STM32 examples). Frames on `rpdo + node_id`
  write into the OD; `access` is enforced as the bus view (`rw`/`wo` writable,
  `ro`/`const` rejected — scripts are device-internal and may write `ro`).
  With `heartbeat: true` the node sends a CANopen bootup and then a producer
  heartbeat on `0x700 + node_id`, period from **OD 0x1017.00** (ms, live —
  edit it in the GUI).

### GUI

One row per OD datapoint (name, `index.sub · type · access`, live value) with
`−` / `+` / toggle controls for `rw` entries; right panel with the virtual
LEDs (`0x6200`), the momentary **Taster** (`0x6000` via `button()`), the last
script `print` line and CAN TX/RX counters. All GUI writes go through the same
access checks as the bus.

Try on the shell: `candump vcan0` shows TPDOs and the heartbeat;
`cansend vcan0 210#0121000200000000` sets `blink_on` (0x2101.00) to false and
stops the light chaser — same frames as the H5 board demos.

### Note on LVGL source download

`oxivgl-sys` downloads LVGL v9.5.0 from GitHub at build time. In restricted
environments set `LVGL_SRC_DIR=/path/to/lvgl-9.5.0` to a local checkout of the
LVGL source tree instead.

### Tests

`cargo test` covers the node core without SDL: config parsing, script
scheduling, RPDO/UI access semantics, value clamping and the CANopenNode
OD import (including `$NODEID` resolution).
