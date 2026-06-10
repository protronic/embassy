# 4D Systems gen4-RP2350-70CT-CLB Examples

Examples for the **4D Systems gen4-RP2350-70CT-CLB** intelligent display module,
featuring an **OxivGL** (C LVGL v9.5) widget demo.

## Board Information

- **Display**: 7.0" TN TFT LCD, capacitive touch, cover lens bezel (CLB)
- **Resolution**: 800x480 pixels, parallel RGB565 (DPI) interface driven by PIO
- **Microcontroller**: Raspberry Pi RP2350B (Cortex-M33 / Hazard3, 48 GPIO)
- **Flash**: 16MB Winbond W25Q128JVSIQ (QSPI, QMI CS0)
- **PSRAM**: 8MB AP Memory APS6404L-3SQR (QSPI, QMI CS1 on GPIO0)
- **Touch controller**: FocalTech FT5446 on I2C1
- **Interface**: 30-way FFC (UART, I2C, SPI, GPIO, SWD, 5V supply)

## Getting Started

### Prerequisites

1. Install [probe-rs](https://probe.rs/) for flashing and debugging:
   ```bash
   cargo install probe-rs-tools --locked
   ```

2. Install the ARM target:
   ```bash
   rustup target add thumbv8m.main-none-eabihf
   ```

3. For the OxivGL demo, install the ARM C toolchain (LVGL is compiled from
   source by `oxivgl-sys`):
   ```bash
   sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi
   ```

### Building and Running

```bash
cd examples/gen4-rp2350-70ct-clb
cargo run --bin hello_world                                  # RTT + PSRAM check
cargo run --bin oxivgl_widget_demo --features oxivgl         # widget demo
cargo run --bin oxivgl_widget_demo --features oxivgl,touch   # + FT5446 touch
```

Flashing options:

- **SWD** (default runner): connect a debug probe to `SWCLK`/`SWD` on FFC
  pins 18/19 and run with probe-rs.
- **USB bootloader**: hold BOOTSEL while plugging in USB-C, then use
  `picotool load -u -v -x -t elf <binary>` (uncomment the alternative runner
  in `.cargo/config.toml`).

## Available Examples

- `hello_world.rs` - RTT logging plus APS6404L PSRAM bring-up check
- `oxivgl_widget_demo.rs` - Multi-widget **OxivGL** demo (real C LVGL v9.5 via
  [oxivgl](https://github.com/emobotics-dev/oxivgl)): buttons with click
  counter, slider mirrored by a bar, switch, checkbox, and spinner

OxivGL builds need:

- `arm-none-eabi-gcc` and `libnewlib-arm-none-eabi`
- **Nightly Rust** (pinned by `rust-toolchain.toml` in this crate)
- Network access on first build (LVGL v9.5 sources are downloaded and
  compiled by `oxivgl-sys`)

The `oxivgl` / `oxivgl-sys` crates are vendored once in
`examples/rvt50hqsnwc00-b/vendor/` and shared by this crate via a
`[patch.crates-io]` path entry.

## Display Architecture

The RP2350B has no LCD controller, so the panel is driven entirely by PIO +
DMA (see `src/dpi.rs`):

- The **framebuffer lives in PSRAM** (800 x 480 x RGB565 ≈ 752KB does not fit
  in the 520KB SRAM). Each row is prefixed with one 32-bit word that the PIO
  program discards while DE is low, which keeps the state machine parked at a
  safe point between lines and frames.
- A single **PIO state machine** shifts 16-bit pixels out of GPIO18..33 and
  generates **DE** and **PCLK** on its side-set pins. The panel runs in
  **DE-only sync mode**; HSYNC/VSYNC are parked inactive.
- One **DMA channel** streams a whole frame (192,480 words) from PSRAM into
  the PIO TX FIFO per `present()` call; a small Embassy task re-kicks it at
  ~20 fps. The pause between kicks is the vertical blanking interval.
- The pixel clock is intentionally low (10 MHz) because the scan-out competes
  with code XIP and LVGL flush writes for QSPI bandwidth on the shared QMI.
- **LVGL** renders in `PARTIAL` mode into two 40-line SRAM stripe buffers;
  the flush callback copies dirty stripes into the PSRAM framebuffer
  (`src/oxivgl/display.rs`).

## Pin Configuration

User-facing pins (FFC, PSRAM, touch, SD) follow 4D Systems' published
gen4-RP2350 board files:

| Function | GPIO |
|----------|------|
| PSRAM CS (QMI CS1) | 0 |
| FFC user GPIO | 1..9, 16, 40..45 |
| UART1 TX/RX (FFC 24/23) | 4 / 5 |
| I2C0 SDA/SCL (FFC 4/3) | 8 / 9 |
| microSD (SDIO via PIO) | 10..15 |
| Touch INT / SCL / SDA / RST (I2C1) | 38 / 39 / 46 / 47 |

LCD interface pins used by this example (`src/gen4_board.rs::pins`):

| Signal | GPIO |
|--------|------|
| Backlight enable | 17 |
| RGB565 data B0..B4, G0..G5, R0..R4 | 18..22, 23..28, 29..33 |
| DE (PIO side-set 0) | 34 |
| PCLK (PIO side-set 1) | 35 |
| HSYNC (parked, DE mode) | 36 |
| VSYNC (parked, DE mode) | 37 |

> **Note**: 4D Systems does not publish the internal LCD pin mapping of the
> RGB series in the datasheet. The LCD block above follows the same family
> layout as their published gen4-RP2350 (MCU series) board files — backlight
> on GPIO17 and the LCD on the following pins. If the colors or sync signals
> are off on real hardware, verify the mapping against the Graphics4D board
> file from Workshop5 (or the module schematic) and adjust
> `src/gen4_board.rs::pins` plus the `LcdPins` wiring in the demo binary.

### Touch Controller (FT5446)

- **Bus**: I2C1, 400 kHz, 7-bit address `0x38`
- **Registers**: FT5x06 layout — `TD_STATUS` at `0x02`, P1 coordinates at
  `0x03..0x06`
- Up to 5 touch points; the demo uses the first point only

## Memory Usage

| Region | Use |
|--------|-----|
| PSRAM (8MB) | Framebuffer: 480 rows x (4B prefix + 800x2B) = ~752KB |
| SRAM (520KB) | 2 x 64KB LVGL stripe buffers, 64KB LVGL pool, 64KB Rust heap |
| Flash (16MB) | Code + LVGL (XIP) |

## License

All examples are licensed under either MIT or Apache-2.0 at your option.
