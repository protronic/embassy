# gen4-FT813-70CTP-CLB on STM32H573I-DK (OxivGL over SPI)

Embassy + **OxivGL** (real C LVGL v9.5 via [oxivgl](https://github.com/emobotics-dev/oxivgl))
for a 4D Systems **gen4-FT813-70CTP-CLB** 7.0″ 800×480 capacitive-touch display
driven by an **STM32H573I-DK** Discovery kit over **SPI**.

This is the FT81x/EVE2 counterpart to the `gen4-rp2350-70ct` (PIO RGB) and
`rvt50hqsnwc00-b` (LTDC) OxivGL ports: same widget demo, same UI loop, but the
panel hangs off a 4-wire SPI bus instead of a parallel RGB interface.

## Hardware

| Feature   | Detail |
|-----------|--------|
| MCU       | STM32H573IIK3Q (Cortex-M33 @ 250 MHz, 2 MB flash, 640 KB SRAM) |
| Display   | gen4-FT813-70CTP-CLB — FT813 (EVE2), 800×480 RGB565, capacitive touch, cover-lens bezel |
| Interface | SPI2 (mode 0), manual CS + PD (power-down) GPIOs |
| Touch     | FT813 built-in capacitive touch engine, polled over the same SPI bus |
| Backlight | Driven by the FT813 (`REG_PWM_DUTY`) — no extra MCU pin |

### Wiring (Arduino R3 header, matches the Zephyr `stm32h573i_dk` pin map)

| gen4 FFC / gen4-PA | Arduino | MCU pin | Function |
|--------------------|---------|---------|----------|
| SCK                | D13     | PI1     | SPI2_SCK |
| MISO (SDO)         | D12     | PI2     | SPI2_MISO |
| MOSI (SDI)         | D11     | PB15    | SPI2_MOSI |
| /CS                | D10     | PA3     | GPIO out |
| PD                 | D9      | PA8     | GPIO out |
| INT                | D8      | PG8     | unused (touch is polled) |
| 5V / GND           | 5V/GND  | —       | module supply (backlight boost needs 5 V) |

Use a **gen4-PA** or **gen4-IB** breakout for the 30-way FFC. Logic is 3.3 V.

### CAN wiring (`oxivgl_touch_can` only)

FDCAN2 is on the same Arduino header; the DK has **no on-board CAN
transceiver**, so wire an external 3.3 V transceiver (SN65HVD230,
TJA1051T/3, …) between the MCU pins and the bus:

| Signal | Arduino | MCU pin | Function |
|--------|---------|---------|----------|
| CAN RX | D3      | PB5     | FDCAN2_RX (← transceiver RXD) |
| CAN TX | D15     | PB6     | FDCAN2_TX (→ transceiver TXD) |

## Architecture

```text
LVGL v9.5 (PARTIAL render mode, RGB565)
    └─ renders dirty areas into 2 × 39-line SRAM stripe buffers
flush callback ──host SPI burst──▶ FT813 RAM_G (1 MiB) @ 0x000000
EVE display list (co-processor): fullscreen BITMAP scan-out @ 60 Hz
FT813 touch engine ──SPI poll (REG_TOUCH_SCREEN_XY)──▶ LVGL pointer indev
```

LVGL pixels reach `RAM_G` through fast host SPI writes ([`Ft81x::blit`]); the
bitmap display list is built once by the EVE co-processor with
[`Ft81x::co_show_framebuffer`]. 800×480×2 = 768 000 B of the 1 MiB `RAM_G`
hold the framebuffer; the panel rescans it continuously, so a flushed rectangle
appears on the next refresh.

### EVE GPU renderer (`eve` feature)

Optional: LVGL's [EVE External GPU Renderer](https://lvgl.io/docs/open/integration/external_display_controllers/eve/gpu)
(`LV_USE_DRAW_EVE`). LVGL sends draw commands to the FT813 co-processor instead
of software-rendering pixels into `RAM_G`. No SRAM stripe buffers are needed.

Side-by-side comparison: [RENDER_MODES.md](RENDER_MODES.md).

```bash
cargo run --release --bin canboss_touch --features oxivgl-demo,eve
```

SPI runs at 7.8 MHz during FT813 power-up (spec: ≤ 11 MHz) and 15.625 MHz
afterwards (250 MHz SPI2 kernel clock / 16; the next step, /8 = 31.25 MHz,
would exceed the FT81x's 30 MHz limit). A full-screen repaint therefore takes
~400 ms — fine for a widget UI where LVGL only flushes dirty rectangles.

No external crystal → `CLKINT` (with automatic `CLKEXT` fallback at init).

### Panel timings (7.0″ TN)

From the module datasheet **`gen4_FT81x_Series_Datasheet_R_1_5.pdf`**, section
*"7.0″ LCD Timing Characteristic (TN)"* — the gen4-FT813-70CT-CLB is the 7.0″
TN panel. These are **not** the generic FTDI WQVGA numbers, and they differ
from the 5.0″ and the IPS variants in the same datasheet.

| Item | Symbol | Min | Typ | Max | Unit |
|------|--------|----:|----:|----:|------|
| DCLK Frequency       | Fclk   | 20  | 33.3 | 50   | MHz  |
| Hsync Period Time    | Th     | 908 | 928  | 1088 | DCLK |
| Hsync Display Period | Thdisp | –   | 800  | –    | DCLK |
| Hsync Back Porch     | Thbp   | 1   | 40   | 87   | DCLK |
| Hsync Front Porch    | Thfp   | 20  | 40   | 200  | DCLK |
| Hsync Pulse Width    | Thw    | 1   | 4    | 43   | DCLK |
| Vsync Period Time    | Tv     | 517 | 525  | 712  | H    |
| Vsync Display Period | Tvdisp | –   | 480  | –    | H    |
| Vsync Back Porch     | Tvbp   | 29  | 31   | 31   | H    |
| Vsync Front Porch    | Tvfp   | 5   | 13   | 200  | H    |
| Vsync Pulse Width    | Tvw    | 1   | 1    | 3    | H    |

Mapped to the FT81x registers in `src/ft81x.rs` (TYP values):

| Register | Value | Derivation |
|----------|------:|------------|
| `REG_HCYCLE`  | 928 | Th |
| `REG_HSIZE`   | 800 | Thdisp |
| `REG_HOFFSET` | 44  | Thw + Thbp (4 + 40) |
| `REG_HSYNC0`  | 0   | — |
| `REG_HSYNC1`  | 4   | Thw |
| `REG_VCYCLE`  | 525 | Tv |
| `REG_VSIZE`   | 480 | Tvdisp |
| `REG_VOFFSET` | 32  | Tvw + Tvbp (1 + 31) |
| `REG_VSYNC0`  | 0   | — |
| `REG_VSYNC1`  | 1   | Tvw |
| `REG_PCLK`    | 2   | 60 MHz ÷ 2 = 30 MHz DCLK (within Fclk 20–50) |

> **Note:** with these datasheet-correct timings the colour bars can still look
> streaky toward the bottom on a loose flywire hookup. That is a signal/ground
> integrity issue (weak module GND), not a timing bug — the same weak ground
> that makes the capacitive touch drop out while the panel is scanning. Solid,
> short GND + power to the module is the fix. See the touch note below.

### Touch & known limitations

Touch is the FT813's built-in capacitive engine, read by polling
`REG_TOUCH_SCREEN_XY` over SPI. This panel reports the axes **transposed** vs
the FT81x default (X in the low 16 bits, Y in the high 16), which `Ft81x::touch`
compensates for. Coordinates are raw/uncalibrated — run `CMD_CALIBRATE` for an
exact finger-to-pixel mapping.

Panel timings must be applied **after** the first co-processor frame while
`REG_PCLK` is still off ([`Ft81x::apply_panel_timings`]), then
[`Ft81x::enable_display`] asserts DISP and starts the pixel clock. Do not
re-write timings inside `enable_display` — that was a regression in the
timing-fix commit.

While the panel is scanning, touch can also drop out on a marginal power hookup
(weak module GND / insufficient current when the backlight ramps). Solid, short
GND wires and adequate supply to the module are required.

## Build

Requirements: nightly toolchain (pinned in `rust-toolchain.toml`),
`arm-none-eabi-gcc` + newlib headers (the LVGL C sources are cross-compiled by
`oxivgl-sys`), and [probe-rs](https://probe.rs/).

```bash
cd examples/gen4-ft813-70ctp-h573i-dk

# board support + FT813 driver smoke test (no LVGL, no C toolchain needed)
cargo run --release --bin ft813_selftest

# OxivGL (C LVGL) widget demo
cargo run --release --bin oxivgl_widget_demo --features oxivgl-demo

# JSON-driven hall lighting UI + CAN press/hold/repeat (FDCAN2)
cargo run --release --bin oxivgl_touch_can --features oxivgl-demo

# CANbossTouch Rust-Port: CANopen-Bediengeraet (EDS-Screens + SDO-Client)
cargo run --release --bin canboss_touch --features oxivgl-demo

# Same demos with LVGL EVE GPU renderer (co-processor draws widgets)
cargo run --release --bin canboss_touch --features oxivgl-demo,eve
```

`ft813_selftest` paints eight colour bars and logs touch coordinates — run it
first to verify wiring and FT813 bring-up without the C stack.

### CANbossTouch Rust-Port (`canboss_touch`)

Vollport des C-Projekts [protronic/CANbossTouch](https://github.com/protronic/CANbossTouch)
auf Rust/embassy/oxivgl — dieselbe Bediengeraete-Funktionalitaet auf derselben
Hardware:

- **EDS-Screens**: `build.rs` parst `eds/network.json` + EDS-Dateien (CiA 306)
  und generiert je Knoten eine Datenpunkt-Tabelle (Rust-Pendant zu
  `tools/eds2lvgl.py`). Der Navigator zeigt ein Hauptmenue (Knotenliste) und
  je Knoten einen Screen mit einer Widget-Zeile pro Datenpunkt — verfeinerte
  Zuordnung wie im C-Port: Wertanzeige, Balken (ro + Limits), Switch (BOOL /
  0..1), Slider (Limit-Spanne ≤ 2000, Schreiben beim Loslassen), Spinbox
  (Integer bzw. REAL32 als Festkomma ×1000), Textfeld mit Tastatur.
- **SDO-Client** (`src/canboss/sdo.rs`): eigener CiA-301-Client (expedited +
  segmented Up-/Download, 0x600/0x580+NodeID, 500 ms Timeout); die UI pollt
  Slot-Zustaende wie im C-Original (zyklischer Refresh 1 s/Zeile, max. 4 neue
  Reads pro Scan, Edit-Hold 2 s).
- **PoC-Hallenlicht** als Menuepunkt: der `hall_view`-Screen inklusive
  Rhai-PLC (`touch-hall-common`) — SDO- und PoC-Frames teilen sich FDCAN2
  ueber eine TX-Queue + RX-Router (`src/canboss/canbus.rs`).

Anders als das C-Original (CANopenNode-Stack) ist das Panel hier kein
vollstaendiger CANopen-Knoten (kein NMT/Heartbeat/SDO-Server) — implementiert
ist die Bediengeraete-Seite.

### CANbossTouch PoC (`oxivgl_touch_can`)

The FT81x/SPI port of the `rvt50hqsnwc00-b` / `rp2350-touch-lcd-7` hall
lighting CAN demo. UI strings, button/field layout, CAN IDs, bitrate, and the
optional Rhai state script are generated at build time from
`examples/touch-projects/Demo/{hall,can}_config.json` (pick another project
directory with the `TOUCH_PROJECT` env var — see
`examples/touch-projects/README`-files). Button presses/holds/releases are
translated into the CAN bitmask protocol from `examples/touch-hall-common`
on FDCAN2, and incoming `minp` frames drive the button highlight state
(optionally through the `state.rhai` PLC scan cycle).

### OxivGL notes

- `conf/lv_conf.h` — LVGL configuration (128 KiB builtin pool, RGB565).
- `fonts/` — Montserrat 14/16 with Latin-1 coverage (German umlauts);
  regenerate with `fonts/generate.sh`.
- `vendor/` — vendored `oxivgl` 0.5.0 / `oxivgl-sys` 0.2.2 (patched for
  bare-metal thumb cross-compilation), shared with `examples/gen4-rp2350-70ct`.
- `oxivgl-sys` downloads the LVGL 9.5.0 sources at build time (SHA256-pinned);
  set `LVGL_SRC_DIR` to a local checkout for offline builds.
