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
flush callback ──SPI──▶ FT813 RAM_G (1 MiB) framebuffer @ 0x000000
static EVE display list: fullscreen BITMAP scan-out @ 60 Hz panel refresh
FT813 touch engine ──SPI poll (REG_TOUCH_SCREEN_XY)──▶ LVGL pointer indev
```

The FT813 is used as a *dumb framebuffer*: no co-processor (`RAM_CMD`) usage,
just direct `RAM_G` writes. 800×480×2 = 768 000 B of the 1 MiB `RAM_G` hold the
framebuffer; the display list is written once and the panel rescans it
continuously, so a flushed rectangle appears on the next refresh.

SPI runs at 7.8 MHz during FT813 power-up (spec: ≤ 11 MHz) and 15.625 MHz
afterwards (250 MHz SPI2 kernel clock / 16; the next step, /8 = 31.25 MHz,
would exceed the FT81x's 30 MHz limit). A full-screen repaint therefore takes
~400 ms — fine for a widget UI where LVGL only flushes dirty rectangles.

Panel timings are the standard 800×480 WVGA set used for gen4-FT812/FT813-70
modules (HCYCLE 928, VCYCLE 525, PCLK 60/2 = 30 MHz, no external crystal →
`CLKINT`, with automatic `CLKEXT` fallback at init).

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
