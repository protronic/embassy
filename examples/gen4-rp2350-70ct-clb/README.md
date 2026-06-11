# 4D Systems gen4-RP2350-70CT-CLB Examples

Examples for the **4D Systems gen4-RP2350-70CT-CLB** intelligent display module (7.0" capacitive touch, cover lens bezel).

## Board Information

- **Display**: 7.0" TN TFT, 800×480 RGB565 (landscape)
- **MCU**: Raspberry Pi RP2350B
- **Flash**: 16 MiB QSPI (W25Q128)
- **PSRAM**: 8 MiB APS6404L (QMI CS1 / GPIO0)
- **Touch**: FocalTech FT5446 (I2C1, capacitive)
- **Programming**: USB-C (BOOTSEL + RESET buttons)

## Prerequisites

1. [probe-rs](https://probe.rs/) for flashing:

   ```bash
   cargo install probe-rs --locked
   rustup target add thumbv8m.main-none-eabihf
   ```

2. **Nightly Rust** (see `rust-toolchain.toml` in this crate).

3. For OxivGL builds:

   - `arm-none-eabi-gcc` and `libnewlib-arm-none-eabi`
   - **Graphics4D** from [protronic/Graphics4D-pico](https://github.com/protronic/Graphics4D-pico) for live RGB panel scanout (see below)

## OxivGL widget demo

Real LVGL v9.5 via [OxivGL](https://github.com/emobotics-dev/oxivgl), with the same multi-widget lighting scene used on the Riverdi RVT50 port.

```bash
cd examples/gen4-rp2350-70ct-clb
cargo run --bin oxivgl_widget_demo --features oxivgl
cargo run --bin oxivgl_widget_demo --features oxivgl,touch
```

## RGB panel scanout (Graphics4D-pico)

The on-module RGB interface is driven by 4D Systems' proprietary **Graphics4D** PIO driver from Workshop5. Clone the companion library into `vendor/Graphics4D-pico`:

```bash
cd examples/gen4-rp2350-70ct-clb
./scripts/init-graphics4d-pico.sh          # or use your existing vendor/Graphics4D-pico clone
export PICO_SDK_PATH=~/pico/pico-sdk       # Pico SDK 2.x
./scripts/build-graphics4d-lib.sh          # if the repo has sources but no .a yet
./scripts/check-graphics4d.sh
cargo run --bin oxivgl_widget_demo --features oxivgl,touch
```

The init script clones [protronic/Graphics4D-pico](https://github.com/protronic/Graphics4D-pico). Many Workshop5 exports ship **`src/Graphics4D.h` + C++ sources** without a prebuilt Linux archive — `build-graphics4d-lib.sh` compiles `lib/libgraphics4d_rp2350.a` using the Pico SDK (or runs `embassy/build.sh` / CMake when present).

The Embassy build needs **`Graphics4D.h`** and **`libgraphics4d_rp2350.a`** under the SDK root. Without them, LVGL still renders into PSRAM but `gen4_lcd_present_rgb565()` is a no-op stub — USB log shows:

```text
panel: Graphics4D not found — ...
panel_presents=N   # counter rises, but panel stays blank
```

After linking Graphics4D, rebuild, reflash, and confirm:

```text
panel: Graphics4D linked — RGB scanout active
```

> **Note:** `vendor/Graphics4D-pico/` is gitignored. If the GitHub repo does not exist yet, export the Workshop5 Graphics4D tree from Windows and set `GEN4_GRAPHICS4D_SDK` on Linux (see below).

### Workshop5 — `libgraphics4d_rp2350.a` not found?

Workshop5 often ships **only headers** in Graphics4D-pico (`src/Graphics4D.h`), not a ready-made `.a`. That is normal.

```bash
./scripts/inspect-graphics4d-pico.sh    # what is in your tree?
```

**A) `src/*.cpp` present** — build on Linux (needs `cmake`, `ninja`, Pico SDK 2.x):

```bash
export PICO_SDK_PATH=/usr/share/pico-sdk
export GEN4_GRAPHICS4D_SDK=./vendor/Graphics4D-pico
./scripts/build-graphics4d-lib.sh
```

The script uses an **Embassy CMake wrapper** by default. The upstream `Graphics4D-pico/CMakeLists.txt` expects a Workshop5 parent project (`../pico_sdk_import.cmake`) and is only tried with `GRAPHICS4D_NATIVE_CMAKE=1`.

**B) only `.h`, no sources** — on Windows, compile any Workshop5 gen4-RP2350 project, then:

```powershell
powershell -File scripts\find-graphics4d-lib.ps1
```

Copy the found archive to `vendor/Graphics4D-pico/lib/libgraphics4d_rp2350.a`, then on Linux:

```bash
./scripts/check-graphics4d.sh
cargo build --bin oxivgl_widget_demo --features oxivgl,touch
# reflash — old firmware stays stub until you flash again
```

If Graphics4D was built against the Pico SDK, also set `PICO_SDK_PATH`.

### Optional Embassy glue inside Graphics4D-pico

The submodule may ship Embassy-specific files:

| Path | Purpose |
|------|---------|
| `embassy/gen4_lcd_glue.cpp` | Replaces the default glue in `c/display_gfx4d.cpp` |
| `embassy/link.txt` | Extra linker arguments (Pico SDK libs, etc.) |

## Memory layout

| Resource | Size | Location |
|----------|------|----------|
| LVGL stripe buffers | 2 × 32 KiB | On-chip SRAM |
| Full-screen framebuffers | 2 × 750 KiB | PSRAM base |
| OxivGL / LVGL heap | 256 KiB | On-chip SRAM |

## Pin map

From Pico SDK `board/gen4_rp2350_70ct.h` (CLB uses the same map):

| Function | GPIO | Notes |
|----------|------|-------|
| PSRAM CS1 | 0 | APS6404L (QMI CS1) |
| Backlight PWM | 17 | `LCD_BACKLIGHT` |
| LCD DE | 18 | RGB data-enable |
| LCD VSYNC | 19 | |
| LCD HSYNC | 20 | |
| LCD PCLK | 21 | 25 MHz (`LCD_CLK_FREQ`) |
| LCD DATA0…15 | 22–37 | 16-bit RGB565 (B0 LSB on GPIO22) |
| Touch INT | 38 | FT5446, active low |
| Touch SCL | 39 | I2C1 |
| Touch SDA | 46 | I2C1 |
| Touch RST | 47 | Active-low reset in `gen4_board.rs` |

Panel reset and PIO RGB timing are handled by **Graphics4D** (`gfx.Initialize()`), not by Embassy.
Touch coordinates use `LCD_TOUCH_SWAP_XY` from the board header.

## Troubleshooting

- **Blank screen without Graphics4D**: expected with the default stub — run `git submodule update --init vendor/Graphics4D-pico` or set `GEN4_GRAPHICS4D_SDK`.
- **PSRAM init fails**: verify you are targeting RP2350B (`rp235xb` feature) and APS6404L is populated.
- **Touch coordinates wrong**: the board file sets `LCD_TOUCH_SWAP_XY`; adjust `gen4_board::read_touch()` if your mounting differs.

## Resources

- [gen4-RP2350-RGB datasheet](https://resources.4dsystems.com.au/datasheets/rp2350/gen4-rp2350-rgb/)
- [gen4-RP2350-70CT-CLB downloads](https://resources.4dsystems.com.au/downloads/rp2350/gen4-RP2350-70CT-CLB/)
- [Embassy documentation](https://docs.embassy.dev/)

## License

MIT OR Apache-2.0 at your option.
