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

The on-module RGB interface is driven by 4D Systems' proprietary **Graphics4D** PIO driver from Workshop5. This example links it via the git submodule:

```bash
# once, from the embassy repo root:
git submodule update --init examples/gen4-rp2350-70ct-clb/vendor/Graphics4D-pico

cd examples/gen4-rp2350-70ct-clb
cargo run --bin oxivgl_widget_demo --features oxivgl,touch
```

The build script looks for `include/Graphics4D.h` and `lib/libgraphics4d_rp2350.a` inside `vendor/Graphics4D-pico`. Without it, LVGL still renders into PSRAM but `gen4_lcd_present_rgb565()` is a no-op stub (blank panel).

### Workshop5 / local SDK override

```bash
export GEN4_GRAPHICS4D_SDK=/path/to/workshop5/graphics4d-rp2350
cargo run --bin oxivgl_widget_demo --features oxivgl,touch
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

## Pin map (auxiliary signals)

| Function | GPIO | Notes |
|----------|------|-------|
| PSRAM CS1 | 0 | APS6404L (fixed on module) |
| Backlight PWM | 17 | TIM PWM |
| LCD reset | 37 | Active-low sequence in `gen4_board.rs` |
| Touch INT | 38 | FT5446 interrupt (active low) |
| Touch SCL | 39 | I2C1 |
| Touch SDA | 46 | I2C1 |
| Touch reset | 47 | Active-low |

RGB data / sync pins are wired internally on the module and are configured inside Graphics4D when linked.

## Troubleshooting

- **Blank screen without Graphics4D**: expected with the default stub — run `git submodule update --init vendor/Graphics4D-pico` or set `GEN4_GRAPHICS4D_SDK`.
- **PSRAM init fails**: verify you are targeting RP2350B (`rp235xb` feature) and APS6404L is populated.
- **Touch coordinates inverted**: adjust the axis flip in `gen4_board::read_touch()` for your mounting orientation.

## Resources

- [gen4-RP2350-RGB datasheet](https://resources.4dsystems.com.au/datasheets/rp2350/gen4-rp2350-rgb/)
- [gen4-RP2350-70CT-CLB downloads](https://resources.4dsystems.com.au/downloads/rp2350/gen4-RP2350-70CT-CLB/)
- [Embassy documentation](https://docs.embassy.dev/)

## License

MIT OR Apache-2.0 at your option.
