# Vendored Graphics4D (RP2350)

Prebuilt scan-out library for the gen4-RP2350-70CT-CLB Embassy example.

## Status

**`lib/libgraphics4d_rp2350.a` is NOT in git yet** — only this README was committed so far.
The cloud agent cannot build it (`protronic/Graphics4D-pico` is private).

Without the `.a`, the example uses **Embassy PIO+DPI scan-out** (`src/dpi.rs`) automatically.

## Add the library (maintainers, local machine)

```bash
cd examples/gen4-rp2350-70ct-clb

# Arch / CachyOS
sudo pacman -S cmake ninja arm-none-eabi-gcc pico-sdk
export PICO_SDK_PATH=/usr/share/pico-sdk
export GEN4_GRAPHICS4D_SDK=./vendor/Graphics4D-pico   # your Workshop5 export

./scripts/vendor-graphics4d-into-repo.sh
./scripts/commit-vendored-graphics4d.sh
```

After push, `cargo build` links Graphics4D instead of the Embassy DPI fallback.

## Layout (after vendoring)

```
vendor/graphics4d-rp2350/
  lib/libgraphics4d_rp2350.a
  include/Graphics4D.h
  include/...              # other headers pulled in by Graphics4D.h
```
