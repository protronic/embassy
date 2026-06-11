# Vendored Graphics4D (RP2350)

Prebuilt scan-out library for the gen4-RP2350-70CT-CLB Embassy example.

## Status

Install a prebuilt archive here, or build from Graphics4D-pico sources.

**Prebuilt release** (Graphics4D-pico repo):

```bash
git fetch --tags origin
git checkout libgraphics4d-rp2350-20250611
export GEN4_GRAPHICS4D_SDK=$PWD
/path/to/embassy/examples/gen4-rp2350-70ct-clb/scripts/install-graphics4d-from-sdk.sh
```

Without the `.a`, the example uses **Embassy PIO+DPI scan-out** (`src/dpi.rs`) automatically.

## Self-hosted worker (recommended)

See `README.md` → **Self-hosted worker** and `.github/workflows/vendor-graphics4d.yml`.

## Add the library manually (local machine)

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
