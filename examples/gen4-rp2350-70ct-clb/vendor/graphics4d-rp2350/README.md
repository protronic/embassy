# Vendored Graphics4D (RP2350)

This directory should contain:

```text
lib/libgraphics4d_rp2350.a
include/Graphics4D.h
```

Generate on Arch/CachyOS:

```bash
cd examples/gen4-rp2350-70ct-clb
sudo pacman -S cmake ninja arm-none-eabi-gcc pico-sdk
export PICO_SDK_PATH=/usr/share/pico-sdk
export GEN4_GRAPHICS4D_SDK=./vendor/Graphics4D-pico
./scripts/vendor-graphics4d-into-repo.sh
git add vendor/graphics4d-rp2350
```

Until the `.a` is committed, Embassy falls back to the scanout stub (blank panel).
