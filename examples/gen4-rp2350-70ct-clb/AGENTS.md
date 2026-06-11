# AGENTS.md — Build `libgraphics4d_rp2350.a` for LVGL scan-out

This document is for **cloud agents** building the Graphics4D static library so the
Embassy OxivGL/LVGL example can drive the gen4-RP2350-70CT-CLB panel.

## What the library is used for (LVGL use case)

Graphics4D is **not** the UI toolkit in this project. LVGL v9.5 (via OxivGL) draws
widgets into a PSRAM RGB565 framebuffer. Graphics4D is only the **panel driver**:

| Graphics4D API | Purpose in Embassy |
|----------------|-------------------|
| `gfx.Initialize()` | Panel reset, PIO RGB timing, PSRAM setup |
| `gfx.WriteToFrameBuffer(0, pixels, count)` | Copy LVGL framebuffer into scan-out buffer |
| `gfx.Refresh()` | Push frame to the physical 800×480 panel |

Workshop5 widget APIs (`gfx.Button()`, `gfx.Text()`, etc.) are **not** called.

Data flow:

```text
LVGL flush  →  PSRAM RGB565 (800×480)  →  gen4_lcd_present_rgb565()
                                              →  WriteToFrameBuffer + Refresh
                                              →  panel
```

Glue code lives in `c/display_gfx4d.cpp` (Embassy repo). It requires a global
`Graphics4D gfx` object linked from the static library.

## Build output (success criteria)

After a successful build, these must exist:

```text
lib/libgraphics4d_rp2350.a          # static archive
src/Graphics4D.h                    # public header (source tree)
```

For Embassy integration, also install into the vendored tree:

```text
vendor/graphics4d-rp2350/
  lib/libgraphics4d_rp2350.a
  include/Graphics4D.h
  include/...                       # other headers #included by Graphics4D.h
```

Verify in Embassy:

```bash
cd examples/gen4-rp2350-70ct-clb
./scripts/check-graphics4d.sh
cargo check --bin oxivgl_widget_demo --features oxivgl,touch
# build log must say: "Linking Graphics4D RGB scanout"
# must NOT say: "Embassy PIO+DPI scanout" as the only path
```

## Agent environment: Graphics4D-pico repo

When the agent workspace is `git@github.com:protronic/Graphics4D-pico.git`:

### 1. Source tree requirements

```bash
test -f src/Graphics4D.h
find src -name '*.cpp' -o -name '*.c' | wc -l   # expect >> 0 (typically 70+)
```

If only `Graphics4D.h` exists with no `.cpp` files, the Workshop5 export is
incomplete — stop and report.

Sources compiled for LVGL scan-out (Embassy CMake wrapper):

- all `src/*.c`, `src/*.cpp`, `src/*.cc`
- **excluded:** `no-OS-FatFS/`, `FatFS/`, `ff_*` (SD card not needed for display)

### 2. Install toolchain

**Ubuntu (cloud VM):**

```bash
sudo apt-get update
sudo apt-get install -y cmake ninja-build git gcc-arm-none-eabi libnewlib-arm-none-eabi
```

**Arch / CachyOS:**

```bash
sudo pacman -S --needed cmake ninja arm-none-eabi-gcc git
```

### 3. Pico SDK 2.x (RP2350)

```bash
git clone --depth 1 --branch 2.1.1 https://github.com/raspberrypi/pico-sdk.git /tmp/pico-sdk
git -C /tmp/pico-sdk submodule update --init --depth 1
export PICO_SDK_PATH=/tmp/pico-sdk
```

Required files:

```bash
test -f "$PICO_SDK_PATH/external/pico_sdk_import.cmake"
test -f "$PICO_SDK_PATH/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake"
```

### 4. Clone Embassy (board header + CMake wrapper only)

```bash
git clone --depth 1 --branch cursor/gen4-rp2350-70ct-clb-36d2 \
  https://github.com/protronic/embassy.git /tmp/embassy
```

Needed paths from Embassy:

```text
/tmp/embassy/examples/gen4-rp2350-70ct-clb/board/gen4_rp2350_70ct.h
/tmp/embassy/examples/gen4-rp2350-70ct-clb/cmake/graphics4d-lib/CMakeLists.txt
/tmp/embassy/examples/gen4-rp2350-70ct-clb/scripts/build-graphics4d-lib.sh
```

### 5. Environment variables

```bash
export PICO_SDK_PATH=/tmp/pico-sdk
export GEN4_GRAPHICS4D_SDK=$PWD          # Graphics4D-pico repo root
export PICO_BOARD=gen4_rp2350_70ct
```

### 6. Build (preferred: Embassy CMake wrapper)

From Graphics4D-pico repo root:

```bash
mkdir -p lib

EMBASSY=/tmp/embassy/examples/gen4-rp2350-70ct-clb
TOOLCHAIN=$PICO_SDK_PATH/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake

cmake -S "$EMBASSY/cmake/graphics4d-lib" -B build-embassy-cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
  -DGRAPHICS4D_SDK_PATH="$GEN4_GRAPHICS4D_SDK" \
  -DPICO_SDK_PATH="$PICO_SDK_PATH" \
  -DPICO_BOARD=gen4_rp2350_70ct \
  -DPICO_BOARD_HEADER_DIRS="$EMBASSY/board"

cmake --build build-embassy-cmake
cp build-embassy-cmake/libgraphics4d_rp2350.a lib/
ls -lh lib/libgraphics4d_rp2350.a
```

**Alternative one-liner** (if Embassy is cloned):

```bash
EMBASSY=/tmp/embassy/examples/gen4-rp2350-70ct-clb
bash "$EMBASSY/scripts/build-graphics4d-lib.sh"
# output: $GEN4_GRAPHICS4D_SDK/lib/libgraphics4d_rp2350.a
```

### 7. Compile flags (informational)

Target: **RP2350B Cortex-M33**, hard-float (`fpv5-sp-d16`).

Defines set by the Embassy wrapper:

```text
PICO_BOARD=gen4_rp2350_70ct
GEN4_RP2350_70CT
GEN4_RP2350_RGB
PICO_PLATFORM=rp2350
```

Linked Pico SDK components: `pico_stdlib`, `hardware_pio`, `hardware_dma`,
`hardware_gpio`, `hardware_sync`, `hardware_clocks`, `hardware_timer`,
`hardware_irq`, `pico_flash`, `hardware_flash`, `hardware_xip_cache`.

## Agent environment: Embassy repo

When the agent workspace is `protronic/embassy`:

```bash
cd examples/gen4-rp2350-70ct-clb

# Graphics4D-pico must be available:
export GEN4_GRAPHICS4D_SDK=/path/to/Graphics4D-pico
# OR
export GRAPHICS4D_PICO_TOKEN=ghp_...   # clones private repo automatically

export PICO_SDK_PATH=/tmp/pico-sdk     # or /usr/share/pico-sdk on Arch

./scripts/operate-graphics4d-worker.sh local --push \
  --branch cursor/gen4-rp2350-70ct-clb-36d2
```

This builds, vendors into `vendor/graphics4d-rp2350/`, runs `cargo check`, and
commits + pushes the `.a`.

## Install vendored lib into Embassy (manual)

```bash
cd examples/gen4-rp2350-70ct-clb
export GEN4_GRAPHICS4D_SDK=/path/to/Graphics4D-pico
./scripts/vendor-graphics4d-into-repo.sh
./scripts/check-graphics4d.sh
./scripts/commit-vendored-graphics4d.sh
```

## What is NOT required for LVGL scan-out

- Workshop5 IDE or generated widget code
- FatFS / SD card sources (excluded from build)
- Native Graphics4D-pico `CMakeLists.txt` parent layout (`../pico_sdk_import.cmake`)
  unless `GRAPHICS4D_NATIVE_CMAKE=1` is explicitly needed
- Rust toolchain for building the `.a` itself (only for post-build `cargo check`)

## Failure modes

| Symptom | Action |
|---------|--------|
| `Graphics4D.h not found` | Incomplete Graphics4D-pico tree |
| `No C/C++ sources under src/` | Workshop5 export missing implementation files |
| `Pico toolchain file missing` | Wrong Pico SDK version — use 2.1.x with RP2350 |
| `undefined reference to gfx` | Library must export global `Graphics4D gfx` — check link / glue |
| Build OK but blank panel | Use Graphics4D path, not stub; check USB log for `Graphics4D linked` |
| `Embassy PIO+DPI scanout` in log | `.a` not linked — run `check-graphics4d.sh` |

## Post-build: confirm LVGL path works

After vendoring and flashing `oxivgl_widget_demo`:

```text
panel: Graphics4D linked — RGB scanout active
oxivgl heartbeat: lvgl_flushes=N panel_presents=M   # both counters increase
```

Without Graphics4D, the example still renders via Embassy DPI (`src/dpi.rs`) —
that is a separate scan-out path, not a substitute `.a`.
