#!/usr/bin/env bash
# Build libgraphics4d_rp2350.a from a Graphics4D-pico source tree (Workshop5 export).
#
# Prerequisites:
#   - arm-none-eabi-gcc / g++ / ar in PATH
#   - PICO_SDK_PATH pointing at Raspberry Pi Pico SDK 2.x (with RP2350 support)
#
# Usage:
#   export PICO_SDK_PATH=~/pico/pico-sdk
#   ./scripts/build-graphics4d-lib.sh
#   GEN4_GRAPHICS4D_SDK=./vendor/Graphics4D-pico ./scripts/build-graphics4d-lib.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SDK="${GEN4_GRAPHICS4D_SDK:-${CRATE_DIR}/vendor/Graphics4D-pico}"
SDK="$(cd "${SDK}" && pwd)"
LIB_OUT="${SDK}/lib/libgraphics4d_rp2350.a"
BOARD_DIR="${CRATE_DIR}/board"
PICO_BOARD="${PICO_BOARD:-gen4_rp2350_70ct}"

die() { echo "error: $*" >&2; exit 1; }

[[ -d "${SDK}" ]] || die "SDK not found: ${SDK}"
[[ -f "${SDK}/src/Graphics4D.h" || -f "${SDK}/include/Graphics4D.h" || -f "${SDK}/Graphics4D.h" ]] \
    || die "Graphics4D.h not found under ${SDK}"

if [[ -z "${PICO_SDK_PATH:-}" ]]; then
    die "PICO_SDK_PATH is not set (need Pico SDK 2.x with RP2350 / gen4 board headers)"
fi
[[ -d "${PICO_SDK_PATH}" ]] || die "PICO_SDK_PATH=${PICO_SDK_PATH} is not a directory"

command -v arm-none-eabi-g++ >/dev/null || die "arm-none-eabi-g++ not found"
command -v arm-none-eabi-ar >/dev/null || die "arm-none-eabi-ar not found"

mkdir -p "${SDK}/lib"

# Prefer the library's own Embassy/CMake recipe when present.
if [[ -f "${SDK}/embassy/build.sh" ]]; then
    echo "Running ${SDK}/embassy/build.sh ..."
    PICO_SDK_PATH="${PICO_SDK_PATH}" PICO_BOARD="${PICO_BOARD}" BOARD_DIR="${BOARD_DIR}" \
        bash "${SDK}/embassy/build.sh"
    [[ -f "${LIB_OUT}" ]] || die "embassy/build.sh did not produce ${LIB_OUT}"
    echo "Built ${LIB_OUT}"
    exit 0
fi

EMBASSY_CMAKE="${CRATE_DIR}/cmake/graphics4d-lib"
if [[ -d "${EMBASSY_CMAKE}" ]] && command -v cmake >/dev/null; then
    BUILD_DIR="${SDK}/build-embassy-cmake"
    TOOLCHAIN="${PICO_SDK_PATH}/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake"
    [[ -f "${TOOLCHAIN}" ]] || die "Pico toolchain file missing: ${TOOLCHAIN}"

    echo "CMake build (embassy wrapper) in ${BUILD_DIR} ..."
    cmake -S "${EMBASSY_CMAKE}" -B "${BUILD_DIR}" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
        -DPICO_SDK_PATH="${PICO_SDK_PATH}" \
        -DPICO_BOARD="${PICO_BOARD}"
    env GRAPHICS4D_SDK="${SDK}" cmake --build "${BUILD_DIR}"

    mapfile -t BUILT < <(find "${BUILD_DIR}" -name 'libgraphics4d_rp2350.a' 2>/dev/null)
    [[ ${#BUILT[@]} -gt 0 ]] || die "CMake wrapper build produced no libgraphics4d_rp2350.a"
    cp -f "${BUILT[0]}" "${LIB_OUT}"
    echo "Built ${LIB_OUT} (from ${BUILT[0]})"
    exit 0
fi

if [[ -f "${SDK}/CMakeLists.txt" ]] && command -v cmake >/dev/null; then
    BUILD_DIR="${SDK}/build-embassy"
    TOOLCHAIN="${PICO_SDK_PATH}/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake"
    [[ -f "${TOOLCHAIN}" ]] || die "Pico toolchain file missing: ${TOOLCHAIN}"

    echo "CMake build in ${BUILD_DIR} ..."
    cmake -S "${SDK}" -B "${BUILD_DIR}" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DPICO_SDK_PATH="${PICO_SDK_PATH}" \
        -DPICO_BOARD="${PICO_BOARD}" \
        -DPICO_BOARD_HEADER_DIRS="${BOARD_DIR}" \
        -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}"
    cmake --build "${BUILD_DIR}"

    # Pick up the archive wherever the Graphics4D CMake target wrote it.
    mapfile -t BUILT < <(find "${BUILD_DIR}" -name 'libgraphics4d*.a' -o -name 'libGraphics4D*.a' 2>/dev/null | head -5)
    if [[ ${#BUILT[@]} -eq 0 ]]; then
        die "CMake build finished but no libgraphics4d*.a found under ${BUILD_DIR}"
    fi
    cp -f "${BUILT[0]}" "${LIB_OUT}"
    echo "Built ${LIB_OUT} (from ${BUILT[0]})"
    exit 0
fi

# Fallback: compile all C/C++ sources under src/ into one static library.
OBJ_DIR="${SDK}/build-embassy-obj"
rm -rf "${OBJ_DIR}"
mkdir -p "${OBJ_DIR}"

mapfile -t SOURCES < <(find "${SDK}/src" -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.c' \) | sort)
[[ ${#SOURCES[@]} -gt 0 ]] || die "no .c/.cpp sources under ${SDK}/src — cannot build static lib"

COMMON_FLAGS=(
    -mcpu=cortex-m33
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv5-sp-d16
    -Os
    -ffunction-sections
    -fdata-sections
    -fno-exceptions
    -fno-rtti
    -fno-threadsafe-statics
    -DPICO_BOARD="${PICO_BOARD}"
    -DPICO_PLATFORM=rp2350
    -DPICO_RP2350A=0
    -DGEN4_RP2350_70CT
    -DGEN4_RP2350_RGB
)

PICO_INCLUDES=(
    "${PICO_SDK_PATH}/src/common/pico_base_headers/include"
    "${PICO_SDK_PATH}/src/rp2_common/pico_stdlib/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_pio/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_dma/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_sync/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_clocks/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_timer/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_irq/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_pll/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_vreg/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_xosc/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_watchdog/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_uart/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_flash/include"
    "${PICO_SDK_PATH}/src/rp2_common/hardware_xip_cache/include"
    "${PICO_SDK_PATH}/src/rp2350/pico_platform/include"
    "${PICO_SDK_PATH}/src/rp2350/hardware_regs/include"
    "${PICO_SDK_PATH}/src/rp2350/hardware_structs/include"
    "${PICO_SDK_PATH}/src/boards/include"
    "${BOARD_DIR}"
    "${SDK}/src"
    "${SDK}/include"
)

INC_FLAGS=()
for inc in "${PICO_INCLUDES[@]}"; do
    [[ -d "${inc}" ]] && INC_FLAGS+=(-I"${inc}")
done

echo "Compiling ${#SOURCES[@]} Graphics4D source file(s) ..."
OBJS=()
for src in "${SOURCES[@]}"; do
    base="$(basename "${src}")"
    obj="${OBJ_DIR}/${base%.*}.o"
    case "${src}" in
        *.c)
            arm-none-eabi-gcc -c "${COMMON_FLAGS[@]}" "${INC_FLAGS[@]}" -o "${obj}" "${src}"
            ;;
        *)
            arm-none-eabi-g++ -c "${COMMON_FLAGS[@]}" "${INC_FLAGS[@]}" -o "${obj}" "${src}"
            ;;
    esac
    OBJS+=("${obj}")
done

rm -f "${LIB_OUT}"
arm-none-eabi-ar rcs "${LIB_OUT}" "${OBJS[@]}"
echo "Built ${LIB_OUT} (${#OBJS[@]} objects)"
echo "Next: ./scripts/check-graphics4d.sh && cargo build --bin oxivgl_widget_demo --features oxivgl,touch"
