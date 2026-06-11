#!/usr/bin/env bash
# Build libgraphics4d_rp2350.a from a Graphics4D-pico source tree (Workshop5 export).
#
# Prerequisites:
#   - arm-none-eabi-gcc / g++ / ar in PATH
#   - cmake + ninja
#   - PICO_SDK_PATH pointing at Raspberry Pi Pico SDK 2.x (with RP2350 support)
#
# Usage:
#   export PICO_SDK_PATH=/usr/share/pico-sdk
#   export GEN4_GRAPHICS4D_SDK=./vendor/Graphics4D-pico
#   ./scripts/build-graphics4d-lib.sh
#
# The native Graphics4D-pico CMakeLists.txt expects a Workshop5 parent project
# (../pico_sdk_import.cmake). By default we use the Embassy CMake wrapper instead.
# Set GRAPHICS4D_NATIVE_CMAKE=1 to try the upstream CMake after symlinking the import.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SDK="${GEN4_GRAPHICS4D_SDK:-${CRATE_DIR}/vendor/Graphics4D-pico}"
SDK="$(cd "${SDK}" && pwd)"
LIB_OUT="${SDK}/lib/libgraphics4d_rp2350.a"
BOARD_DIR="${CRATE_DIR}/board"
PICO_BOARD="${PICO_BOARD:-gen4_rp2350_70ct}"
EMBASSY_CMAKE="${CRATE_DIR}/cmake/graphics4d-lib"
VENDOR_DIR="$(dirname "${SDK}")"

die() { echo "error: $*" >&2; exit 1; }
warn() { echo "warning: $*" >&2; }

[[ -d "${SDK}" ]] || die "SDK not found: ${SDK}"
[[ -f "${SDK}/src/Graphics4D.h" || -f "${SDK}/include/Graphics4D.h" || -f "${SDK}/Graphics4D.h" ]] \
    || die "Graphics4D.h not found under ${SDK}"

if [[ -z "${PICO_SDK_PATH:-}" ]]; then
    for candidate in /usr/share/pico-sdk /usr/lib/pico-sdk "${HOME}/pico/pico-sdk" "${HOME}/pico-sdk"; do
        if [[ -d "${candidate}/external/pico_sdk_import.cmake" || -f "${candidate}/external/pico_sdk_import.cmake" ]]; then
            PICO_SDK_PATH="${candidate}"
            export PICO_SDK_PATH
            echo "Using PICO_SDK_PATH=${PICO_SDK_PATH}"
            break
        fi
    done
fi
[[ -n "${PICO_SDK_PATH:-}" ]] || die "PICO_SDK_PATH is not set (Pico SDK 2.x with RP2350 support; Arch: /usr/share/pico-sdk)"
[[ -d "${PICO_SDK_PATH}" ]] || die "PICO_SDK_PATH=${PICO_SDK_PATH} is not a directory"

command -v arm-none-eabi-g++ >/dev/null || die "arm-none-eabi-g++ not found (Arch: sudo pacman -S arm-none-eabi-gcc)"
command -v arm-none-eabi-ar >/dev/null || die "arm-none-eabi-ar not found"
command -v cmake >/dev/null || die "cmake not found (Arch: sudo pacman -S cmake)"
command -v ninja >/dev/null || die "ninja not found (Arch: sudo pacman -S ninja)"

TOOLCHAIN="${PICO_SDK_PATH}/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake"
[[ -f "${TOOLCHAIN}" ]] || die "Pico toolchain file missing: ${TOOLCHAIN}"

mkdir -p "${SDK}/lib"

export PICO_SDK_PATH
export GRAPHICS4D_SDK="${SDK}"

copy_built_lib() {
    local build_dir="$1"
    mapfile -t BUILT < <(
        find "${build_dir}" -type f \( \
            -name 'libgraphics4d_rp2350.a' -o \
            -name 'libgraphics4d.a' -o \
            -name 'libGraphics4D.a' \
        \) 2>/dev/null | sort
    )
    [[ ${#BUILT[@]} -gt 0 ]] || return 1
    cp -f "${BUILT[0]}" "${LIB_OUT}"
    echo "Built ${LIB_OUT} (from ${BUILT[0]})"
    return 0
}

setup_workshop5_pico_import() {
    local import_src="${PICO_SDK_PATH}/external/pico_sdk_import.cmake"
    [[ -f "${import_src}" ]] || die "missing ${import_src}"
    ln -sfn "${import_src}" "${VENDOR_DIR}/pico_sdk_import.cmake"
    echo "Linked ${VENDOR_DIR}/pico_sdk_import.cmake -> Pico SDK import"
}

try_embassy_cmake() {
    [[ -d "${EMBASSY_CMAKE}" ]] || return 1
    local build_dir="${SDK}/build-embassy-cmake"
    echo "=== CMake (Embassy wrapper) -> ${build_dir} ==="
    rm -rf "${build_dir}"
    cmake -S "${EMBASSY_CMAKE}" -B "${build_dir}" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
        -DGRAPHICS4D_SDK_PATH="${SDK}" \
        -DPICO_SDK_PATH="${PICO_SDK_PATH}" \
        -DPICO_BOARD="${PICO_BOARD}" \
        -DPICO_BOARD_HEADER_DIRS="${BOARD_DIR}"
    cmake --build "${build_dir}"
    copy_built_lib "${build_dir}"
}

try_native_cmake() {
    [[ -f "${SDK}/CMakeLists.txt" ]] || return 1
    setup_workshop5_pico_import
    local build_dir="${SDK}/build-embassy"
    echo "=== CMake (Graphics4D-pico native) -> ${build_dir} ==="
    rm -rf "${build_dir}"
    cmake -S "${SDK}" -B "${build_dir}" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
        -DPICO_SDK_PATH="${PICO_SDK_PATH}" \
        -DPICO_BOARD="${PICO_BOARD}" \
        -DPICO_BOARD_HEADER_DIRS="${BOARD_DIR}"
    cmake --build "${build_dir}"
    copy_built_lib "${build_dir}"
}

try_manual_compile() {
    local obj_dir="${SDK}/build-embassy-obj"
    echo "=== Manual compile -> ${obj_dir} ==="
    rm -rf "${obj_dir}"
    mkdir -p "${obj_dir}"

    mapfile -t SOURCES < <(
        find "${SDK}/src" -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.c' \) \
            ! -path '*/no-OS-FatFS/*' \
            ! -path '*/FatFS/*' \
            | sort
    )
    [[ ${#SOURCES[@]} -gt 0 ]] || return 1

    local common_flags=(
        -mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16
        -Os -ffunction-sections -fdata-sections
        -fno-exceptions -fno-rtti -fno-threadsafe-statics
        -DPICO_BOARD="${PICO_BOARD}" -DPICO_PLATFORM=rp2350 -DPICO_RP2350A=0
        -DGEN4_RP2350_70CT -DGEN4_RP2350_RGB
    )

    local pico_includes=(
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
        "${BOARD_DIR}" "${SDK}/src" "${SDK}/include"
    )

    local inc_flags=()
    local inc
    for inc in "${pico_includes[@]}"; do
        [[ -d "${inc}" ]] && inc_flags+=(-I"${inc}")
    done

    echo "Compiling ${#SOURCES[@]} source file(s) ..."
    local objs=() src rel obj
    for src in "${SOURCES[@]}"; do
        rel="${src#${SDK}/}"
        obj="${obj_dir}/${rel%.*}.o"
        mkdir -p "$(dirname "${obj}")"
        case "${src}" in
            *.c) arm-none-eabi-gcc -c "${common_flags[@]}" "${inc_flags[@]}" -o "${obj}" "${src}" ;;
            *)   arm-none-eabi-g++ -c "${common_flags[@]}" "${inc_flags[@]}" -o "${obj}" "${src}" ;;
        esac
        objs+=("${obj}")
    done

    rm -f "${LIB_OUT}"
    arm-none-eabi-ar rcs "${LIB_OUT}" "${objs[@]}"
    echo "Built ${LIB_OUT} (${#objs[@]} objects)"
    return 0
}

# 1) Optional library-maintained script.
if [[ -f "${SDK}/embassy/build.sh" ]]; then
    echo "=== ${SDK}/embassy/build.sh ==="
    PICO_BOARD="${PICO_BOARD}" BOARD_DIR="${BOARD_DIR}" bash "${SDK}/embassy/build.sh"
    [[ -f "${LIB_OUT}" ]] && exit 0
    warn "embassy/build.sh did not produce ${LIB_OUT}, trying other methods"
fi

# 2) Embassy CMake wrapper (default — works without Workshop5 parent layout).
if try_embassy_cmake; then
    echo "Next: ./scripts/check-graphics4d.sh && cargo build --bin oxivgl_widget_demo --features oxivgl,touch"
    exit 0
fi
warn "Embassy CMake wrapper failed"

# 3) Native Graphics4D-pico CMake (opt-in — needs ../pico_sdk_import.cmake symlink).
if [[ "${GRAPHICS4D_NATIVE_CMAKE:-0}" == "1" ]]; then
    if try_native_cmake; then
        exit 0
    fi
    warn "native Graphics4D-pico CMake failed"
else
    echo "Skipping native Graphics4D-pico CMake (set GRAPHICS4D_NATIVE_CMAKE=1 to try)"
fi

# 4) Manual compile fallback.
if try_manual_compile; then
    echo "Next: ./scripts/check-graphics4d.sh && cargo build --bin oxivgl_widget_demo --features oxivgl,touch"
    exit 0
fi

die "all build methods failed — post the full log from this script"
