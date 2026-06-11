#!/usr/bin/env bash
# Compile Pico SDK sources and merge into libpico_embassy.a for Embassy + Graphics4D.
# Skips crt0/stdio objects that clash with cortex-m-rt link.x.
#
# Usage:
#   PICO_SDK_PATH=... OUT_DIR=... BOARD_DIR=... ./scripts/build-pico-embassy-lib.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CMAKE_DIR="${CRATE_DIR}/cmake/pico-embassy-lib"
BOARD_DIR="${BOARD_DIR:-${CRATE_DIR}/board}"
OUT_DIR="${OUT_DIR:?OUT_DIR required}"
BUILD_DIR="${OUT_DIR}/build"
LIB_OUT="${OUT_DIR}/libpico_embassy.a"

die() { echo "build-pico-embassy-lib: error: $*" >&2; exit 1; }

[[ -n "${PICO_SDK_PATH:-}" ]] || die "PICO_SDK_PATH is not set"
[[ -d "${PICO_SDK_PATH}" ]] || die "PICO_SDK_PATH=${PICO_SDK_PATH} missing"
[[ -f "${PICO_SDK_PATH}/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake" ]] \
    || die "Pico RP2350 toolchain file missing under PICO_SDK_PATH"

TOOLCHAIN="${PICO_SDK_PATH}/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake"
mkdir -p "${OUT_DIR}"

cmake -S "${CMAKE_DIR}" -B "${BUILD_DIR}" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
    -DPICO_SDK_PATH="${PICO_SDK_PATH}" \
    -DPICO_BOARD=gen4_rp2350_70ct \
    -DPICO_PLATFORM=rp2350 \
    -DPICO_BOARD_HEADER_DIRS="${BOARD_DIR}"

# Link step fails (no main) — object files are still produced.
cmake --build "${BUILD_DIR}" --target pico_embassy_objects || true

OBJ_DIR="${BUILD_DIR}/CMakeFiles/pico_embassy_objects.dir"
[[ -d "${OBJ_DIR}" ]] || die "object dir missing: ${OBJ_DIR}"

drop_object() {
    case "$1" in
        anchor.c.o|crt0.S.o|newlib_interface.c.o|runtime_init.c.o|runtime_init_clocks.c.o|runtime_init_stack_guard.c.o|stdio.c.o|stdio_uart.c.o|standard_binary_info.c.o|panic.c.o|mutex.c.o|runtime.c.o|malloc.c.o)
            return 0
            ;;
    esac
    return 1
}

rm -f "${LIB_OUT}"
count=0
while IFS= read -r obj; do
    name="$(basename "${obj}")"
    drop_object "${name}" && continue
    arm-none-eabi-ar q "${LIB_OUT}" "${obj}"
    count=$((count + 1))
done < <(find "${OBJ_DIR}" -name '*.o' | sort)

[[ "${count}" -gt 0 ]] || die "no Pico SDK objects collected"
echo "build-pico-embassy-lib: ${count} object(s) -> ${LIB_OUT}" >&2
