#!/usr/bin/env bash
# Install toolchain for Graphics4D builds (Ubuntu CI / cloud agent VM).
#
# Usage: ./scripts/ci-install-graphics4d-deps.sh
# Sets PICO_SDK_PATH when not already set.

set -euo pipefail

PICO_SDK_DEST="${PICO_SDK_DEST:-/tmp/pico-sdk}"

if [[ -n "${PICO_SDK_PATH:-}" && -f "${PICO_SDK_PATH}/external/pico_sdk_import.cmake" ]]; then
    echo "PICO_SDK_PATH already set: ${PICO_SDK_PATH}"
    exit 0
fi

if [[ -f "${PICO_SDK_DEST}/external/pico_sdk_import.cmake" ]]; then
    export PICO_SDK_PATH="${PICO_SDK_DEST}"
    echo "PICO_SDK_PATH=${PICO_SDK_PATH}"
    exit 0
fi

echo "=== apt: arm-none-eabi + build tools ==="
if command -v apt-get >/dev/null; then
    sudo apt-get update -qq
    sudo apt-get install -y --no-install-recommends \
        cmake ninja-build git gcc-arm-none-eabi libnewlib-arm-none-eabi
fi

echo "=== clone Pico SDK ==="
rm -rf "${PICO_SDK_DEST}"
git clone --depth 1 --branch 2.1.1 https://github.com/raspberrypi/pico-sdk.git "${PICO_SDK_DEST}"
git -C "${PICO_SDK_DEST}" submodule update --init --depth 1

export PICO_SDK_PATH="${PICO_SDK_DEST}"
echo "PICO_SDK_PATH=${PICO_SDK_PATH}"

if ! command -v rustup >/dev/null; then
    echo "warning: rustup not found — cargo check may fail"
elif ! rustup target list --installed | grep -q thumbv8m.main-none-eabihf; then
    rustup target add thumbv8m.main-none-eabihf
fi
