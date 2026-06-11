#!/usr/bin/env bash
# Build libgraphics4d_rp2350.a from Graphics4D-pico and install into the
# committed vendor tree: vendor/graphics4d-rp2350/
#
# Run once on Arch (or any Linux with Pico SDK 2.x), then commit:
#   git add vendor/graphics4d-rp2350
#   git commit -m "vendor(gen4): prebuilt libgraphics4d_rp2350.a"
#
# Prerequisites (Arch/CachyOS):
#   sudo pacman -S cmake ninja arm-none-eabi-gcc pico-sdk
#
# Usage:
#   export GEN4_GRAPHICS4D_SDK=./vendor/Graphics4D-pico
#   export PICO_SDK_PATH=/usr/share/pico-sdk
#   ./scripts/vendor-graphics4d-into-repo.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SDK="${GEN4_GRAPHICS4D_SDK:-${CRATE_DIR}/vendor/Graphics4D-pico}"
DEST="${CRATE_DIR}/vendor/graphics4d-rp2350"

SDK="$(cd "${SDK}" && pwd)"
export GEN4_GRAPHICS4D_SDK="${SDK}"

if [[ "${INSTALL_ONLY:-0}" == "1" ]]; then
    echo "=== install only (INSTALL_ONLY=1) ==="
    bash "${SCRIPT_DIR}/install-graphics4d-from-sdk.sh"
else
    echo "=== 1/2 build static library ==="
    bash "${SCRIPT_DIR}/build-graphics4d-lib.sh"
    echo "=== 2/2 install into vendor tree ==="
    bash "${SCRIPT_DIR}/install-graphics4d-from-sdk.sh"
fi

echo "=== validate (ARM-only, no host x86 objects) ==="
bash "${SCRIPT_DIR}/validate-graphics4d-lib.sh" "${DEST}/lib/libgraphics4d_rp2350.a"

echo "=== done ==="
ls -lh "${DEST}/lib/libgraphics4d_rp2350.a"
echo
echo "Commit into embassy:"
echo "  git add vendor/graphics4d-rp2350"
echo "  git commit -m 'vendor(gen4): prebuilt libgraphics4d_rp2350.a'"
