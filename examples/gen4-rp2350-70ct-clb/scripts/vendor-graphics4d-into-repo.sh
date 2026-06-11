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

echo "=== 1/3 build static library ==="
bash "${SCRIPT_DIR}/build-graphics4d-lib.sh"

SDK="$(cd "${SDK}" && pwd)"
LIB_SRC="${SDK}/lib/libgraphics4d_rp2350.a"
[[ -f "${LIB_SRC}" ]] || { echo "error: ${LIB_SRC} missing after build" >&2; exit 1; }

echo "=== 2/3 install into ${DEST} ==="
rm -rf "${DEST}"
mkdir -p "${DEST}/lib" "${DEST}/include"

cp -f "${LIB_SRC}" "${DEST}/lib/libgraphics4d_rp2350.a"

# Copy public headers (Graphics4D.h pulls in others from src/).
while IFS= read -r -d '' hdr; do
    rel="${hdr#${SDK}/}"
    case "${rel}" in
        src/*) rel="${rel#src/}" ;;
        include/*) rel="${rel#include/}" ;;
        *) continue ;;
    esac
    install -D -m 0644 "${hdr}" "${DEST}/include/${rel}"
done < <(find "${SDK}" \( -path "${SDK}/src/*.h" -o -path "${SDK}/include/*.h" \) -print0 2>/dev/null)

if [[ ! -f "${DEST}/include/Graphics4D.h" ]]; then
    cp -f "${SDK}/src/Graphics4D.h" "${DEST}/include/Graphics4D.h"
fi

cat > "${DEST}/BUILD_INFO.txt" <<EOF
built_utc=$(date -u +%Y-%m-%dT%H:%MZ)
source=${SDK}
host=$(uname -n)
EOF

echo "=== 3/3 done ==="
ls -lh "${DEST}/lib/libgraphics4d_rp2350.a"
echo
echo "Commit into embassy:"
echo "  git add vendor/graphics4d-rp2350"
echo "  git commit -m 'vendor(gen4): prebuilt libgraphics4d_rp2350.a'"
