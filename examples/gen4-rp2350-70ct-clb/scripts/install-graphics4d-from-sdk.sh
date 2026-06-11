#!/usr/bin/env bash
# Copy an existing libgraphics4d_rp2350.a + headers into vendor/graphics4d-rp2350/.
# Does NOT compile — use after checking out a prebuilt Graphics4D-pico release tag.
#
# Usage:
#   export GEN4_GRAPHICS4D_SDK=/path/to/Graphics4D-pico
#   ./scripts/install-graphics4d-from-sdk.sh
#
# Prebuilt release tag (Graphics4D-pico repo):
#   git fetch --tags origin
#   git checkout libgraphics4d-rp2350-20250611
#   export GEN4_GRAPHICS4D_SDK=$PWD
#   /path/to/embassy/examples/gen4-rp2350-70ct-clb/scripts/install-graphics4d-from-sdk.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SDK="${GEN4_GRAPHICS4D_SDK:-${CRATE_DIR}/vendor/Graphics4D-pico}"
DEST="${CRATE_DIR}/vendor/graphics4d-rp2350"

die() { echo "error: $*" >&2; exit 1; }

[[ -d "${SDK}" ]] || die "SDK not found: ${SDK}"
SDK="$(cd "${SDK}" && pwd)"

LIB_SRC="${SDK}/lib/libgraphics4d_rp2350.a"
[[ -f "${LIB_SRC}" ]] || die "${LIB_SRC} missing — build first or checkout prebuilt tag libgraphics4d-rp2350-*"

bash "${SCRIPT_DIR}/validate-graphics4d-lib.sh" "${LIB_SRC}"

echo "=== install from ${SDK} -> ${DEST} ==="
rm -rf "${DEST}"
mkdir -p "${DEST}/lib" "${DEST}/include"

cp -f "${LIB_SRC}" "${DEST}/lib/libgraphics4d_rp2350.a"
bash "${SCRIPT_DIR}/validate-graphics4d-lib.sh" "${DEST}/lib/libgraphics4d_rp2350.a"

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
    [[ -f "${SDK}/src/Graphics4D.h" ]] || die "Graphics4D.h not found under ${SDK}"
    cp -f "${SDK}/src/Graphics4D.h" "${DEST}/include/Graphics4D.h"
fi

TAG="$(git -C "${SDK}" describe --tags --exact-match 2>/dev/null || true)"
cat > "${DEST}/BUILD_INFO.txt" <<EOF
installed_utc=$(date -u +%Y-%m-%dT%H:%MZ)
source=${SDK}
git_tag=${TAG:-unknown}
host=$(uname -n)
EOF

echo "=== done ==="
ls -lh "${DEST}/lib/libgraphics4d_rp2350.a"
echo "Next: ./scripts/check-graphics4d.sh && cargo build --bin oxivgl_widget_demo --features oxivgl,touch"
