#!/usr/bin/env bash
# Clone protronic/Graphics4D-pico into vendor/ for RGB panel scan-out.
#
# Usage (from this example crate):
#   ./scripts/init-graphics4d-pico.sh
#
# Or from the embassy repo root:
#   examples/gen4-rp2350-70ct-clb/scripts/init-graphics4d-pico.sh
#
# Override the remote:
#   GRAPHICS4D_PICO_URL=git@github.com:protronic/Graphics4D-pico.git ./scripts/init-graphics4d-pico.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEST="${CRATE_DIR}/vendor/Graphics4D-pico"
URL="${GRAPHICS4D_PICO_URL:-https://github.com/protronic/Graphics4D-pico.git}"

if [[ -f "${DEST}/include/Graphics4D.h" || -f "${DEST}/Graphics4D.h" ]]; then
    echo "Graphics4D-pico already present at ${DEST}"
    exit 0
fi

if [[ -d "${DEST}/.git" ]]; then
    echo "Updating existing clone in ${DEST} ..."
    git -C "${DEST}" pull --ff-only
    exit 0
fi

mkdir -p "$(dirname "${DEST}")"
echo "Cloning ${URL} -> ${DEST}"
git clone "${URL}" "${DEST}"

if [[ ! -f "${DEST}/include/Graphics4D.h" && ! -f "${DEST}/Graphics4D.h" ]]; then
    echo "warning: clone succeeded but Graphics4D.h was not found — check the repository layout" >&2
fi
