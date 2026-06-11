#!/usr/bin/env bash
# Report whether Graphics4D is discoverable for this example crate.
#
# Usage:
#   ./scripts/check-graphics4d.sh
#   GEN4_GRAPHICS4D_SDK=/path/to/sdk ./scripts/check-graphics4d.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENDOR="${CRATE_DIR}/vendor/Graphics4D-pico"

ok() { printf '  OK   %s\n' "$1"; }
miss() { printf '  MISS %s\n' "$1"; }

check_root() {
    local root="$1"
    local label="$2"
    echo "${label}: ${root}"

    if [[ ! -d "${root}" ]]; then
        miss "directory does not exist"
        return 1
    fi

    local found_header=0
    for h in \
        "${root}/include/Graphics4D.h" \
        "${root}/Graphics4D.h" \
        "${root}/src/Graphics4D.h"
    do
        if [[ -f "${h}" ]]; then
            ok "header ${h#${root}/}"
            found_header=1
            break
        fi
    done
    [[ "${found_header}" -eq 1 ]] || miss "Graphics4D.h (tried include/, root, src/)"

    local found_lib=0
    for lib in \
        "${root}/lib/libgraphics4d_rp2350.a" \
        "${root}/lib/libgraphics4d.a" \
        "${root}/lib/libGraphics4D.a" \
        "${root}/libgraphics4d_rp2350.a"
    do
        if [[ -f "${lib}" ]]; then
            ok "static lib ${lib#${root}/}"
            found_lib=1
            break
        fi
    done
    [[ "${found_lib}" -eq 1 ]] || miss "libgraphics4d_rp2350.a (tried lib/ and root)"

    [[ "${found_header}" -eq 1 && "${found_lib}" -eq 1 ]]
}

echo "Graphics4D check for gen4-rp2350-70ct-clb"
echo

status=1
if [[ -n "${GEN4_GRAPHICS4D_SDK:-}" ]]; then
    check_root "${GEN4_GRAPHICS4D_SDK}" "GEN4_GRAPHICS4D_SDK" && status=0
    echo
fi

if check_root "${VENDOR}" "vendor/Graphics4D-pico"; then
    status=0
fi

echo
if [[ "${status}" -eq 0 ]]; then
    echo "Graphics4D looks ready — rebuild and reflash:"
    echo "  cargo build --bin oxivgl_widget_demo --features oxivgl,touch"
    echo "Boot log should show: panel: Graphics4D linked — RGB scanout active"
else
    echo "Graphics4D NOT ready — current firmware uses the scanout STUB (blank panel)."
    echo
    echo "Option A — clone companion repo (once it exists on GitHub):"
    echo "  ./scripts/init-graphics4d-pico.sh"
    echo
    echo "Option B — point at a local Workshop5 / exported SDK tree:"
    echo "  export GEN4_GRAPHICS4D_SDK=/path/to/graphics4d-rp2350"
    echo "  cargo build --bin oxivgl_widget_demo --features oxivgl,touch"
    echo
    echo "Typical Workshop5 layout on Windows (adjust drive letter):"
    echo "  C:/Users/<you>/AppData/Local/4D Systems/Workshop5/..."
    echo "Search for Graphics4D.h and libgraphics4d_rp2350.a, then export GEN4_GRAPHICS4D_SDK"
    echo "to the directory that contains include/ and lib/."
fi

exit "${status}"
