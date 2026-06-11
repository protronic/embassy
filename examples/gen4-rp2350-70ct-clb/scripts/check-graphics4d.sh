#!/usr/bin/env bash
# Report whether Graphics4D is discoverable for this example crate.
#
# Usage:
#   ./scripts/check-graphics4d.sh
#   GEN4_GRAPHICS4D_SDK=/path/to/sdk ./scripts/check-graphics4d.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PREBUILT="${CRATE_DIR}/vendor/graphics4d-rp2350"
VENDOR="${CRATE_DIR}/vendor/Graphics4D-pico"

ok() { printf '  OK   %s\n' "$1"; }
miss() { printf '  MISS %s\n' "$1"; }
hint() { printf '  HINT %s\n' "$1"; }

find_static_lib() {
    local root="$1"
    local lib
    while IFS= read -r lib; do
        if [[ -f "${lib}" ]]; then
            echo "${lib}"
            return 0
        fi
    done < <(
        find "${root}" \( -path '*/build-embassy-obj/*' -o -path '*/.git/*' \) -prune -o \
            -type f \( -name 'libgraphics4d_rp2350.a' -o -name 'libgraphics4d.a' -o -name 'libGraphics4D.a' \) -print 2>/dev/null \
            | sort
    )
    return 1
}

count_sources() {
    local root="$1"
    find "${root}/src" -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.c' \) 2>/dev/null | wc -l
}

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

    local lib
    if lib="$(find_static_lib "${root}")"; then
        ok "static lib ${lib#${root}/}"
        if bash "${SCRIPT_DIR}/validate-graphics4d-lib.sh" "${lib}"; then
            ok "archive architecture (ARM-only)"
        else
            miss "archive architecture — mixed host x86_64 + ARM (rebuild with validate-graphics4d-lib.sh)"
            hint "cloud agent: copy ONLY build-embassy-cmake/libgraphics4d_rp2350.a — see AGENTS.md"
            return 1
        fi
        [[ "${found_header}" -eq 1 ]]
        return
    fi
    miss "libgraphics4d_rp2350.a"

    local n_sources
    n_sources="$(count_sources "${root}")"
    if [[ "${n_sources}" -gt 0 ]]; then
        ok "sources ${n_sources} file(s) under src/"
        hint "build the static lib: PICO_SDK_PATH=... ./scripts/build-graphics4d-lib.sh"
    else
        miss "C/C++ sources under src/"
    fi

    if [[ -f "${root}/CMakeLists.txt" ]]; then
        hint "CMakeLists.txt present — build-graphics4d-lib.sh will try CMake + Pico SDK"
    fi
    if [[ -f "${root}/embassy/build.sh" ]]; then
        hint "embassy/build.sh present — preferred build recipe"
    fi

    return 1
}

echo "Graphics4D check for gen4-rp2350-70ct-clb"
echo

status=1
if [[ -n "${GEN4_GRAPHICS4D_SDK:-}" ]]; then
    check_root "${GEN4_GRAPHICS4D_SDK}" "GEN4_GRAPHICS4D_SDK" && status=0
    echo
fi

if check_root "${PREBUILT}" "vendor/graphics4d-rp2350 (committed)"; then
    status=0
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
    echo "Graphics4D NOT ready — firmware uses Embassy PIO+DPI scanout (src/dpi.rs)."
    echo "To link Graphics4D instead, vendor libgraphics4d_rp2350.a (see below)."
    echo
    echo "Prebuilt tag (Graphics4D-pico repo, no compile):"
    echo "  git fetch --tags origin && git checkout libgraphics4d-rp2350-20250611"
    echo "  export GEN4_GRAPHICS4D_SDK=\$PWD"
    echo "  INSTALL_ONLY=1 ./scripts/vendor-graphics4d-into-repo.sh"
    echo
    echo "Or build from sources (Arch/CachyOS):"
    echo "  sudo pacman -S cmake ninja arm-none-eabi-gcc pico-sdk"
    echo "  export PICO_SDK_PATH=/usr/share/pico-sdk"
    echo "  export GEN4_GRAPHICS4D_SDK=./vendor/Graphics4D-pico"
    echo "  ./scripts/vendor-graphics4d-into-repo.sh"
    echo "  ./scripts/commit-vendored-graphics4d.sh"
    echo "  ./scripts/check-graphics4d.sh"
    echo "  cargo build --bin oxivgl_widget_demo --features oxivgl,touch"
    echo
    echo "Diagnose your tree:"
    echo "  ./scripts/inspect-graphics4d-pico.sh"
    echo
    echo "Windows (Workshop5): after compiling any RP2350 project:"
    echo "  powershell -File scripts/find-graphics4d-lib.ps1"
    echo "  copy found .a -> vendor/Graphics4D-pico/lib/libgraphics4d_rp2350.a"
fi

exit "${status}"
