#!/usr/bin/env bash
# Fail if libgraphics4d_rp2350.a contains non-ARM objects (common Workshop5 mistake:
# host x86_64 Pico SDK tools merged into the firmware .a).
#
# Cloud agents and CI MUST pass this before commit/push.
#
# Usage:
#   ./scripts/validate-graphics4d-lib.sh path/to/libgraphics4d_rp2350.a
#   ./scripts/validate-graphics4d-lib.sh   # defaults to vendor lib

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LIB="${1:-${CRATE_DIR}/vendor/graphics4d-rp2350/lib/libgraphics4d_rp2350.a}"

die() { echo "validate-graphics4d-lib: error: $*" >&2; exit 1; }
warn() { echo "validate-graphics4d-lib: warning: $*" >&2; }

command -v readelf >/dev/null || die "readelf not found"
command -v ar >/dev/null || die "ar not found"

[[ -f "${LIB}" ]] || die "${LIB} not found"

# Reject HTML error pages saved as .a
if ! head -c 8 "${LIB}" | grep -q '^!<arch>'; then
    die "${LIB} is not a Unix ar archive (wrong download?)"
fi

tmpdir="$(mktemp -d)"
trap 'rm -rf "${tmpdir}"' EXIT

cd "${tmpdir}"
ar t "${LIB}" > members.txt || die "ar t failed on ${LIB}"

total=0
arm=0
host=0
host_list=""

while IFS= read -r member; do
    [[ -n "${member}" ]] || continue
    total=$((total + 1))
    ar p "${LIB}" "${member}" > "${member}" 2>/dev/null || continue
    machine="$(readelf -h "${member}" 2>/dev/null | awk '/Machine:/{for(i=2;i<=NF;i++) printf "%s%s", (i>2?" ":""), $i; print ""}')"
    case "${machine}" in
        ARM)
            arm=$((arm + 1))
            ;;
        *)
            host=$((host + 1))
            if [[ "${host}" -le 12 ]]; then
                host_list="${host_list}  - ${member} (${machine:-unknown})\n"
            fi
            ;;
    esac
done < members.txt

echo "validate-graphics4d-lib: ${LIB}"
echo "  objects: total=${total} arm=${arm} non-arm=${host}"

if [[ "${arm}" -eq 0 ]]; then
    die "no ARM objects — archive was not built with arm-none-eabi / pico_arm_cortex_m33_gcc.cmake"
fi

if [[ "${host}" -gt 0 ]]; then
    echo >&2
    echo "validate-graphics4d-lib: REJECTED — mixed host + ARM archive" >&2
    echo "  Every .o member must be ARM (thumbv8m.main-none-eabihf / Cortex-M33)." >&2
    echo "  Found ${host} host object(s), e.g.:" >&2
    printf "%b" "${host_list}" >&2
    echo >&2
    echo "  Cloud agent fix:" >&2
    echo "    1. Use Embassy CMake wrapper ONLY:" >&2
    echo "       cmake -S cmake/graphics4d-lib -B build-embassy-cmake -G Ninja \\" >&2
    echo "         -DCMAKE_TOOLCHAIN_FILE=\$PICO_SDK_PATH/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake \\" >&2
    echo "         -DGRAPHICS4D_SDK_PATH=\$GEN4_GRAPHICS4D_SDK -DPICO_SDK_PATH=\$PICO_SDK_PATH \\" >&2
    echo "         -DPICO_BOARD=gen4_rp2350_70ct -DPICO_BOARD_HEADER_DIRS=board" >&2
    echo "       cmake --build build-embassy-cmake" >&2
    echo "    2. Copy ONLY: build-embassy-cmake/libgraphics4d_rp2350.a" >&2
    echo "       Do NOT merge Workshop5 exports, build/ trees, or host tool .o files." >&2
    echo "    3. Re-run: ./scripts/validate-graphics4d-lib.sh lib/libgraphics4d_rp2350.a" >&2
    echo >&2
    echo "  See examples/gen4-rp2350-70ct-clb/AGENTS.md section 'Archive must be ARM-only'." >&2
    exit 1
fi

# Embassy uses cortex-m-rt link.x — warn if Pico app startup slipped in.
startup=0
while IFS= read -r member; do
    case "${member}" in
        crt0.S.o|newlib_interface.c.o|bs2_default_padded_checksummed.S.o)
            startup=$((startup + 1))
            warn "Pico startup object present: ${member} (may clash with Embassy link.x)"
            ;;
    esac
done < members.txt

if [[ "${startup}" -gt 0 ]]; then
    warn "${startup} Pico startup object(s) — prefer a clean graphics4d_rp2350 CMake target output"
fi

echo "  OK   all ${arm} object(s) are ARM"
exit 0
