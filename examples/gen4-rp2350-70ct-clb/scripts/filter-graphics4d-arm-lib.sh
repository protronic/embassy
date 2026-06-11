#!/usr/bin/env bash
# Extract ARM ELF32 objects from a Workshop5 .a that may also contain host x86_64
# objects, and drop Pico startup objects that clash with Embassy's link.x.
#
# Usage: filter-graphics4d-arm-lib.sh INPUT.a OUTPUT.a

set -euo pipefail

in="${1:?input .a}"
out="${2:?output .a}"

tmpdir="$(mktemp -d)"
trap 'rm -rf "${tmpdir}"' EXIT

cd "${tmpdir}"
ar x "${in}"

rm -f "${out}"
count=0
for obj in *.o; do
    [[ -f "${obj}" ]] || continue
    case "${obj}" in
        crt0.S.o|newlib_interface.c.o|bs2_default_padded_checksummed.S.o)
            continue
            ;;
    esac
    if readelf -h "${obj}" 2>/dev/null | grep -q 'Machine:.*ARM'; then
        arm-none-eabi-ar q "${out}" "${obj}"
        count=$((count + 1))
    fi
done

[[ "${count}" -gt 0 ]] || {
    echo "filter-graphics4d-arm-lib: no ARM objects in ${in}" >&2
    exit 1
}

echo "filter-graphics4d-arm-lib: ${count} ARM object(s) -> ${out}" >&2
