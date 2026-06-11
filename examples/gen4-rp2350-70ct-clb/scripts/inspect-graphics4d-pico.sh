#!/usr/bin/env bash
# Show what is inside vendor/Graphics4D-pico (or GEN4_GRAPHICS4D_SDK).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SDK="${GEN4_GRAPHICS4D_SDK:-${CRATE_DIR}/vendor/Graphics4D-pico}"

echo "=== Graphics4D SDK tree: ${SDK} ==="
if [[ ! -d "${SDK}" ]]; then
    echo "directory missing"
    exit 1
fi

echo
echo "-- top level --"
ls -la "${SDK}"

echo
echo "-- headers --"
find "${SDK}" -maxdepth 3 -name 'Graphics4D.h' -print 2>/dev/null || true

echo
echo "-- C/C++ sources (maxdepth 4) --"
find "${SDK}" -maxdepth 4 -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.c' \) | sort || true

echo
echo "-- static libraries (*.a) --"
find "${SDK}" -type f -name '*.a' 2>/dev/null | sort || true

echo
echo "-- CMake / build helpers --"
for f in CMakeLists.txt embassy/build.sh embassy/CMakeLists.txt; do
    [[ -f "${SDK}/${f}" ]] && echo "  ${f}"
done

SRC_COUNT=$(find "${SDK}/src" -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.c' \) 2>/dev/null | wc -l)
echo
echo "source files under src/: ${SRC_COUNT}"

if [[ "${SRC_COUNT}" -eq 0 ]]; then
    echo
    echo ">>> Keine .cpp/.c unter src/ — Workshop5 liefert die Implementierung oft"
    echo ">>> NICHT als fertige libgraphics4d_rp2350.a, sondern nur als Header +"
    echo ">>> geschlossene Bibliothek im Workshop5/Pico-Build auf Windows."
    echo ">>> Auf Windows nach dem Kompilieren eines Workshop5-Projekts suchen:"
    echo ">>>   powershell -File scripts/find-graphics4d-lib.ps1"
    echo ">>> oder die .a aus dem Projekt-build/ nach vendor/Graphics4D-pico/lib/ kopieren."
else
    echo
    echo ">>> Quellen vorhanden — statische Lib bauen:"
    echo ">>>   export PICO_SDK_PATH=~/pico/pico-sdk"
    echo ">>>   ./scripts/build-graphics4d-lib.sh"
fi
