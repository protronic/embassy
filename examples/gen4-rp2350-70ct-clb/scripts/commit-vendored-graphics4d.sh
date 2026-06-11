#!/usr/bin/env bash
# Commit and push vendor/graphics4d-rp2350/ after building locally.
#
# Run on YOUR machine (where Graphics4D-pico sources and Pico SDK live):
#
#   export PICO_SDK_PATH=/usr/share/pico-sdk
#   export GEN4_GRAPHICS4D_SDK=./vendor/Graphics4D-pico
#   ./scripts/vendor-graphics4d-into-repo.sh
#   ./scripts/commit-vendored-graphics4d.sh
#
# Or if you already copied libgraphics4d_rp2350.a + headers into vendor/graphics4d-rp2350/:
#
#   ./scripts/commit-vendored-graphics4d.sh
#
# Optional: COMMIT_MSG=... PUSH=0 ./scripts/commit-vendored-graphics4d.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(git -C "${CRATE_DIR}" rev-parse --show-toplevel)"
VENDOR_REL="examples/gen4-rp2350-70ct-clb/vendor/graphics4d-rp2350"
DEST="${CRATE_DIR}/vendor/graphics4d-rp2350"
LIB="${DEST}/lib/libgraphics4d_rp2350.a"
HDR="${DEST}/include/Graphics4D.h"
PUSH="${PUSH:-1}"
TARGET_BRANCH="${TARGET_BRANCH:-}"
COMMIT_MSG="${COMMIT_MSG:-vendor(gen4): add prebuilt libgraphics4d_rp2350.a}"

die() { echo "error: $*" >&2; exit 1; }

[[ -f "${LIB}" ]] || die "${LIB} missing — run ./scripts/vendor-graphics4d-into-repo.sh first"
[[ -f "${HDR}" ]] || die "${HDR} missing — vendor script should copy Graphics4D.h"
bash "${SCRIPT_DIR}/validate-graphics4d-lib.sh" "${LIB}"

echo "=== vendored Graphics4D ==="
ls -lh "${LIB}"
echo "headers:"
find "${DEST}/include" -name '*.h' 2>/dev/null | head -20
echo

if git -C "${REPO_ROOT}" ls-files --error-unmatch "${VENDOR_REL}/lib/libgraphics4d_rp2350.a" &>/dev/null \
    && git -C "${REPO_ROOT}" diff --quiet -- "${VENDOR_REL}" \
    && git -C "${REPO_ROOT}" diff --cached --quiet -- "${VENDOR_REL}"; then
    echo "${VENDOR_REL} already committed and clean — nothing to do"
    exit 0
fi

git -C "${REPO_ROOT}" add "${VENDOR_REL}/"
echo "=== staged ==="
git -C "${REPO_ROOT}" diff --cached --stat -- "${VENDOR_REL}/"

if git -C "${REPO_ROOT}" diff --cached --quiet -- "${VENDOR_REL}/"; then
    die "nothing staged under ${VENDOR_REL}/ — is the .a already committed?"
fi

git -C "${REPO_ROOT}" commit -m "${COMMIT_MSG}"

if [[ "${PUSH}" == "1" ]]; then
    branch="${TARGET_BRANCH:-$(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD)}"
    echo "=== pushing ${branch} ==="
    git -C "${REPO_ROOT}" push -u origin "${branch}"
fi

echo
echo "Done. Verify:"
echo "  ./scripts/check-graphics4d.sh"
echo "  cargo build --bin oxivgl_widget_demo --features oxivgl,touch"
