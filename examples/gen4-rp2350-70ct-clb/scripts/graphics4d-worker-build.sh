#!/usr/bin/env bash
# Build + vendor libgraphics4d_rp2350.a — for local use or a self-hosted CI worker.
#
# Environment (worker machine or GitHub Actions):
#   PICO_SDK_PATH          Pico SDK 2.x (e.g. /usr/share/pico-sdk)
#   GEN4_GRAPHICS4D_SDK    Existing Graphics4D-pico tree (preferred on worker)
#   GRAPHICS4D_PICO_TOKEN  Optional PAT to clone protronic/Graphics4D-pico
#   GRAPHICS4D_PICO_URL    Override clone URL
#   PUSH                   1 = git commit + push vendored tree (default 0 in CI)
#   TARGET_BRANCH          Branch to push (default: current)
#
# Usage:
#   ./scripts/graphics4d-worker-build.sh
#   PUSH=1 TARGET_BRANCH=cursor/gen4-rp2350-70ct-clb-36d2 ./scripts/graphics4d-worker-build.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(git -C "${CRATE_DIR}" rev-parse --show-toplevel)"
VENDOR_REL="examples/gen4-rp2350-70ct-clb/vendor/graphics4d-rp2350"
SDK="${GEN4_GRAPHICS4D_SDK:-${CRATE_DIR}/vendor/Graphics4D-pico}"
PUSH="${PUSH:-0}"
TARGET_BRANCH="${TARGET_BRANCH:-$(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD)}"
GRAPHICS4D_PICO_URL="${GRAPHICS4D_PICO_URL:-https://github.com/protronic/Graphics4D-pico.git}"

die() { echo "error: $*" >&2; exit 1; }

echo "=== Graphics4D worker build ==="
echo "repo:      ${REPO_ROOT}"
echo "crate:     ${CRATE_DIR}"
echo "branch:    ${TARGET_BRANCH}"
echo "push:      ${PUSH}"
echo

ensure_graphics4d_sources() {
    if [[ -f "${SDK}/src/Graphics4D.h" || -f "${SDK}/include/Graphics4D.h" ]]; then
        echo "Graphics4D sources: ${SDK}"
        return 0
    fi

    if [[ -n "${GRAPHICS4D_PICO_TOKEN:-}" ]]; then
        echo "Cloning Graphics4D-pico with token ..."
        rm -rf "${SDK}"
        mkdir -p "$(dirname "${SDK}")"
        git clone --depth 1 \
            "https://x-access-token:${GRAPHICS4D_PICO_TOKEN}@${GRAPHICS4D_PICO_URL#https://}" \
            "${SDK}"
    elif [[ -d "${SDK}/.git" ]]; then
        echo "Updating existing clone ${SDK} ..."
        git -C "${SDK}" pull --ff-only
    else
        die "Graphics4D sources missing at ${SDK} — set GEN4_GRAPHICS4D_SDK or GRAPHICS4D_PICO_TOKEN"
    fi

    [[ -f "${SDK}/src/Graphics4D.h" || -f "${SDK}/include/Graphics4D.h" ]] \
        || die "Graphics4D.h not found after clone"
}

detect_pico_sdk() {
    if [[ -n "${PICO_SDK_PATH:-}" && -f "${PICO_SDK_PATH}/external/pico_sdk_import.cmake" ]]; then
        return 0
    fi
    for candidate in /usr/share/pico-sdk /usr/lib/pico-sdk "${HOME}/pico/pico-sdk" "${HOME}/pico-sdk"; do
        if [[ -f "${candidate}/external/pico_sdk_import.cmake" ]]; then
            export PICO_SDK_PATH="${candidate}"
            echo "PICO_SDK_PATH=${PICO_SDK_PATH}"
            return 0
        fi
    done
    die "PICO_SDK_PATH not set and pico-sdk not found (Arch: sudo pacman -S pico-sdk)"
}

ensure_graphics4d_sources
detect_pico_sdk
export GEN4_GRAPHICS4D_SDK="${SDK}"

echo
echo "=== vendor into ${VENDOR_REL} ==="
bash "${SCRIPT_DIR}/vendor-graphics4d-into-repo.sh"

echo
echo "=== verify ==="
bash "${SCRIPT_DIR}/check-graphics4d.sh"

echo
echo "=== cargo check (Graphics4D linked) ==="
cd "${CRATE_DIR}"
cargo check --bin oxivgl_widget_demo --features oxivgl,touch

if [[ "${PUSH}" == "1" ]]; then
    echo
    echo "=== commit + push ==="
    git -C "${REPO_ROOT}" checkout "${TARGET_BRANCH}"
    git -C "${REPO_ROOT}" pull --ff-only origin "${TARGET_BRANCH}" || true
    PUSH=1 TARGET_BRANCH="${TARGET_BRANCH}" \
        COMMIT_MSG="vendor(gen4): prebuilt libgraphics4d_rp2350.a (worker)" \
        bash "${SCRIPT_DIR}/commit-vendored-graphics4d.sh"
fi

echo
echo "Worker build complete."
ls -lh "${CRATE_DIR}/vendor/graphics4d-rp2350/lib/libgraphics4d_rp2350.a"
