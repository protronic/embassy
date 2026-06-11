#!/usr/bin/env bash
# Operate the Graphics4D build worker from a cloud agent, laptop, or CI.
#
# Modes:
#   local   Build on THIS machine (cloud VM or dev box), optional commit+push
#   remote  Trigger GitHub Actions workflow (needs workflow on default branch)
#   status  Show last workflow run
#   fetch   Download latest artifact into vendor/graphics4d-rp2350/
#
# Cloud agent (Cursor): set GRAPHICS4D_PICO_TOKEN in environment, then:
#   ./scripts/operate-graphics4d-worker.sh local --push
#
# Usage:
#   GRAPHICS4D_PICO_TOKEN=ghp_... ./scripts/operate-graphics4d-worker.sh local
#   ./scripts/operate-graphics4d-worker.sh local --push --branch cursor/gen4-rp2350-70ct-clb-36d2
#   ./scripts/operate-graphics4d-worker.sh remote --runner cloud --wait
#   ./scripts/operate-graphics4d-worker.sh fetch

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRATE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(git -C "${CRATE_DIR}" rev-parse --show-toplevel)"
REPO="${GITHUB_REPOSITORY:-protronic/embassy}"
WORKFLOW_NAME="Vendor Graphics4D lib"
DEFAULT_BRANCH="${DEFAULT_BRANCH:-cursor/gen4-rp2350-70ct-clb-36d2}"

die() { echo "error: $*" >&2; exit 1; }

usage() {
    cat <<EOF
Usage: $0 <mode> [options]

Modes:
  local    Build on this machine (default for cloud agents)
  remote   Trigger GitHub Actions workflow
  status   Print last workflow run
  fetch    Download latest successful artifact into vendor tree

Options (local):
  --push              Commit and push vendored lib
  --branch BRANCH     Target branch (default: ${DEFAULT_BRANCH})
  --no-cargo-check    Skip cargo check after build

Options (remote):
  --runner cloud|self-hosted   (default: cloud)
  --branch BRANCH     Branch to update (default: ${DEFAULT_BRANCH})
  --push              Push commit from workflow (default: true)
  --wait              Poll until workflow completes
  --ref REF           Workflow git ref (default: main)

Environment:
  GRAPHICS4D_PICO_TOKEN   PAT to clone private Graphics4D-pico (required for cloud/local)
  GEN4_GRAPHICS4D_SDK     Use existing source tree instead of cloning
  PICO_SDK_PATH           Pico SDK path (auto-detected or installed)
EOF
}

mode_local() {
    local push=0 branch="${DEFAULT_BRANCH}" skip_cargo=0
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --push) push=1 ;;
            --branch) branch="$2"; shift ;;
            --no-cargo-check) skip_cargo=1 ;;
            *) die "unknown option: $1" ;;
        esac
        shift
    done

    if [[ -z "${GRAPHICS4D_PICO_TOKEN:-}" && ! -f "${GEN4_GRAPHICS4D_SDK:-${CRATE_DIR}/vendor/Graphics4D-pico}/src/Graphics4D.h" ]]; then
        die "set GRAPHICS4D_PICO_TOKEN or GEN4_GRAPHICS4D_SDK with Graphics4D-pico sources"
    fi

    echo "=== local worker (this machine) ==="
    # shellcheck source=/dev/null
    source "${SCRIPT_DIR}/ci-install-graphics4d-deps.sh"
    export PICO_SDK_PATH

    git -C "${REPO_ROOT}" checkout "${branch}" 2>/dev/null || true
    git -C "${REPO_ROOT}" pull --ff-only origin "${branch}" 2>/dev/null || true

    SKIP_CARGO_CHECK="${skip_cargo}" PUSH="${push}" TARGET_BRANCH="${branch}" \
        bash "${SCRIPT_DIR}/graphics4d-worker-build.sh"
}

mode_remote() {
    local runner=cloud branch="${DEFAULT_BRANCH}" push=1 wait=0 ref=main
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --runner) runner="$2"; shift ;;
            --branch) branch="$2"; shift ;;
            --push) push=1 ;;
            --no-push) push=0 ;;
            --wait) wait=1 ;;
            --ref) ref="$2"; shift ;;
            *) die "unknown option: $1" ;;
        esac
        shift
    done

    command -v gh >/dev/null || die "gh CLI required for remote mode"

    echo "=== trigger GitHub Actions (${runner}) ==="
    if ! gh workflow run "${WORKFLOW_NAME}" -R "${REPO}" --ref "${ref}" \
        -f target_branch="${branch}" \
        -f push_commit="${push}" \
        -f runner_type="${runner}" 2>&1; then
        die "workflow trigger failed — is '${WORKFLOW_NAME}' on branch '${ref}' (usually main)?"
    fi

    sleep 3
    mode_status

    if [[ "${wait}" -eq 1 ]]; then
        echo "=== waiting for workflow ==="
        gh run watch -R "${REPO}" --exit-status
    fi
}

mode_status() {
    command -v gh >/dev/null || die "gh CLI required"
    gh run list -R "${REPO}" --workflow "${WORKFLOW_NAME}" -L 3 2>/dev/null \
        || gh run list -R "${REPO}" -L 5
}

mode_fetch() {
    command -v gh >/dev/null || die "gh CLI required"
    local run_id dir
    run_id="$(gh run list -R "${REPO}" --workflow "${WORKFLOW_NAME}" -L 1 --json databaseId -q '.[0].databaseId' 2>/dev/null || true)"
    [[ -n "${run_id}" ]] || die "no workflow runs found"

    dir="$(mktemp -d)"
    echo "Downloading artifact from run ${run_id} ..."
    gh run download "${run_id}" -R "${REPO}" -D "${dir}"

    local artifact
    artifact="$(find "${dir}" -name 'libgraphics4d_rp2350.a' | head -1)"
    [[ -f "${artifact}" ]] || die "artifact libgraphics4d_rp2350.a not found"

    local dest="${CRATE_DIR}/vendor/graphics4d-rp2350"
    mkdir -p "${dest}/lib"
    cp -f "${artifact}" "${dest}/lib/libgraphics4d_rp2350.a"
    echo "Installed ${dest}/lib/libgraphics4d_rp2350.a"
    ls -lh "${dest}/lib/libgraphics4d_rp2350.a"
}

[[ $# -ge 1 ]] || { usage; exit 1; }
case "$1" in
    local) shift; mode_local "$@" ;;
    remote) shift; mode_remote "$@" ;;
    status) shift; mode_status "$@" ;;
    fetch) shift; mode_fetch "$@" ;;
    -h|--help) usage ;;
    *) die "unknown mode: $1 (try: local, remote, status, fetch)" ;;
esac
