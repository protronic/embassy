#!/usr/bin/env bash
# Helper: print steps to register a self-hosted GitHub Actions runner for Graphics4D builds.
#
# Usage:
#   ./scripts/setup-graphics4d-runner.sh
#   ./scripts/setup-graphics4d-runner.sh https://github.com/protronic/embassy

set -euo pipefail

REPO_URL="${1:-https://github.com/protronic/embassy}"
RUNNER_LABEL="${RUNNER_LABEL:-graphics4d}"

cat <<EOF
Graphics4D self-hosted worker setup
===================================

1) Install build deps (Arch / CachyOS):

   sudo pacman -S --needed cmake ninja arm-none-eabi-gcc pico-sdk git rustup
   rustup default stable
   rustup target add thumbv8m.main-none-eabihf

2) Graphics4D-pico sources — pick ONE:

   a) Fixed path on this machine (recommended):
      git clone git@github.com:protronic/Graphics4D-pico.git ~/Graphics4D-pico
      # In GitHub repo Settings → Actions → Variables (repository):
      #   GEN4_GRAPHICS4D_SDK = /home/YOU/Graphics4D-pico
      #   PICO_SDK_PATH       = /usr/share/pico-sdk

   b) Clone via PAT in CI only:
      # Settings → Secrets → GRAPHICS4D_PICO_TOKEN (read access to Graphics4D-pico)

3) Register self-hosted runner (label: ${RUNNER_LABEL}):

   mkdir -p ~/actions-runner && cd ~/actions-runner
   # Download runner from: ${REPO_URL}/settings/actions/runners/new
   # Then:
   ./config.sh --url ${REPO_URL} --token <TOKEN_FROM_GITHUB> --labels ${RUNNER_LABEL}
   sudo ./svc.sh install
   sudo ./svc.sh start

4) Trigger build:

   GitHub → Actions → "Vendor Graphics4D lib" → Run workflow
   Branch: cursor/gen4-rp2350-70ct-clb-36d2
   push_commit: true

5) Local dry-run (same script as the worker):

   cd examples/gen4-rp2350-70ct-clb
   export PICO_SDK_PATH=/usr/share/pico-sdk
   export GEN4_GRAPHICS4D_SDK=~/Graphics4D-pico
   ./scripts/graphics4d-worker-build.sh

   # Or commit manually:
   PUSH=1 TARGET_BRANCH=cursor/gen4-rp2350-70ct-clb-36d2 ./scripts/graphics4d-worker-build.sh

EOF
