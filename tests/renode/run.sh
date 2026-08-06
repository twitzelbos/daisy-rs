#!/usr/bin/env bash
# Run the daisy-rs Renode test suite locally.
#
# Assumes `renode` is on PATH (either from the .dmg install or Docker
# wrapper). Builds the firmware in release first so the ELF is fresh.

set -euo pipefail

cd "$(dirname "$0")/../.."

echo "==> Building firmware for thumbv7em-none-eabihf (release)..."
cargo build --release --target thumbv7em-none-eabihf -p daisy-boot -p daisy-app-template

echo "==> Running Renode Robot tests..."
# renode-test is the CLI shipped with Renode that wraps Robot Framework
# with the right library imports.
renode-test tests/renode/bootloader.robot "$@"
