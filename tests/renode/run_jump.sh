#!/usr/bin/env bash
# Diagnostic Renode run for the "bootloader jumps but app doesn't run" bug.
# Builds both binaries in release, then runs bootloader_jump.robot which
# preloads the app into QSPI and traces the jump.

set -euo pipefail

cd "$(dirname "$0")/../.."

echo "==> Building bootloader + app template (release)..."
cargo build --release --target thumbv7em-none-eabihf -p daisy-boot -p daisy-app-template

echo "==> Running Renode Robot jump-trace test..."
renode-test tests/renode/bootloader_jump.robot "$@"
