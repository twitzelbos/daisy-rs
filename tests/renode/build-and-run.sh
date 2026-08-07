#!/usr/bin/env bash
# Build the daisy-rs firmware for Renode and run the Renode test suite.
#
# Two distinct builds are needed:
#   1. daisy-boot with `--features renode_test` — skips USB init so it can
#      simulate under Renode (Renode does not model Synopsys DWC_OTG). Goes
#      to `target/renode/…/release/daisy-boot`.
#   2. daisy-app-template — the QSPI-XIP application, standard build. Goes
#      to `target/…/release/daisy-app-template`.
#
# The default hardware build in `target/…/release/daisy-boot` (without
# `--features renode_test`) is left untouched, so `daisy flash --bootloader`
# still produces a hardware-correct binary.
set -euo pipefail

cd "$(dirname "$0")/../.."

echo "==> Building daisy-boot for Renode (renode_test feature)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p daisy-boot \
    --features renode_test \
    --target-dir target/renode

echo "==> Building daisy-app-template…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p daisy-app-template

echo "==> Building fault-exerciser (exception-vector test firmware)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p fault-exerciser

echo "==> Building daisy-usb-audio (renode_test — skips USB sim can't model)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p daisy-usb-audio \
    --features renode_test \
    --target-dir target/renode

echo "==> Running Renode test suite…"
# Use the source-built Renode (with our QSPI-XIP fidelity patches) and the
# robotframework venv. Override RENODE_ROOT / RENODE_TEST to point elsewhere.
RENODE_ROOT="${RENODE_ROOT:-$HOME/projects/renode}"
export PATH="$RENODE_ROOT/.venv/bin:$HOME/.local/bin:$PATH"
"${RENODE_TEST:-$RENODE_ROOT/renode-test}" \
    tests/renode/app_standalone.robot \
    tests/renode/boot_blink.robot \
    tests/renode/bootloader_jump.robot \
    tests/renode/qspi_abort.robot \
    tests/renode/qspi_mode_bits_missing.robot \
    tests/renode/qspi_dummy_cycle_mismatch.robot \
    tests/renode/qspi_continuous_persist.robot \
    tests/renode/flash_protocol.robot \
    tests/renode/sdram_region.robot \
    tests/renode/sdram_fmc.robot \
    tests/renode/rcc_clock.robot \
    tests/renode/clocks_boot.robot \
    tests/renode/dwt_clocked.robot \
    tests/renode/pwr.robot \
    tests/renode/fault_exerciser.robot \
    tests/renode/sai_dma.robot \
    tests/renode/usb_audio_xip.robot \
    "$@"
