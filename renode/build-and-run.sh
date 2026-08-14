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

cd "$(dirname "$0")/.."

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

echo "==> Building mpu-exerciser (MPU verification test firmware)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p mpu-exerciser

echo "==> Building adc-exerciser (ADC bring-up test firmware)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p adc-exerciser

echo "==> Building usb-init-exerciser (USB core-init test firmware)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p usb-init-exerciser

echo "==> Building usb-enum-exerciser (USB enumeration test firmware)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p usb-enum-exerciser

echo "==> Building usb-iso-exerciser (USB isochronous-loopback test firmware)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p usb-iso-exerciser

echo "==> Building dsp-bench (daisy-dsp cycle-bench test firmware)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p dsp-bench

echo "==> Building cache-coherency-exerciser (D-cache/DMA coherency test firmware)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p cache-coherency-exerciser

echo "==> Building fft-shootout (competing-FFT cycle-bench firmware)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p fft-shootout

echo "==> Building daisy-usb-audio (renode_test — skips USB sim can't model)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p daisy-usb-audio \
    --features renode_test \
    --target-dir target/renode

echo "==> Building daisy-usb-audio CODEC build (renode_test,codec — boot-to-main smoke)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p daisy-usb-audio \
    --features renode_test,codec \
    --target-dir target/renode-codec

echo "==> Building gap-exerciser (verifies the RAM-gap guards)…"
cargo build \
    --release \
    --target thumbv7em-none-eabihf \
    -p gap-exerciser

echo "==> Running Renode test suite…"
# Use the source-built Renode (with our QSPI-XIP fidelity patches) and the
# robotframework venv. Override RENODE_ROOT / RENODE_TEST to point elsewhere.
RENODE_ROOT="${RENODE_ROOT:-$HOME/projects/renode}"
export PATH="$RENODE_ROOT/.venv/bin:$HOME/.local/bin:$PATH"
"${RENODE_TEST:-$RENODE_ROOT/renode-test}" \
    renode/app_standalone.robot \
    renode/boot_blink.robot \
    renode/bootloader_jump.robot \
    renode/qspi_abort.robot \
    renode/qspi_mode_bits_missing.robot \
    renode/qspi_dummy_cycle_mismatch.robot \
    renode/qspi_continuous_persist.robot \
    renode/qspi_ncs_pinmux.robot \
    renode/bootloader_warm_reboot.robot \
    renode/bootloader_dfu_fallback.robot \
    renode/flash_protocol.robot \
    renode/sdram_region.robot \
    renode/sdram_fmc.robot \
    renode/rcc_clock.robot \
    renode/clocks_boot.robot \
    renode/dwt_clocked.robot \
    renode/pwr.robot \
    renode/adc_bringup.robot \
    renode/adc_firmware.robot \
    renode/otg_core.robot \
    renode/otg_device.robot \
    renode/otg_fifo.robot \
    renode/otg_firmware.robot \
    renode/otg_enum.robot \
    renode/otg_iso.robot \
    renode/otg_vbus.robot \
    renode/fault_exerciser.robot \
    renode/dsp_bench.robot \
    renode/mpu.robot \
    renode/sai_dma.robot \
    renode/usb_audio_xip.robot \
    renode/usb_audio_codec_boot.robot \
    renode/gap_guard.robot \
    renode/cache_coherency.robot \
    renode/fft_shootout.robot \
    "$@"
