# daisy-rs task runner — every build/flash/test recipe in one place.
# Requires `just` (`cargo install just`). Run `just` (or `just --list`) to see all.
#
# ⚠️  THE LINKER TRAP (why these recipes exist): NEVER set `RUSTFLAGS` for a
#     *firmware* (thumbv7em) build. The env var REPLACES — does not merge with —
#     .cargo/config.toml's rustflags, dropping `-C link-arg=-Tlink.x`, so the
#     linker produces a broken ELF (entry 0x0000_0000, sections at 0x10000) that
#     Renode/probe-rs reject. Every firmware recipe below `unset RUSTFLAGS`
#     first. Only clippy uses `-D warnings` — it type-checks without linking, so
#     it's safe there.

set shell := ["bash", "-c"]

host := `rustc -vV | sed -n 's/^host: //p'`
target := "thumbv7em-none-eabihf"
app := "daisy-usb-audio" # default package for build/flash recipes

# The daisy CLI, built + run for the host (no install required).
daisy := "cargo run -q -p daisy-cli --target " + host + " --"

# Show all recipes.
default:
    @just --list

# ─── firmware builds (plain cargo, RUSTFLAGS cleared) ───────────────────────

# Bootloader → internal flash @ 0x0800_0000.
build-boot:
    unset RUSTFLAGS; cargo build --release --target {{ target }} -p daisy-boot

# Build an XIP app → QSPI @ 0x9000_0000.  e.g. `just build-app daisy-hothouse [--features seed3]`
build-app pkg=app *features:
    unset RUSTFLAGS; cargo build --release --target {{ target }} -p {{ pkg }} {{ features }}

# Bootloader + all shipping apps (what CI's firmware job effectively covers).
build-all: build-boot
    unset RUSTFLAGS; cargo build --release --target {{ target }} \
        -p daisy-app-template -p daisy-usb-audio -p daisy-hothouse

# ─── flashing (each builds first, then DFUs) ────────────────────────────────

# Which Daisy(s) are connected?
list:
    {{ daisy }} list

# Flash the bootloader via STM32 ROM DFU (hold BOOT, tap RESET, release BOOT).
flash-boot:
    unset RUSTFLAGS; {{ daisy }} flash --bootloader

# Build + flash an app to QSPI; tap RESET first.  e.g. `just flash daisy-hothouse [--features seed3]`
flash pkg=app *features:
    unset RUSTFLAGS; {{ daisy }} flash -p {{ pkg }} {{ features }}

# One-shot Seed-3 USB soundcard: bootloader (ROM DFU), then the seed3 app (tap RESET).
flash-soundcard: flash-boot
    unset RUSTFLAGS; {{ daisy }} flash -p daisy-usb-audio --features seed3

# ─── the host CLI ───────────────────────────────────────────────────────────

# Run the daisy CLI on the host.  e.g.  `just cli list`
cli *args:
    {{ daisy }} {{ args }}

# Install `daisy` onto PATH (optional; the recipes above don't need it).
install-cli:
    cargo install --path crates/daisy-cli --target {{ host }}

# ─── lint & test (mirrors CI) ───────────────────────────────────────────────

# Host tests for every host-testable crate — exactly what CI runs (incl. doctests).
test:
    cargo test -p daisy-bsp -p daisy-cli -p ratatui-serial -p daisy-dsp -p daisy-midi --target {{ host }}

# Just the DSP crate's tests (unit + doctests).
test-dsp:
    cargo test -p daisy-dsp --target {{ host }}

fmt:
    cargo fmt --all

fmt-check:
    cargo fmt --all -- --check

# Clippy as CI runs it — dev + release, `-D warnings` (safe: clippy doesn't link firmware).
clippy:
    RUSTFLAGS="-D warnings" cargo clippy --workspace \
        --exclude daisy-cli --exclude daisy-dsp-testkit --exclude pad-audition --profile dev
    RUSTFLAGS="-D warnings" cargo clippy --workspace \
        --exclude daisy-cli --exclude daisy-dsp-testkit --exclude pad-audition --profile release

# Reproduce the full CI gate set locally before pushing.
ci: fmt-check clippy test build-all

# ─── simulation & docs ──────────────────────────────────────────────────────

# Build the test firmware + run the Renode robot suite.
renode:
    ./renode/build-and-run.sh

# Build & open the daisy-dsp API docs in a browser.
doc:
    cargo doc -p daisy-dsp --no-deps --target {{ host }} --open

clean:
    cargo clean
