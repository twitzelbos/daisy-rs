# daisy-rs

An all-Rust firmware environment for the STM32H750 **Daisy Seed**: a bootloader
in internal flash, applications in QSPI executed in place (XIP), a `daisy` host
CLI for building/flashing, and a Renode simulation for CI.

## Memory layout

| Region         | Address       | Contents                                  |
| -------------- | ------------- | ----------------------------------------- |
| Internal flash | `0x0800_0000` | bootloader (`daisy-boot`)                 |
| QSPI flash     | `0x9000_0000` | application, executed in place (XIP)      |

The bootloader configures the clock tree (PLL1 = 400 MHz sysclk, PLL2 → FMC/SDRAM,
PLL3 → SAI), brings QSPI up in memory-mapped mode, validates the app's vector
table, and jumps to it.

## Board revisions

Every STM32H750 Daisy Seed is pin-compatible; the one difference that reaches
firmware is the **audio codec**, selected by a cargo feature on codec-using apps:

| Board      | Codec                          | Feature            |
| ---------- | ------------------------------ | ------------------ |
| Seed 1.1   | Wolfson WM8731 (I²C)           | *default* (none)   |
| **Seed 3** | **TI TAC5242** (HW-strapped)   | **`seed3`**        |

The Seed 3's TAC5242 has no I²C/register interface — it's configured by on-board
straps and clocked entirely from the SAI, so the `seed3` feature simply selects
the correct SAI setup (block A = TX master / B = RX slave, 32-bit, MCLK on PE2).

## Applications

Flashable firmware. The bootloader lives in internal flash; the rest are XIP
apps loaded to QSPI with `daisy flash`.

| Crate                | What it is                                                                                                                                                    |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `daisy-boot`         | The **bootloader** (internal flash). Configures clocks + QSPI memory-mapped mode, validates the app's vector table, and jumps to XIP. No MPU/caches.           |
| `daisy-app-template` | A minimal **XIP app to start from**: `#[pre_init]` MPU + L1 caches, LED heartbeat, DWT-based delays. Copy it for new applications.                              |
| `daisy-hothouse`     | For a Daisy in a Cleveland Music Co. **Hothouse pedal**: a live control panel (6 pots, 3 toggles, 2 footswitches) rendered as a ratatui TUI over USB-CDC serial. |
| `daisy-usb-audio`    | A **USB composite device** — CDC-ACM serial + UAC1 stereo-48 kHz audio (in *and* out) + USB-MIDI — bridged to the on-board codec. The Seed-3 soundcard build.  |
| `daisy-usb-audio --features fx_loop` | **fx-loop** *(planned)* — the soundcard as a hardware-insert: patch any guitar pedal between the Hothouse OUT/IN jacks and it becomes a real-time insert effect on a Mac track, with footswitch true-bypass + send/return trim + wet/dry on the Hothouse controls. Design: [`docs/fx-loop-app.md`](docs/fx-loop-app.md). |

## Supporting crates

| Crate            | Role                                                                          |
| ---------------- | ----------------------------------------------------------------------------- |
| `daisy-bsp`      | Board support: clocks, pins, and peripherals for the Seed + carrier boards.   |
| `daisy-audio`    | SAI + codec audio driver (WM8731 / TAC5242) with a block-based callback API.  |
| `daisy-dsp`      | Pure-Rust DSP primitives — oscillators, filters, envelopes, effects.          |
| `ratatui-serial` | A `no_std` ratatui backend that renders to a byte sink (USB-CDC) via ANSI.    |
| `daisy-cli`      | The host `daisy` tool — scaffold, build, flash (DFU), stream RTT/serial.       |

Six **Renode test-firmware** crates (`*-exerciser`) drive specific hardware
paths — faults, the MPU, the ADC, and the USB OTG core / enumeration /
isochronous paths — so the Renode peripheral models can be verified against the
real HAL and the reference manuals. They are not meant to be flashed.

## Prerequisites

```sh
rustup target add thumbv7em-none-eabihf
```

The workspace `.cargo/config.toml` defaults the build target to
`thumbv7em-none-eabihf`, so firmware builds "just work" with `cargo build`. The
**host** `daisy` CLI must be built for the host target:

```sh
# install it (recommended):
cargo install --path crates/daisy-cli --target "$(rustc -vV | awk '/^host:/{print $2}')"

# …or run it ad-hoc without installing:
cargo run -p daisy-cli --target "$(rustc -vV | awk '/^host:/{print $2}')" -- <SUBCOMMAND>
```

DFU flashing goes through `nusb` — no external `dfu-util` is required.

## Building

`daisy build` (and `daisy flash`) accept `--features`; use them together with
`-p`, since a feature applies per crate.

### Bootloader → internal flash

```sh
daisy build -p daisy-boot
# equivalently: cargo build -p daisy-boot --release
```

The bootloader is currently one binary for all Seed revisions. (Board-specific
bootloaders can be added later — e.g. to route the SAI kernel-clock mux for a
particular board.)

### Applications → QSPI (XIP)

Apps link to `0x9000_0000`. For a **Seed 3** app that uses the codec, enable the
`seed3` feature:

```sh
# USB soundcard for the Seed 3 (TAC5242):
daisy build -p daisy-usb-audio --features seed3

# generic, codec-agnostic app template:
daisy build -p daisy-app-template

# the same app on a Seed 1.1 (WM8731) — just omit the feature:
daisy build -p daisy-usb-audio
```

## Flashing

Check what's connected first:

```sh
daisy list
```

### Bootloader (internal flash, `0x0800_0000`)

The bootloader lives in internal flash, so it's written through the **STM32 ROM
DFU** (system bootloader), *not* the running daisy-boot. Put the Seed into ROM
DFU mode — **hold BOOT, tap RESET, release BOOT** — then:

```sh
daisy flash --bootloader                       # builds + flashes daisy-boot
# or a prebuilt ELF:
daisy flash --bootloader --elf target/thumbv7em-none-eabihf/release/daisy-boot
```

### Application (QSPI, `0x9000_0000`)

With the bootloader already programmed, the Daisy exposes DFU for an **~8-second
window after reset** (it advertises ST's DFU VID:PID, so `daisy flash` finds it).
**Tap RESET**, then within the window:

```sh
# Seed 3 soundcard — build with the feature and flash:
daisy flash -p daisy-usb-audio --features seed3

# generic template (the default package):
daisy flash

# or flash a prebuilt ELF (target address comes from the ELF):
daisy flash --elf target/thumbv7em-none-eabihf/release/daisy-usb-audio
```

`daisy flash` writes to QSPI by default and to internal flash with
`--bootloader`, and sanity-checks the ELF's linked address against the target.

### End-to-end: a Seed 3 USB soundcard

```sh
# 1. one-time — flash the bootloader (ROM DFU: hold BOOT, tap RESET, release BOOT):
daisy flash --bootloader

# 2. flash the soundcard app (tap RESET, then within ~8 s):
daisy flash -p daisy-usb-audio --features seed3
```

The host then enumerates a composite USB device — a stereo 48 kHz audio
interface (soundcard) + CDC serial + USB-MIDI. Analog audio is on the Seed's
**Audio In/Out L/R** header pins.

## Testing

```sh
# host-side logic tests (pure-logic modules compiled for the host):
cargo test -p daisy-bsp --target "$(rustc -vV | awk '/^host:/{print $2}')"

# Renode simulation suite (builds firmware + runs the robot tests):
./renode/build-and-run.sh
```

See `renode/` for the simulation models and scenarios.
