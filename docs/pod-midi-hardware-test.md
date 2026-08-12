# Pod USB-MIDI — hardware test with an Arturia MiniLab 3

Everything in the MIDI path is unit-tested (`daisy-midi`) or compile-checked
(`pod-midi`, the `pod` feature), but the UART → USB link can only be validated on
a real board. This is a quick bench procedure that closes that gap using an
**Arturia MiniLab 3** as the MIDI source and a **Daisy Pod** running `pod-midi`.

## What it proves

- USART1 RX bring-up (PB7 / Seed D14, 31250 baud 8N1) with real `CoreClocks`.
- The serial → USB-MIDI packetizer on live traffic (Note On/Off with running
  status, Control Change, Pitch Bend, Program Change).
- The USB-MIDI class enumerating **driverless** and its bulk IN carrying events.

## Equipment

- **Daisy Pod** (Seed seated in the Pod host board).
- **Arturia MiniLab 3** (any controller with a TRS/DIN MIDI OUT works).
- A **3.5 mm TRS MIDI cable** (MiniLab OUT ↔ Pod IN) — see the cabling note.
- A **USB charger** to power the MiniLab, plus a **data** USB cable + computer.
- A flashing path for the Pod (probe-rs, or ROM DFU / `dfu-util`).

## Signal path

```
 MiniLab 3 ── TRS MIDI OUT ─────────▶ Pod MIDI-IN (TRS → PB7, USART1)
 MiniLab 3 ── USB ──▶ charger (power only)          │ pod-midi
 computer  ◀── USB ── Pod                           ▼  enumerates as
                                          "Daisy Pod MIDI" input port
```

**Power the MiniLab from a charger, not the test computer.** Then the computer's
MIDI monitor sees the **Pod's** port — proving the TRS → USB path — instead of
the MiniLab's own built-in USB-MIDI port.

## Cabling note (check this first)

TRS MIDI exists in **Type A** and **Type B** wirings. The MiniLab 3 is **Type A**
(the MMA standard). Confirm the Pod's MIDI-IN is also Type A (expected) — if the
two ends differ, a straight cable passes nothing and you need a Type-A↔B adapter.
If either jack is 5-pin DIN, use a TRS(A)↔DIN adapter.

## Flash `pod-midi`

`pod-midi` is a standalone image (internal flash, its own `clocks::init`) — the
simplest target for a plain Pod:

```sh
cargo build --release --target thumbv7em-none-eabihf -p pod-midi

# probe-rs (ST-Link / debug probe):
probe-rs download --chip STM32H750IB --binary-format elf \
  target/thumbv7em-none-eabihf/release/pod-midi
probe-rs reset --chip STM32H750IB

# …or ROM DFU (hold BOOT, tap RESET, release BOOT), via dfu-util:
arm-none-eabi-objcopy -O binary \
  target/thumbv7em-none-eabihf/release/pod-midi pod-midi.bin
dfu-util -a 0 -s 0x08000000:leave -D pod-midi.bin
```

> `pod-midi` runs from `0x0800_0000`, so it **replaces the bootloader** as the
> internal-flash image. Re-flash the bootloader to go back to XIP apps.
>
> To test the *composite* app's `pod` feature instead (soundcard + MIDI in one
> device), that's the XIP `daisy-usb-audio --features pod` build loaded through
> the bootloader — but it assumes the Seed audio codec, so `pod-midi` is the
> cleaner check of just the MIDI path.

## Procedure

1. Flash `pod-midi`; connect MiniLab TRS OUT → Pod MIDI-IN; power the MiniLab
   from the charger; connect the Pod to the computer with a **data** USB cable.
2. Confirm the Pod enumerates as a MIDI input named **"Daisy Pod MIDI"**
   (VID `0x1209`, PID `0xDA17`) with no driver install.
3. Open a MIDI monitor and select the **Pod's** port:
   - **macOS** — [MIDI Monitor](https://www.snoize.com/MIDIMonitor/) (or Audio MIDI Setup).
   - **Linux** — `aseqdump -l` to list, then `aseqdump -p "Daisy Pod MIDI"`.
   - **Windows** — MIDI-OX.
4. Play the MiniLab and watch the monitor:

   | MiniLab action        | Expected MIDI            | Packetizer path        |
   | --------------------- | ------------------------ | ---------------------- |
   | play / release a key  | Note On / Note Off       | `0x9`/`0x8`, running status |
   | hit a pad             | Note On/Off (or CC)      | ″                      |
   | turn a knob           | Control Change           | `0xB`                  |
   | pitch touch-strip     | Pitch Bend               | `0xE` (2 data bytes)   |
   | mod touch-strip       | CC #1 (mod wheel)        | `0xB`                  |
   | program change        | Program Change           | `0xC` (1 data byte)    |

**Pass** = the monitor shows the right messages with the right values, and fast
playing produces no stuck or dropped notes (that exercises running status and
confirms there's no UART overrun).

## Not covered by the MiniLab

In normal use the MiniLab 3 does not emit System Real-Time (MIDI clock /
start / stop), System Common, or SysEx over its TRS OUT — so the packetizer's
real-time-interleave and SysEx paths (which *are* unit-tested) aren't exercised
here. To check those on hardware, drive the Pod from a source that sends clock or
SysEx — e.g. a DAW's MIDI out through a USB↔DIN interface, or a groovebox sending
clock.

## Troubleshooting

- **Not enumerating** — the Pod USB cable must be data, not power-only; confirm
  `pod-midi` actually flashed and is running.
- **Enumerates but silent** — re-check the TRS Type (A vs B); confirm the
  MiniLab's TRS OUT is enabled and sending (its own USB-MIDI port will show
  activity too); make sure you're monitoring the **Pod's** port.
- **Stuck / dropped notes when playing fast** — points at a UART overrun or a
  running-status bug; capture the stream and file it.
- **Wrong values** — monitor the MiniLab's own USB-MIDI port alongside the Pod's
  to tell whether the source or our path is at fault.
