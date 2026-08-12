# USB-audio bringup on hardware — 2026-08-12

First attempt to bring up the composite `daisy-usb-audio` XIP app end-to-end on
real hardware (Daisy Seed 1.1 / STM32H750, ST-Link V3 + probe-rs, board's
micro-USB into a Mac). This surfaced **two real bootloader bugs** (both fixed
here) and narrowed the remaining blocker to app-level USB enumeration.

> macOS note: `system_profiler SPUSBDataType` returns 0 bytes on the test Mac
> (broken). Use `ioreg -p IOUSB -l`. The authoritative signals below are read
> **device-side over SWD** (OTG core registers), immune to host-tool flakiness.

## Bootloader bug 1 — host enumeration pinned the bootloader in DFU

`daisy-boot` decided "a host wants to flash me, stay in DFU service mode" from
`dfu_saw_activity()` = *"has the DFUse address pointer moved from its initial
value?"* (`main.rs`). On macOS (no DFU driver) the mere act of **enumerating**
the DFU interface moved the pointer, so the bootloader committed to service mode
and **never jumped to the app** while plugged into the Mac (confirmed on HW:
`VTOR` stuck at `0x0800_0000` past the boot window). Apps only booted with USB
disconnected during the window.

**Fix:** commit to service mode only on a **real flash write** — a new
`DFU_SAW_WRITE` flag set from `store_write_buffer`/`erase`/`program`/`erase_all`
in `dfu_mem.rs`, which host enumeration never calls. Verified on HW: the
bootloader now jumps to the app on a cold boot with USB connected.

## Bootloader bug 2 — no clean USB disconnect at hand-off

Once bug 1 was fixed and the bootloader jumped, the app **still didn't
enumerate**. Device-side OTG registers showed the app running fine (`VTOR =
0x9000_0000`, no fault) and presenting a valid FS device (`DCTL.SDIS`=0 pullup
on, `GOTGCTL.BSVLD`=1), but **`DCFG` device address = 0** and
**`DSTS.SUSPSTS` = 1** — the host never even issued a bus reset.

Root cause: the DFU device's D+ pullup is asserted for the whole boot window,
and `dfu.release()` doesn't drop it. The bootloader jumped to the app with the
pullup continuously on, so the host kept its stale view of the DFU device and
never re-enumerated the app (whose OTG core re-inits to address 0). The OTG
register config was **byte-identical** between the working bootloader USB and
the ignored app USB — proving it was a hand-off timing problem, not a config
one.

**Fix:** assert `DCTL.SDIS` (soft disconnect) right after the boot window, so
the ~6 s of `stage_pulses` LED diagnostics hold a clean disconnect before the
jump; the app re-asserts the pullup on its own bringup and the host sees a fresh
connect. Verified on HW: after this fix the host **starts enumerating** the app
(`DSTS.SUSPSTS` flips 1→0, bus reset observed, `ENUMSPD` = FS) — a state it never
reached before.

## Remaining blocker (open) — app enumeration doesn't complete

With both bootloader fixes, the host now resets and begins enumerating the app,
but enumeration **doesn't complete**: device address stays 0 and the device
falls back to suspend. Verified on HW that the app's OTG init follows the RM
device sequence — `GAHBCFG.GINT`=1, `GINTMSK` RXFLVL/USBRST/ENUMDNE unmasked,
`NVIC` OTG_FS (IRQ 101) enabled, `DCFG` speed set — and `ICSR.VECTACTIVE`=0 (not
wedged in a fault/ISR; the default build sleeps in `wfi` between USB events, so a
static heartbeat while suspended is expected).

The failure is at the **EP0 / descriptor stage, before addressing**. Leading
suspect: the composite CDC + UAC1 + MIDI descriptor set, which macOS validates
far more strictly than Renode's OTG model (where this app enumerates in sim).
Next steps: capture the enumeration on the host (or a bus analyzer), audit the
UAC1 + IAD descriptors against the USB-audio spec, and instrument the app's EP0
path with DTCM markers to see how far the first `GET_DESCRIPTOR` gets.

## Status

| Item | Result |
| --- | --- |
| Bootloader: host-enumeration DFU pin | **fixed + HW-verified** |
| Bootloader: hand-off USB disconnect | **fixed + HW-verified** (host now enumerates the app) |
| App: composite USB enumeration completes | **open** — stalls at EP0/descriptor; UAC descriptor audit next |
