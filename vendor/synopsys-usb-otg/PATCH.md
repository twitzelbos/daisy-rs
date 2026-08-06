# daisy-rs local patches on synopsys-usb-otg 0.4.0

This is a vendored copy of `synopsys-usb-otg 0.4.0` from crates.io with two
targeted patches applied. The patches make the crate work correctly on the
STM32H7 family, which upstream 0.4.0 does not fully support.

Both changes are in `src/bus.rs`; nothing else in the crate is modified.

## Patch A — STM32H7 core-ID arm in `configure_all()`

**Where:** the `match core_id { … }` block near the end of `UsbBus::enable()`.
Upstream handles F429 (`0x0000_1200 / 0x1100`) and F446 / F72x
(`0x0000_2000..=0x3100`) core IDs; the STM32H7's OTG core reports
`0x4F54_300A` (per ST's `stm32h7xx_ll_usb.h`) which falls into the default
`_ => {}` arm — so the H7-specific VBUS-sensing / session-validity fixups
never run. With `vbus_sensing_enable = disabled` (the standard config on
boards like the Electrosmith Daisy Seed that source VDD33USB externally),
`GOTGCTL.BVALOEN / BVALOVAL` are never set, the OTG core never asserts
B-session-valid, and the device never enumerates.

**Fix:** add an arm matching H7 core IDs (`0x4F54_300A / 300B / 310A`) that
does what the F446-family arm does — clear `GCCFG.VBDEN` and set
`GOTGCTL.BVALOEN + BVALOVAL`.

**Refs:**
- ST's `stm32h7xx_hal_driver/Src/stm32h7xx_ll_usb.c:301-322` (`USB_DevInit`)
  performs the same register writes in the `vbus_sensing_enable == 0`
  branch.
- RM0433 §57.14.1 (GOTGCTL, p. 2621) — BVALOEN/BVALOVAL semantics.
- RM0433 §57.14.15 (GCCFG, p. 2647) — VBDEN semantics.
- ST core-ID constants: `stm32h7xx_hal_driver/Inc/stm32h7xx_ll_usb.h:268`.

## Patch B — Move FDMOD write to AFTER the core soft-reset (with CMOD poll)

**Where:** upstream's `hs`-branch `modify_reg!(GUSBCFG, SRPCAP:0, TOCAL:0x1,
FDMOD:1)` sits at ~line 385 — **before** the CSRST at ~line 462. The
FDMOD bit is deleted from that pre-reset write and re-added in a new
block inserted **after** the `while … CSRST == 1 {}` wait.

**Fix:**

1. In the pre-reset `hs`-branch write, keep only `SRPCAP: 0, TOCAL: 0x1`.
   Do NOT set FDMOD there.
2. Immediately after the CSRST wait loop, add:
   ```rust
   #[cfg(feature = "hs")]
   {
       modify_reg!(otg_global, regs.global(), GUSBCFG, FDMOD: 1);
       #[cfg(feature = "cortex-m")]
       {
           cortex_m::asm::delay(12_000_000);   // 25 ms guarantee
           let mut budget: u32 = 24_000_000;   // ~50 ms poll budget
           while read_reg!(otg_global, regs.global(), GINTSTS, CMOD) != 0
               && budget > 0
           {
               cortex_m::asm::delay(1_000);
               budget = budget.saturating_sub(1_000);
           }
       }
   }
   ```

**Why:** RM0433 §57.14.5 (GRSTCTL): *"CSRST resets the core state machine
and all the CSRs except the AHB configuration registers to their default
value."* Since `GUSBCFG.FDMOD`'s reset value is 0, any FDMOD write done
before CSRST is wiped by the reset — the peripheral then auto-selects
mode from the OTG_ID pin. On boards like the Daisy Seed with a floating
ID line, the SIE ends up in a hybrid state (USBSUSP fires, so it's
device-mode-adjacent) but the D+ pull-up never engages and the host sees
nothing on the bus.

RM0433 §57.14.4 (GUSBCFG.FDMOD): *"After setting the force bit, the
application must wait at least 25 ms before the change takes effect."*
The 25 ms `asm::delay` satisfies that guarantee. The subsequent
`GINTSTS.CMOD` (bit 0, 0 = device, 1 = host) poll mirrors ST HAL's
`USB_SetCurrentMode` pattern (`ll_usb.c:254-291`).

**Refs:**
- RM0433 §57.14.5 (GRSTCTL / CSRST)
- RM0433 §57.14.4 (GUSBCFG / FDMOD 25 ms note)
- RM0433 §57.14.6 (GINTSTS / CMOD)
- `stm32h7xx_hal_driver/Src/stm32h7xx_ll_usb.c:254-291`
  (`USB_SetCurrentMode`)
- `stm32h7xx_hal_driver/Src/stm32h7xx_hal_pcd.c:175-186` (init ordering:
  CoreInit → SetCurrentMode → DevInit)

Gated on `feature = "hs"` for the whole block; the `cortex_m::asm::delay`
+ poll are further gated on `feature = "cortex-m"` so RISC-V users can
plug in an equivalent wait/poll themselves.

**Prior revision:** an earlier version of this patch added a 25 ms
`asm::delay` immediately after the pre-reset FDMOD write. That was dead
weight — CSRST wiped FDMOD right after the delay elapsed, defeating the
guarantee. Kept in commit history so future maintainers know why the
naive fix doesn't work.

## Upstreaming

Both patches are single-cause, mechanical, and well-motivated by ST HAL
source and RM0433 quotes. They should be viable PRs to
[stm32-rs/synopsys-usb-otg](https://github.com/stm32-rs/synopsys-usb-otg).
Once upstream accepts a release with these fixes, this vendored copy can
be dropped and `Cargo.toml`'s `[patch.crates-io]` entry removed.

## Preserving the patches on version bumps

If bumping `synopsys-usb-otg` to a newer version:

1. Re-vendor the new version to `vendor/synopsys-usb-otg/` (fresh copy).
2. Re-apply Patch A — search for the last arm in the `configure_all()`
   `match core_id` (the F446 arm ends with the BVALOEN/BVALOVAL write) and
   insert the H7 arm immediately after.
3. Re-apply Patch B — search for the `hs`-branch FDMOD write and add the
   gated `cortex_m::asm::delay(12_000_000)` immediately after.
4. Update this file with the new line numbers if they shifted.
