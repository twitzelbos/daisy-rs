# Plan: software reset & field DFU entry for sealed pedals

## Motivation

In a bench setup you reflash a Daisy by tapping the **RESET** button to hit the
bootloader's ~2 s DFU window. In a **sealed pedal (e.g. a Hothouse)** the RESET
button is inside the enclosure and inaccessible. A deployed pedal therefore
needs a **software-triggered** way to:

1. **Reboot** the running app (soft reset), and
2. **Reboot _into_ the bootloader's DFU service mode** so a new app can be
   flashed over USB — via a user gesture (footswitch combo) or a host command,
   with no case-opening and no RESET button.

This is the whole reason the bootloader exists (field-updatable pedals), but the
path is currently **designed yet disconnected**. This plan wires it up.

## Current state (what exists, what's missing)

| Piece | Status | Location |
|---|---|---|
| Bootloader reads a "stay in DFU" magic | **Disabled** — `let force_dfu = false;` (TODO) | `crates/daisy-boot/src/main.rs:256` |
| Magic constant + slot address | Defined | `main.rs:187` `BACKUP_SRAM_MAGIC_ADDR = 0x3880_0000`, `:189` `BOOTLOADER_MAGIC = 0xB007_D45E` |
| Boot decision honoring `force_dfu` | Present | `main.rs:328` `if force_dfu \|\| !qspi_looks_valid { enter_service_mode(...) }` |
| Backup-SRAM enable (DBP + BKPRAMEN) | Present, reusable | `crates/daisy-bsp/src/clocks.rs:127` `enable_backup_sram()` (private, in `handoff`) |
| Hothouse DFU gesture predicate | Present, **unused** | `crates/daisy-bsp/src/hothouse.rs:288` `dfu_gesture()` = both footswitches held |
| App-side reboot / magic-write / gesture action | **Missing** | — |
| `daisy-bsp` reboot helper | **Missing** | — |

So: the bootloader can be _told_ to stay in DFU, and the Hothouse can _detect_
the gesture, but nothing connects the gesture → magic → reset → bootloader.

## The blocker that parked this: two issues to resolve

### 1. The magic slot collides with the clocks hand-off
`handoff::stash()` writes its `Handoff` struct to **`0x3880_0000`** with its own
magic `0xDA15_C0C0` at offset 0 (`clocks.rs:111,113`) — the **same word** the
bootloader reads `BOOTLOADER_MAGIC` from (`main.rs:187`). They overwrite each
other. Because the magic check is disabled, it hasn't bitten yet.

**Fix:** give the DFU-request flag its own reserved word, distinct from the
hand-off struct. Backup SRAM is 4 KiB (`0x3880_0000`–`0x3880_0FFF`); the hand-off
struct lives at the base. Put the DFU flag at the **top word, `0x3880_0FFC`**.
Update both the bootloader read and the app write to use it. Document the Backup
SRAM layout in one place (see "Shared constants" below).

### 2. Ordering: the magic must be read after `BKPRAMEN` is on
Reading `0x3880_0000` before `RCC_AHB4ENR.BKPRAMEN` is set hard-faults (the
original TODO reason, `main.rs:251`). In today's flow `BKPRAMEN` is enabled at
`main.rs:271` (`handoff::stash` → `enable_backup_sram`), which is **after**
`force_dfu` is decided at `:256` but **before** the boot decision at `:328`.

**Fix:** read the DFU flag **between** the Backup-SRAM enable and the boot
decision. Restructure to: enable Backup SRAM → read + clear the DFU flag → stash
clocks → boot decision. (The flag at `0x3880_0FFC` and the hand-off struct at
`0x3880_0000` no longer overlap, so read order vs. stash is safe either way, but
reading before stash keeps intent clear.)

## Design

### Shared constants — single source of truth in `daisy-bsp`
Move the magic + slot out of `daisy-boot` into `daisy-bsp` so the bootloader and
every app agree. New `crates/daisy-bsp/src/reset.rs`:

```rust
/// Backup SRAM base (4 KiB, battery-backed). Layout:
///   0x3880_0000 : clocks hand-off struct (see clocks::handoff)
///   0x3880_0FFC : DFU-request flag (this module)  ← top word, no overlap
pub const BACKUP_SRAM_BASE: usize = 0x3880_0000;
/// Word an app sets to ask the bootloader to stay in DFU service mode.
pub const DFU_REQUEST_ADDR: *mut u32 = (BACKUP_SRAM_BASE + 0x0FFC) as *mut u32;
pub const DFU_REQUEST_MAGIC: u32 = 0xB007_D45E;
```

`daisy-boot` imports these instead of its private copies.

### `daisy-bsp::reset` — reboot helpers
```rust
/// Soft-reset the MCU immediately. Reboots into whatever the bootloader
/// selects (normally the same QSPI app).
pub fn reboot() -> ! {
    cortex_m::asm::dsb();
    cortex_m::peripheral::SCB::sys_reset() // never returns
}

/// Ask the bootloader to stay in DFU service mode, then soft-reset. The pedal
/// comes up ready for `daisy flash` over USB — no RESET button needed.
pub fn reboot_to_bootloader() -> ! {
    unsafe {
        enable_backup_sram();                        // DBP + BKPRAMEN (shared)
        core::ptr::write_volatile(DFU_REQUEST_ADDR, DFU_REQUEST_MAGIC);
        cortex_m::asm::dsb();
    }
    reboot()
}
```

`enable_backup_sram()` currently lives (private) in `clocks::handoff`. Promote it
to `pub(crate)` in a shared spot (or duplicate the two-register write in
`reset.rs`; it's idempotent). Backup SRAM survives a `sys_reset` (only a full
power-loss clears it), so the flag reliably reaches the next boot.

### Bootloader change (`daisy-boot/src/main.rs`)
Replace `let force_dfu = false;` (`:256`) with a real read, placed after Backup
SRAM is enabled and before the boot decision (`:328`):

```rust
// after handoff::stash(...) has enabled BKPRAMEN (or an explicit enable here):
let force_dfu = unsafe {
    let req = core::ptr::read_volatile(daisy_bsp::reset::DFU_REQUEST_ADDR);
    if req == daisy_bsp::reset::DFU_REQUEST_MAGIC {
        core::ptr::write_volatile(daisy_bsp::reset::DFU_REQUEST_ADDR, 0); // consume-and-clear
        cortex_m::asm::dsb();
        true
    } else {
        false
    }
};
```

Consume-and-clear so a second reset boots the app normally (matches the existing
`main.rs:19` intent). The existing `if force_dfu || !qspi_looks_valid` at `:328`
already routes into `enter_service_mode` — no change there.

### App triggers

**memtest (`daisy-sdram-test`)** — add two keys to the CDC control scan in
`poll_ui` (alongside SPACE / `s`), avoiding CPR-reply bytes (digits, `R`):
- `r` → `daisy_bsp::reset::reboot()`
- `b` → `daisy_bsp::reset::reboot_to_bootloader()`

Update the on-screen controls hint (`tui.rs`) to list them.

**Hothouse (`daisy-hothouse`)** — the load-bearing case. In the main loop, when
`switches.dfu_gesture()` (both footswitches, `hothouse.rs:288`) is held for
**~2 s**, call `reboot_to_bootloader()`. Add a hold-timer (DWT-based, like the
existing timing) so a brief double-stomp during play doesn't trigger it, and
surface it on the panel: e.g. once the gesture starts, show "hold to enter DFU…"
and pulse the footswitch LEDs, committing at 2 s. Optionally add a `q`/`b` CDC
key too, since the Hothouse panel is already a CDC TUI.

Any other CDC app (`daisy-usb-audio`) can adopt the same `r`/`b` keys trivially.

## Testing

### Renode
- `renode/daisy_seed_helpers.py` already "plants the magic" for the bootloader
  (`main.rs:183`), so the harness hook exists. Add **`renode/bootloader_dfu_request.robot`**:
  plant `DFU_REQUEST_MAGIC` at `0x3880_0FFC` → boot `daisy-boot` (renode_test) →
  assert it enters service mode (the 1 Hz service-blink / does **not** jump to
  the app), and assert the flag word is cleared.
- A negative test: no magic + valid QSPI → jumps to app (existing behavior).
- Wire both into `renode/build-and-run.sh`.

### Host
- Unit-test the pure bits in `daisy-bsp` (const values, the layout doesn't
  overlap the hand-off struct size) where feasible.

### Hardware (deferred — HW currently disconnected)
- memtest: press `b` in picocom → board drops to DFU → `daisy flash -p …` with
  no RESET tap. Press `r` → clean reboot back into memtest.
- Hothouse (sealed): hold both footswitches ~2 s → pedal enters DFU → reflash.
  Confirm the panel hint + LED feedback. Confirm a normal quick double-stomp
  does **not** trigger it.

## Risks / considerations
- **Backup domain power:** the flag survives `sys_reset` but not a full power
  cycle — which is fine (we only need it across the soft reset). Confirm VBAT is
  fed on the Daisy (it is on-board; Backup SRAM already used for the clocks
  hand-off, so this is proven).
- **MPU:** apps map Backup SRAM **non-cacheable** already (`daisy-hothouse/src/main.rs:165`,
  `daisy-usb-audio/src/main.rs:185`), so the flag write is immediately visible to
  the bootloader after reset — no cache maintenance needed.
- **Don't loop into DFU:** consume-and-clear in the bootloader is essential.
- **Accidental gesture:** the 2 s hold + LED feedback prevents a stray both-stomp
  from bricking a performance; make the timeout tunable.
- **Bootloader is boot-critical:** this touches the boot path. Gate behind the
  Renode `bootloader_dfu_request.robot` + a careful HW bring-up (keep the SWD/DFU
  fallback: even without the magic, the ~3 s post-reset DFU window still exists).

## Task breakdown (suggested PR sequencing)
1. **`daisy-bsp::reset`** — shared consts + `reboot()` / `reboot_to_bootloader()`;
   promote `enable_backup_sram`. Relocate the DFU flag to `0x3880_0FFC`. (lib only)
2. **Bootloader** — import the shared consts, re-enable the magic read
   (consume-and-clear) after `BKPRAMEN`. + `bootloader_dfu_request.robot`.
3. **memtest keys** `r` / `b` + hint. **Hothouse** 2 s footswitch-hold → DFU, with
   panel feedback.
4. **Docs** — update `docs/daisy-pinout.md`/README with the field-update gesture;
   note it in the bootloader module docs.

Items 1+2 are the load-bearing change (make the bootloader honor a software DFU
request); 3 is the per-app UX; 4 is documentation. 1 and 2 should land together
(the bootloader depends on the shared consts).
