//! Software reset + field DFU entry for the Daisy Seed.
//!
//! In a **sealed pedal** the RESET button is inaccessible, so an app needs a
//! software path both to reboot and — crucially — to reboot *into* the
//! bootloader's DFU service mode so a new app can be flashed over USB with no
//! case-opening.
//!
//! An app calls [`reboot_to_bootloader`], which sets a flag in battery-backed
//! Backup SRAM and soft-resets. On the next boot the bootloader calls
//! [`take_dfu_request`] and, if the flag is set, stays in DFU service mode
//! instead of jumping to the app. The flag survives a soft reset (only a full
//! power-loss clears Backup SRAM), and is consumed-and-cleared so a *second*
//! reset boots the app normally.
//!
//! ## Backup SRAM layout (4 KiB @ `0x3880_0000`)
//! * `0x3880_0000` — clocks hand-off struct ([`crate::clocks::handoff`]).
//! * `0x3880_0FFC` — this module's DFU-request flag (the **top** word, so it
//!   never overlaps the hand-off struct at the base).

use cortex_m::peripheral::SCB;

/// Backup SRAM base (4 KiB, battery-backed).
pub const BACKUP_SRAM_BASE: usize = 0x3880_0000;

/// The word an app sets to ask the bootloader to stay in DFU service mode.
/// Placed at the **top** of Backup SRAM so it never overlaps the clocks
/// hand-off struct at the base.
pub const DFU_REQUEST_ADDR: *mut u32 = (BACKUP_SRAM_BASE + 0x0FFC) as *mut u32;

/// Magic written to [`DFU_REQUEST_ADDR`] to request DFU. Random garbage matching
/// it is 1-in-2^32; the bootloader consumes-and-clears it after acting.
pub const DFU_REQUEST_MAGIC: u32 = 0xB007_D45E;

const PWR_CR1: *mut u32 = 0x5802_4800 as *mut u32;
const RCC_AHB4ENR: *mut u32 = 0x5802_44E0 as *mut u32;

/// Enable Backup SRAM access: `PWR_CR1.DBP` (backup-domain write enable) +
/// `RCC_AHB4ENR.BKPRAMEN` (peripheral clock). Accessing `0x3880_0000` before
/// BKPRAMEN is set hard-faults on the H7. Idempotent.
///
/// # Safety
/// Touches PWR/RCC via raw pointers; the caller must own those registers (true
/// at reset time in the bootloader, and in single-threaded app teardown before
/// a self-reset).
pub unsafe fn enable_backup_sram() {
    core::ptr::write_volatile(PWR_CR1, core::ptr::read_volatile(PWR_CR1) | (1 << 8)); // DBP
    core::ptr::write_volatile(
        RCC_AHB4ENR,
        core::ptr::read_volatile(RCC_AHB4ENR) | (1 << 28), // BKPRAMEN
    );
    let _ = core::ptr::read_volatile(RCC_AHB4ENR); // read-back settles the clock
    cortex_m::asm::dsb();
}

/// Soft-reset the MCU immediately. Reboots into whatever the bootloader selects
/// (normally the same QSPI app). Never returns.
pub fn reboot() -> ! {
    cortex_m::asm::dsb();
    SCB::sys_reset()
}

/// Request that the bootloader stay in DFU service mode, then soft-reset. The
/// pedal comes back up ready for `daisy flash` over USB — no RESET button
/// needed. Never returns.
///
/// Trigger this from a deliberate user gesture (e.g. a footswitch held for a
/// couple of seconds) so it can't fire by accident during normal use.
pub fn reboot_to_bootloader() -> ! {
    // SAFETY: single-threaded teardown; we own PWR/RCC/Backup-SRAM here and are
    // about to reset the whole MCU.
    unsafe {
        enable_backup_sram();
        core::ptr::write_volatile(DFU_REQUEST_ADDR, DFU_REQUEST_MAGIC);
        cortex_m::asm::dsb();
    }
    reboot()
}

/// Read the DFU-request flag and, if set, clear it. Returns `true` when an app
/// asked for DFU since the last boot. The **bootloader** calls this once, after
/// [`enable_backup_sram`], to decide whether to stay in service mode.
///
/// # Safety
/// Backup SRAM must be enabled ([`enable_backup_sram`]) first, or the read
/// hard-faults.
pub unsafe fn take_dfu_request() -> bool {
    let requested = core::ptr::read_volatile(DFU_REQUEST_ADDR) == DFU_REQUEST_MAGIC;
    if requested {
        core::ptr::write_volatile(DFU_REQUEST_ADDR, 0); // consume-and-clear
        cortex_m::asm::dsb();
    }
    requested
}
