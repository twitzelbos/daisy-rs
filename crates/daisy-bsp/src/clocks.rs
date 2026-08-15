//! Clock tree init for the Daisy Seed.
//!
//! The Seed has a 16 MHz HSE crystal. We spin PLL1 up to 400 MHz sysclk and,
//! libDaisy-style, configure ALL the peripheral kernel PLLs up front in the
//! single `freeze()` — via the HAL's tested PLL builder, NOT hand-rolled RCC
//! registers:
//!   * **PLL2R = 200 MHz** — the FMC/SDRAM kernel clock (SDCLK = 200 / 2 =
//!     100 MHz). See [`crate::sdram`], which now only routes + programs the FMC
//!     and relies on this PLL already running.
//!   * **PLL3P ≈ 49.152 MHz** — the SAI1 kernel clock (→ 12.288 MHz MCLK after
//!     the SAI MCKDIV ÷4), for audio. See `daisy-audio`.
//!
//! USB uses HSI48 (configured by the USB subsystem), not a PLL.
//!
//! Doing this here means every app — the XIP apps loaded by the bootloader, and
//! monolithic apps that call `init` directly — inherits a fully-configured clock
//! tree, so no app needs to hand-poke RCC post-freeze (the pattern that hid the
//! earlier PLL2 bit-offset bug).

use crate::hal::pac;
use crate::hal::prelude::*;
use crate::hal::rcc::{Ccdr, PllConfigStrategy};

/// Bring the chip up to its maximum supported sysclk — **480 MHz / VOS0 on
/// silicon revision V**, **400 MHz / VOS1 on anything else** — with PLL2 (FMC)
/// and PLL3 (SAI) running. The revision is detected at runtime so one binary is
/// safe on any Daisy Seed.
///
/// Consumes `PWR` and `RCC` (they're single-shot inits) but only borrows
/// `SYSCFG` since callers still need it for other subsystems.
pub fn init(pwr: pac::PWR, rcc: pac::RCC, syscfg: &pac::SYSCFG) -> Ccdr {
    // Runtime silicon-revision gate. 480 MHz/VOS0 is available ONLY on rev V
    // (and later) of the STM32H750/743 — DBGMCU_IDCODE DEV_ID = 0x450 and
    // REV_ID >= 0x2003. Rev Y (0x1003) is hard-capped at 400 MHz/VOS1, and
    // forcing VOS0 there HANGS (the overdrive VOSRDY spin never completes), so
    // we fall back to 400 for any part that isn't a confirmed rev-V H750/743.
    // DBGMCU_IDCODE needs no clock/reset — it's always readable.
    let idcode = unsafe { core::ptr::read_volatile(0x5C00_1000 as *const u32) };
    let use_480 = (idcode & 0x0FFF) == 0x450 && (idcode >> 16) >= 0x2003;

    // 480 MHz needs VOS0 (overdrive). The "lto=fat DCEs main at VOS0" landmine
    // was a MISDIAGNOSIS of stm32h7xx-hal#403: the default
    // `PllConfigStrategy::Normal` caps PLL1's VCO at 420 MHz, so `sys_ck(480)`
    // makes `assert!(vco_ck <= vco_max)` compile-provably false and fat-LTO folds
    // it to a panic. `Iterative` uses the wide 192–836 MHz VCO, so 480 validates.
    // (The runtime `if` also stops the const-fold: the 400 branch is a valid
    // config, so neither branch plants an always-false assert.) `hclk` auto-
    // divides to the VOS ceiling; PLL2R (FMC) and PLL3P (SAI) are independent.
    let pwrcfg = if use_480 {
        pwr.constrain().vos0(syscfg).freeze()
    } else {
        pwr.constrain().freeze()
    };

    let rcc = rcc.constrain().use_hse(16.MHz());
    let rcc = if use_480 {
        rcc.sys_ck(480.MHz())
            .pll1_strategy(PllConfigStrategy::Iterative)
    } else {
        rcc.sys_ck(400.MHz())
    };
    rcc
        // PLL2R = 200 MHz → FMC/SDRAM kernel clock (SDCLK = 200 / 2 = 100 MHz).
        // NB: the HAL only sets PLL2ON when `pll2_p_ck` is requested (rcc/mod.rs
        // `if pll2_p_ck.is_some()`), so requesting `pll2_r_ck` ALONE leaves PLL2
        // OFF and PLL2R at 0 Hz — which would dead-clock the FMC. Request P (to
        // enable the PLL) AND R (the output the FMC mux actually selects).
        //
        // PLL2P = 40 MHz. Its ONLY jobs are (a) enabling PLL2 for R above and
        // (b) being the default ADC kernel clock (ADCSEL=pll2_p). It MUST stay
        // ≤ 80 MHz: at VOS1 the ADC kernel clock is capped at 80 MHz (RM0433
        // Table 59), and the HAL's `Adc::adc1` asserts this — at 200 MHz that
        // assert panics and the app hangs during ADC bring-up. 40 MHz shares
        // PLL2's 400 MHz VCO (DIVP=10) so PLL2R/SDRAM at 200 MHz is unaffected.
        .pll2_p_ck(40.MHz())
        .pll2_r_ck(200.MHz())
        // PLL3P → SAI1 kernel clock. Fractional strategy so it lands on
        // 49.152 MHz (= 256 × 48 kHz × 4); the default integer strategy only
        // reaches 49.0 MHz (a ~0.3 % sample-rate error). MCKDIV ÷4 → 12.288 MHz.
        .pll3_strategy(PllConfigStrategy::Fractional)
        .pll3_p_ck(49_152_000.Hz())
        .freeze(pwrcfg, syscfg)
}

/// Hand-off of the frozen [`CoreClocks`](crate::hal::rcc::CoreClocks) from the
/// bootloader to a QSPI-XIP application.
///
/// An XIP app runs *after* the bootloader's `freeze()`, so it cannot mint its
/// own `CoreClocks`: the struct has no public constructor, and re-running
/// `freeze()` is unsafe under XIP (it reconfigures the very clock feeding the
/// QSPI instruction fetch). Instead the bootloader stashes its REAL
/// `CoreClocks` — which is `#[derive(Copy)]` and pure plain-data — into
/// battery-backed Backup SRAM, and the app copies it back. This recovers the
/// genuine frozen clock config so the app can use HAL drivers that require a
/// `&CoreClocks` (SAI `i2s_ch_a`, `I2c::new`, SPI, …) instead of hand-rolling
/// each peripheral's kernel-clock maths.
///
/// # Version coupling (guarded)
/// The bootloader and app MUST link the same `stm32h7xx-hal` version — the
/// `CoreClocks` layout is `#[repr(Rust)]` and only matches across binaries
/// built from the same crate version (the workspace `Cargo.lock` pins it). A
/// `sys_ck` guard word is stored as a plain `u32` at a fixed `#[repr(C)]`
/// offset; [`handoff::restore`] re-reads `sys_ck` through the recovered struct
/// and, on a layout mismatch, it won't equal the guard, so `restore` returns
/// `None` rather than trusting garbage.
#[cfg(target_os = "none")]
pub mod handoff {
    use crate::hal::rcc::CoreClocks;

    /// Backup SRAM base (4 KiB, battery-backed; the app maps it non-cacheable).
    const ADDR: usize = 0x3880_0000;
    /// Marks a valid hand-off. Random garbage matching this is 1-in-2^32.
    const MAGIC: u32 = 0xDA15_C0C0;

    // `#[repr(C)]` fixes `magic`/`sys_ck` at offsets 0/4 so the guard words can
    // be read as plain u32 regardless of the (repr(Rust)) `CoreClocks` layout.
    #[repr(C)]
    struct Handoff {
        magic: u32,
        sys_ck: u32,
        clocks: CoreClocks,
    }

    /// Enable the Backup SRAM clock (RCC_AHB4ENR.BKPRAMEN) + backup-domain write
    /// access (PWR_CR1.DBP). Accessing 0x3880_0000 before BKPRAMEN is set
    /// hard-faults on the H7. Idempotent.
    unsafe fn enable_backup_sram() {
        const PWR_CR1: *mut u32 = 0x5802_4800 as *mut u32;
        const RCC_AHB4ENR: *mut u32 = 0x5802_44E0 as *mut u32;
        core::ptr::write_volatile(PWR_CR1, core::ptr::read_volatile(PWR_CR1) | (1 << 8)); // DBP
        core::ptr::write_volatile(
            RCC_AHB4ENR,
            core::ptr::read_volatile(RCC_AHB4ENR) | (1 << 28), // BKPRAMEN
        );
        let _ = core::ptr::read_volatile(RCC_AHB4ENR); // enable read-back settles the clock
        cortex_m::asm::dsb();
    }

    /// Bootloader: stash the frozen clocks for the XIP app. Call after
    /// [`super::init`], before jumping to the app.
    ///
    /// # Safety
    /// Writes Backup SRAM via raw pointers; the caller must own the backup
    /// domain (true in the single-threaded bootloader before the jump).
    pub unsafe fn stash(clocks: &CoreClocks) {
        enable_backup_sram();
        let h = Handoff {
            magic: MAGIC,
            sys_ck: clocks.sys_ck().raw(),
            clocks: *clocks,
        };
        core::ptr::write_volatile(ADDR as *mut Handoff, h);
        cortex_m::asm::dsb();
    }

    /// App: recover the bootloader's frozen clocks. Returns `None` if no valid
    /// hand-off is present (bad magic) or the guard fails (HAL-version/layout
    /// mismatch) — the caller should treat `None` as "clocks unavailable" and
    /// not fabricate one.
    ///
    /// # Safety
    /// Reads Backup SRAM via raw pointers. The recovered `CoreClocks` is only
    /// sound when the app and bootloader link the same `stm32h7xx-hal` version;
    /// the guard catches the mismatch but the caller must still uphold that the
    /// stash was written by a matching bootloader.
    pub unsafe fn restore() -> Option<CoreClocks> {
        enable_backup_sram();
        // Read the plain guard word first (fixed repr(C) offset 0) so we never
        // materialise a `CoreClocks` — which holds `Option<Hertz>` that could be
        // an invalid discriminant on a layout mismatch — until MAGIC confirms a
        // same-version bootloader wrote this.
        if core::ptr::read_volatile(ADDR as *const u32) != MAGIC {
            return None;
        }
        let h = core::ptr::read_volatile(ADDR as *const Handoff);
        if h.clocks.sys_ck().raw() == h.sys_ck {
            Some(h.clocks)
        } else {
            None
        }
    }
}
