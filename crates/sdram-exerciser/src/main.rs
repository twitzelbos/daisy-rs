#![no_std]
#![no_main]

//! **SDRAM bring-up exerciser** — the firmware target for `renode/sdram_init.robot`.
//!
//! It runs the REAL [`daisy_bsp::sdram::init`] against Renode's
//! `STM32H7_FMC_SDRAM` model, then sweeps the 64 MiB window to prove the
//! controller became usable. The model leaves `0xC000_0000` UNUSABLE (reads
//! return 0, writes dropped) until the JEDEC power-up command sequence
//! (Clock-Config-Enable → Precharge-All → Auto-Refresh → Load-Mode-Register)
//! completes in order — so if `sdram::init` drove a wrong or mis-ordered
//! sequence, the read-back would return 0 and this exerciser reports errors.
//!
//! `sdram_fmc.robot` exercises that model with hand-written command words; this
//! validates that the actual `init()` FIRMWARE produces the same sequence. It is
//! NOT a DRAM cell test (Renode has a perfect backing store) — real cell/timing
//! faults are covered by the `daisy-sdram-test` CDC app on hardware.
//!
//! Runs standalone from internal flash on the reset clock — no bootloader / PLL;
//! the FMC model is functional, not cycle-accurate, so the timings don't matter.
//!
//! Markers (fixed DTCM, read by the robot):
//!   0x2001_F000  stage    (0x5D2A_11xx — last stage reached)
//!   0x2001_F004  errors   (SDRAM read-back mismatches)
//!   0x2001_F008  done     (0x0000_D09E when finished — written LAST)

use cortex_m_rt::entry;
use panic_halt as _;

// Fixed DTCM markers (below the stack, above .bss — same scheme the other
// exercisers use; the exerciser stack is a few KiB).
const M_STAGE: *mut u32 = 0x2001_F000 as *mut u32;
const M_ERRORS: *mut u32 = 0x2001_F004 as *mut u32;
const M_DONE: *mut u32 = 0x2001_F008 as *mut u32;

#[inline(always)]
fn mark(p: *mut u32, v: u32) {
    unsafe { core::ptr::write_volatile(p, v) }
}

/// Bounded busy-wait of roughly `us` microseconds. The exact duration is
/// irrelevant to the functional FMC model — `sdram::init` only needs a real,
/// terminating delay for its power-up settle.
fn delay_us(us: u32) {
    for _ in 0..us.saturating_mul(16) {
        cortex_m::asm::nop();
    }
}

const SDRAM: usize = 0xC000_0000;
const SDRAM_WORDS: usize = 0x0400_0000 / 4; // 16 Mi words

#[inline(always)]
fn wr(word: usize, v: u32) {
    unsafe { core::ptr::write_volatile((SDRAM + word * 4) as *mut u32, v) }
}
#[inline(always)]
fn rd(word: usize) -> u32 {
    unsafe { core::ptr::read_volatile((SDRAM + word * 4) as *const u32) }
}

// Unique-per-index value (own-address hashed) — distinct across the window, so a
// mis-mapped/aliasing controller mismatches too.
#[inline(always)]
fn expected(i: usize) -> u32 {
    0xC0DE_0000 ^ (i as u32).wrapping_mul(0x9E37_79B1)
}

#[entry]
fn main() -> ! {
    mark(M_STAGE, 0x5D2A_1101); // reached main
    mark(M_ERRORS, 0);
    mark(M_DONE, 0);

    // Real BSP bring-up — drives the FMC model's SDCMR command-sequence gating.
    unsafe { daisy_bsp::sdram::init(delay_us) };
    mark(M_STAGE, 0x5D2A_1102); // init() returned

    let mut errs: u32 = 0;

    // Sparse sweep spanning the whole 64 MiB (stride 16 KiB): proves the window
    // is live end-to-end and each sampled address is distinct.
    const N: usize = 4096;
    let stride = SDRAM_WORDS / N;
    for i in 0..N {
        wr(i * stride, expected(i));
    }
    for i in 0..N {
        if rd(i * stride) != expected(i) {
            errs += 1;
        }
    }
    mark(M_STAGE, 0x5D2A_1103); // sparse sweep done

    // Dense own-address over the first 1 KiB (adjacent cells).
    for i in 0..256 {
        wr(i, i as u32);
    }
    for i in 0..256 {
        if rd(i) != i as u32 {
            errs += 1;
        }
    }
    mark(M_STAGE, 0x5D2A_1104); // dense sweep done

    mark(M_ERRORS, errs);
    mark(M_DONE, 0x0000_D09E); // signal completion LAST

    loop {
        cortex_m::asm::nop();
    }
}
