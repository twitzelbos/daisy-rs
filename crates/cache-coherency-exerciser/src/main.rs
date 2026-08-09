#![no_std]
#![no_main]

//! D-cache / DMA coherency exerciser — the test target for Renode's
//! `CacheCoherencyChecker`. Drives BOTH M7 coherency hazards, each in a
//! buggy variant (no maintenance) and a correct variant (maintenance by MVA),
//! selected by the harness at run time so one ELF proves catch-the-bug AND
//! no-false-positive for both directions.
//!
//! The M7 D-cache is not coherent with DMA (PM0253 / RM0433); firmware must
//! clean or invalidate by address around a DMA'd cacheable buffer.
//!
//! Phase 1 — stale READ (DMA writes, CPU reads):
//!   1. CPU reads the shared buffer at 0x3000_0000 → the line is now cached.
//!   2. Signal `CACHED`, spin until `GO1`. The harness injects a foreign-master
//!      (DMA) write into the buffer in between and, via `FIX1`, chooses whether
//!      firmware should invalidate.
//!   3. If `FIX1`, invalidate the line by address (SCB `DCIMVAC`).
//!   4. CPU re-reads. Without the invalidate the CPU reads STALE cached data on
//!      silicon; the checker flags exactly that.
//!
//! Phase 2 — stale DMA read of a DIRTY line (CPU writes, DMA reads):
//!   5. If `FIX2`, firmware writes the buffer then cleans the line by address
//!      (SCB `DCCMVAC`) so the write reaches backing memory; else it writes and
//!      leaves the line dirty in cache.
//!   6. Signal `DIRTY`, spin until `GO2`. The harness performs a foreign-master
//!      (DMA) READ in between — which on silicon reads STALE backing memory if
//!      the line was never cleaned; the checker flags exactly that.
//!
//! The buffer is treated as cacheable + write-back (the checker assumes the
//! overlaid region is cacheable). Runs on the reset clock — no PLL/RCC needed.

use core::ptr::{read_volatile, write_volatile};

use cortex_m_rt::entry;
use panic_halt as _;

const BUF: *mut u32 = 0x3000_0000 as *mut u32;
const DCIMVAC: *mut u32 = 0xE000_EF5C as *mut u32; // SCB D-cache invalidate by MVA
const DCCMVAC: *mut u32 = 0xE000_EF68 as *mut u32; // SCB D-cache clean by MVA

const M: *mut u32 = 0x2001_F000 as *mut u32; // DTCM markers
const M_CACHED: isize = 0; // phase 1: buffer cached, waiting for the injected DMA write
const M_GO1: isize = 1; // harness → fw: proceed to the phase-1 re-read
const M_FIX1: isize = 2; // harness → fw: 1 = invalidate before re-reading
const M_RESULT: isize = 3; // the value the CPU read back in phase 1
const M_DONE1: isize = 4; // 0xD09E when phase 1 finished
const M_DIRTY: isize = 5; // phase 2: buffer written (dirty), waiting for the DMA read
const M_GO2: isize = 6; // harness → fw: phase 2 done, proceed to finish
const M_FIX2: isize = 7; // harness → fw: 1 = clean the line after writing it
const M_DONE2: isize = 8; // 0xD09E when phase 2 finished

const DONE: u32 = 0x0000_D09E;

#[entry]
fn main() -> ! {
    unsafe {
        for i in 0..=M_DONE2 {
            write_volatile(M.offset(i), 0);
        }

        // ---- Phase 1: stale READ (DMA writes, CPU reads) --------------------
        let _first = read_volatile(BUF); // cache the line
        write_volatile(M.offset(M_CACHED), 1);
        while read_volatile(M.offset(M_GO1)) == 0 {
            cortex_m::asm::nop();
        }
        if read_volatile(M.offset(M_FIX1)) != 0 {
            write_volatile(DCIMVAC, BUF as u32);
            cortex_m::asm::dsb();
        }
        let v = read_volatile(BUF); // STALE on silicon if not invalidated
        write_volatile(M.offset(M_RESULT), v);
        write_volatile(M.offset(M_DONE1), DONE);

        // ---- Phase 2: stale DMA read of a DIRTY line (CPU writes, DMA reads)-
        write_volatile(BUF, 0xCAFE_BABE); // CPU write → line dirty in write-back cache
        if read_volatile(M.offset(M_FIX2)) != 0 {
            write_volatile(DCCMVAC, BUF as u32); // clean → write reaches backing memory
            cortex_m::asm::dsb();
        }
        write_volatile(M.offset(M_DIRTY), 1);
        while read_volatile(M.offset(M_GO2)) == 0 {
            cortex_m::asm::nop();
        }
        write_volatile(M.offset(M_DONE2), DONE);
    }

    loop {
        cortex_m::asm::wfi();
    }
}
