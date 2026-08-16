#![no_std]
#![no_main]

//! DMA-driven D-cache / DMA coherency exerciser — the **hardware** counterpart
//! to `cache-coherency-exerciser`.
//!
//! Why a second exerciser?
//! -----------------------
//! `cache-coherency-exerciser` validates Renode's `CacheCoherencyChecker` by
//! having the *harness* (a Renode monitor write) play the foreign master. On
//! silicon there is no harness, and — as `docs/hardware-tests-2026-08-12.md` §6
//! established — a **debugger cannot stand in for the foreign master**: the
//! SWD/AHBS port is cache-coherent, so a probe write behind a CPU-cached line is
//! snooped and never goes stale. The only real foreign master an isolated board
//! has is an **on-chip DMA engine on the AXI/AHB matrix**, which is NOT coherent
//! with the M7 D-cache (PM0253 / RM0433 — no ACE). This firmware kicks a
//! **DMA1 memory-to-memory** transfer itself to be that master, so the hazard is
//! reproducible on hardware with nothing but a probe reading DTCM afterward.
//!
//! It runs, back-to-back with no harness handshake, the **buggy** (no cache
//! maintenance) and **correct** (maintenance by MVA) variant of both hazards,
//! recording each read-back value to a fixed DTCM marker:
//!
//!   Phase 1 — stale READ (DMA writes, CPU reads):
//!     BUF is cached holding P1_OLD; DMA overwrites backing with P1_NEW.
//!       buggy   → CPU re-reads the STALE cached P1_OLD          (M_STALE_BUGGY)
//!       correct → DCIMVAC first, CPU re-reads the fresh P1_NEW  (M_STALE_CORRECT)
//!
//!   Phase 2 — stale DMA read of a DIRTY line (CPU writes, DMA reads):
//!     backing = P2_BASE; CPU writes P2_DIRTY (dirty in write-back cache).
//!       buggy   → DMA reads STALE backing P2_BASE into SINK     (M_DIRTY_BUGGY)
//!       correct → DCCMVAC first, DMA reads P2_DIRTY into SINK   (M_DIRTY_CORRECT)
//!
//! On silicon: `M_STALE_BUGGY != M_STALE_CORRECT` and
//! `M_DIRTY_BUGGY != M_DIRTY_CORRECT` — the buggy value is the stale one, proving
//! the hazard AND that the maintenance op fixes it.
//!
//! In Renode (no cache model, and a firmware-kicked DMA copy runs in the CPU's
//! context so the checker can't classify it foreign) both variants read the
//! FRESH value — so the sim run only proves the **DMA programming itself works**
//! (the copy happens, TCIF sets, markers reach DONE). That is the documented
//! fidelity boundary; the coherency divergence is hardware-only, which is the
//! whole reason this firmware exists. The sim coherency proof stays with
//! `cache-coherency-exerciser` + `CacheCoherencyChecker`.
//!
//! SCB cache maintenance uses `cortex_m`'s by-address ops, which emit the DSB/ISB
//! the M7 requires between a maintenance op and the following access (ES0392
//! §2.1.5 — store-after-invalidate needs an intervening barrier).
//!
//! Runs on the reset clock (HSI, 64 MHz) — no PLL/RCC clock tree needed; DMA1
//! only needs its AHB1 bus-clock gate.

use core::ptr::{read_volatile, write_volatile};

use cortex_m_rt::entry;
use panic_halt as _;

// --- buffers in D2 SRAM (DMA1-reachable, cacheable write-back) ----------------
// One cache line (32 B) each, two lines apart so they never share a line.
const BUF: usize = 0x3000_0000; // the buffer under test (CPU caches it)
const SRC: usize = 0x3000_0040; // DMA source (phase 1)
const SINK: usize = 0x3000_0080; // DMA destination (phase 2)
const LINE_BYTES: usize = 32;
const LINE_WORDS: u32 = (LINE_BYTES / 4) as u32;

// Distinct, recognisable patterns so a stale read is unambiguous in the marker.
const P1_OLD: u32 = 0xBADC_0FFE; // cached before the DMA write
const P1_NEW: u32 = 0xC0FF_EE01; // what the DMA writes into backing
const P2_BASE: u32 = 0x5EED_0000; // backing before the CPU dirties the line
const P2_DIRTY: u32 = 0xD154_7000; // what the CPU writes (dirty in cache)
const P_SANITY: u32 = 0x1111_1111; // plain-copy DMA sanity pattern

// --- DTCM markers (probe-rs reads these; DTCM is never cached) -----------------
const M: *mut u32 = 0x2001_F000 as *mut u32;
const M_DMA_OK: isize = 0; // plain mem2mem sanity copy result (expect P_SANITY)
const M_STALE_BUGGY: isize = 1; // phase 1 buggy   (HW: P1_OLD; sim: P1_NEW)
const M_STALE_CORRECT: isize = 2; // phase 1 correct (HW & sim: P1_NEW)
const M_DIRTY_BUGGY: isize = 3; // phase 2 buggy   (HW: P2_BASE; sim: P2_DIRTY)
const M_DIRTY_CORRECT: isize = 4; // phase 2 correct (HW & sim: P2_DIRTY)
const M_DONE: isize = 5; // 0xD09E once all phases complete
const M_LAST: isize = M_DONE;
const DONE: u32 = 0x0000_D09E;

// --- DMA1 (RM0433 §15) — stream 0, memory-to-memory ---------------------------
const DMA1: usize = 0x4002_0000;
const DMA1_LISR: *const u32 = DMA1 as *const u32;
const DMA1_LIFCR: *mut u32 = (DMA1 + 0x08) as *mut u32;
const DMA1_S0CR: *mut u32 = (DMA1 + 0x10) as *mut u32;
const DMA1_S0NDTR: *mut u32 = (DMA1 + 0x14) as *mut u32;
const DMA1_S0PAR: *mut u32 = (DMA1 + 0x18) as *mut u32; // mem2mem: SOURCE
const DMA1_S0M0AR: *mut u32 = (DMA1 + 0x1C) as *mut u32; // mem2mem: DESTINATION
const TCIF0: u32 = 1 << 5; // LISR transfer-complete, stream 0
const STREAM0_FLAGS: u32 = 0x3D; // CFEIF0|CDMEIF0|CTEIF0|CHTIF0|CTCIF0 (bits 0,2,3,4,5)

// RCC_AHB1ENR.DMA1EN (RM0433 §8.7.38) — DMA1 bus-clock gate.
const RCC_AHB1ENR: *mut u32 = 0x5802_44D8 as *mut u32;
const DMA1EN: u32 = 1 << 0;

/// Program DMA1 stream 0 for a word-granular memory-to-memory copy and block
/// until it completes. In mem2mem mode the peripheral-address port is the source
/// and the memory-address port is the destination (RM0433 §15.3.13); the stream
/// starts as soon as EN is set — no DMAMUX request is involved.
unsafe fn dma_mem2mem(src: usize, dst: usize, nwords: u32) {
    write_volatile(DMA1_S0CR, 0); // disable
    while read_volatile(DMA1_S0CR) & 1 != 0 {}
    write_volatile(DMA1_LIFCR, STREAM0_FLAGS); // clear stale stream-0 flags
    write_volatile(DMA1_S0PAR, src as u32);
    write_volatile(DMA1_S0M0AR, dst as u32);
    write_volatile(DMA1_S0NDTR, nwords);
    // DIR=10 (mem2mem, bits 7:6), MINC (10), PINC (9), PSIZE=word (12:11=10),
    // MSIZE=word (14:13=10).
    let cr = (0b10 << 6) | (1 << 10) | (1 << 9) | (0b10 << 11) | (0b10 << 13);
    write_volatile(DMA1_S0CR, cr);
    write_volatile(DMA1_S0CR, cr | 1); // EN → transfer starts
    while read_volatile(DMA1_LISR) & TCIF0 == 0 {}
    write_volatile(DMA1_LIFCR, TCIF0);
    write_volatile(DMA1_S0CR, 0);
    cortex_m::asm::dsb();
}

/// Fill a line with `val` and clean it to PoC, so the value is in backing memory
/// (what a foreign DMA read sees) and the line is left clean — coherency of this
/// helper buffer is thus NOT a variable in the phase under test.
unsafe fn fill_and_clean(scb: &mut cortex_m::peripheral::SCB, addr: usize, val: u32) {
    for i in 0..LINE_WORDS as usize {
        write_volatile((addr as *mut u32).add(i), val);
    }
    scb.clean_dcache_by_address(addr, LINE_BYTES);
}

/// Read word 0 of a line after invalidating it, so the CPU re-fetches from
/// backing (used for the DMA *destination*, whose coherency is not under test).
unsafe fn invalidate_and_read(scb: &mut cortex_m::peripheral::SCB, addr: usize) -> u32 {
    scb.invalidate_dcache_by_address(addr, LINE_BYTES);
    read_volatile(addr as *const u32)
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();

    unsafe {
        for i in 0..=M_LAST {
            write_volatile(M.offset(i), 0);
        }
        // Ungate DMA1's bus clock (inert in Renode, required on silicon).
        write_volatile(RCC_AHB1ENR, read_volatile(RCC_AHB1ENR) | DMA1EN);
    }

    // Enable the L1 D-cache — without it there is no hazard to reproduce. Under
    // the default memory map 0x3000_0000 is Normal write-back cacheable, so no
    // MPU programming is needed for this test.
    cp.SCB.enable_dcache(&mut cp.CPUID);
    let scb = &mut cp.SCB;

    unsafe {
        // ---- Sanity: a plain mem2mem copy actually works -------------------
        fill_and_clean(scb, SRC, P_SANITY);
        dma_mem2mem(SRC, SINK, LINE_WORDS);
        let ok = invalidate_and_read(scb, SINK);
        write_volatile(M.offset(M_DMA_OK), ok);

        // ---- Phase 1: stale READ (DMA writes, CPU reads) -------------------
        // Buggy: no invalidate before the re-read → stale cached value on HW.
        fill_and_clean(scb, SRC, P1_NEW); // DMA source = P1_NEW in backing
        fill_and_clean(scb, BUF, P1_OLD); // BUF cached + backing = P1_OLD
        let _ = read_volatile(BUF as *const u32); // ensure BUF is cached (P1_OLD)
        dma_mem2mem(SRC, BUF, LINE_WORDS); // foreign write → backing = P1_NEW
        write_volatile(M.offset(M_STALE_BUGGY), read_volatile(BUF as *const u32));

        // Correct: DCIMVAC before the re-read → fresh value.
        fill_and_clean(scb, SRC, P1_NEW);
        fill_and_clean(scb, BUF, P1_OLD);
        let _ = read_volatile(BUF as *const u32);
        dma_mem2mem(SRC, BUF, LINE_WORDS);
        let fresh = invalidate_and_read(scb, BUF);
        write_volatile(M.offset(M_STALE_CORRECT), fresh);

        // ---- Phase 2: stale DMA read of a DIRTY line (CPU writes, DMA reads)-
        // Buggy: no clean → the DMA reads stale backing (P2_BASE) on HW.
        fill_and_clean(scb, BUF, P2_BASE); // backing = P2_BASE, line clean+cached
        write_volatile(BUF as *mut u32, P2_DIRTY); // dirty in write-back cache
        dma_mem2mem(BUF, SINK, LINE_WORDS); // foreign read of BUF → SINK
        write_volatile(M.offset(M_DIRTY_BUGGY), invalidate_and_read(scb, SINK));

        // Correct: DCCMVAC after the write → the DMA reads P2_DIRTY.
        fill_and_clean(scb, BUF, P2_BASE);
        write_volatile(BUF as *mut u32, P2_DIRTY);
        scb.clean_dcache_by_address(BUF, LINE_BYTES); // push the dirty line to backing
        dma_mem2mem(BUF, SINK, LINE_WORDS);
        write_volatile(M.offset(M_DIRTY_CORRECT), invalidate_and_read(scb, SINK));

        write_volatile(M.offset(M_DONE), DONE);
    }

    loop {
        cortex_m::asm::wfi();
    }
}
