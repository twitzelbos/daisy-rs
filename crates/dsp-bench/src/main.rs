#![no_std]
#![no_main]

//! DWT cycle-bench for the `daisy-dsp` processors.
//!
//! Measures **cycles per 64-sample block** for each processor with the
//! Cortex-M7's DWT CYCCNT, and writes the results to a fixed DTCM array that a
//! Renode test (or `probe-rs` on hardware) reads back.
//!
//! # What each tier's number means
//! - **Hardware (authoritative):** `probe-rs` reads the results array → the real
//!   cycles/block. This is the compute budget. See the crate README for the read
//!   command and the %CPU maths.
//! - **Renode (functional smoke ONLY):** the cycle numbers here are NOT a
//!   trustworthy budget — Renode is a functional translator, so CYCCNT advances
//!   with virtual time (≈ instruction count), not the real M7 pipeline/cache/FPU
//!   latencies (see the `feedback_renode_timing_fidelity` note and the
//!   `STM32H7_DWT_Clocked` "Fidelity boundary" header). The Renode test asserts
//!   only that each processor **executed on the M7 ISA without faulting** (marker
//!   non-zero) and that the firmware reached its done sentinel.
//!
//! # Operating point
//! Runs on the **reset clock** (HSI 64 MHz, internal-flash 0 wait states, no
//! cache) — like the other exercisers, no fragile PLL/cache bring-up. Cycle
//! *counts* are a core-bound property of the code and are a good first-order
//! budget; the definitive 480 MHz + I/D-cache + memory-placement number is a
//! follow-up that adds the full operating-point bring-up.

use core::hint::black_box;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};

use cortex_m_rt::entry;
use panic_halt as _;

use daisy_dsp::delay::DelayLine;
use daisy_dsp::env::Env;
use daisy_dsp::filter::{Biquad, OnePole};
use daisy_dsp::freeze::Freeze;
use daisy_dsp::pad::PadDrone;
use daisy_dsp::reverb::FdnReverb;
use daisy_dsp::DspProcessor;

const SR: f32 = 48_000.0;
const BLOCK: usize = 64; // = daisy_dsp::MAX_BLOCK
const ITERS: u32 = 64; // repeats per measurement; we keep the minimum

// --- Core debug / DWT registers (ARMv7-M ARM §C1.8; raw MMIO like the other
//     exercisers) -----------------------------------------------------------
const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32; // TRCENA @ bit 24
const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32; // CYCCNTENA @ bit 0
const DWT_CYCCNT: *mut u32 = 0xE000_1004 as *mut u32;

// --- Results array in DTCM (the Renode test / probe-rs read these back) ------
// Placed 32 KiB below the stack top (0x2001_8000, stack starts at 0x2002_0000)
// so the large DSP objects on the stack — PadDrone embeds a Granular whose Hann
// table is ~4 KiB by value — can't grow down into the results.
const RESULTS: *mut u32 = 0x2001_8000 as *mut u32;
// Indices:
const R_RESET: isize = 0; // 1 = main reached
const R_BLOCK: isize = 1; // block size (samples)
const R_ITERS: isize = 2; // measurement repeats
const R_OVERHEAD: isize = 3; // measurement bracket overhead (cycles)
const R_ONEPOLE: isize = 4;
const R_BIQUAD: isize = 5;
const R_DELAY: isize = 6;
const R_FDNREVERB: isize = 7;
const R_FREEZE: isize = 8;
const R_PADDRONE: isize = 9;
const R_ENV: isize = 10;
const R_STAGES: isize = 11; // bitmask: one bit set as each bench completes
const R_DONE: isize = 12; // 0xD09E = all benches ran
const DONE_SENTINEL: u32 = 0x0000_D09E;

// Stage bits (OR'd into R_STAGES as each processor finishes — a timing-
// independent proof of execution, so the Renode functional smoke doesn't
// depend on CYCCNT values). All seven = ALL_STAGES.
const S_ONEPOLE: u32 = 1 << 0;
const S_BIQUAD: u32 = 1 << 1;
const S_DELAY: u32 = 1 << 2;
const S_ENV: u32 = 1 << 3;
const S_FDNREVERB: u32 = 1 << 4;
const S_FREEZE: u32 = 1 << 5;
const S_PADDRONE: u32 = 1 << 6; // all seven set = 0x7F (asserted by the Renode test)

// --- DSP scratch buffers (DTCM .bss) -----------------------------------------
// The benches run sequentially, so the capture and reverb buffers are SHARED:
// FdnReverb + PadDrone share the reverb buffer; Freeze + PadDrone share the
// capture buffer. Keeps the whole working set in DTCM (fits 128 KiB).
const DELAY_LEN: usize = 4800; // 100 ms @ 48 kHz
const CAP_LEN: usize = 8192; // freeze / pad capture loop

static mut IN_A: [f32; BLOCK] = [0.0; BLOCK];
static mut IN_B: [f32; BLOCK] = [0.0; BLOCK];
static mut OUT_L: [f32; BLOCK] = [0.0; BLOCK];
static mut OUT_R: [f32; BLOCK] = [0.0; BLOCK];

static mut DELAY_BUF: [f32; DELAY_LEN] = [0.0; DELAY_LEN];
static mut SHARED_REV: [f32; FdnReverb::REQUIRED_BUF] = [0.0; FdnReverb::REQUIRED_BUF];
static mut SHARED_CAP: [f32; CAP_LEN] = [0.0; CAP_LEN];
// PadDrone now holds a granular engine too (power-of-two ring).
const PAD_GRAN_LEN: usize = 4096;
static mut PAD_GRAN: [f32; PAD_GRAN_LEN] = [0.0; PAD_GRAN_LEN];

#[inline(always)]
fn cyccnt() -> u32 {
    unsafe { read_volatile(DWT_CYCCNT) }
}

#[inline(always)]
unsafe fn put(index: isize, value: u32) {
    write_volatile(RESULTS.offset(index), value);
}

/// Run `f` `ITERS` times, each bracketed by CYCCNT, and return the **minimum**
/// delta — the least-noise estimate of the hot-path cost.
#[inline(never)]
fn measure(mut f: impl FnMut()) -> u32 {
    let mut best = u32::MAX;
    for _ in 0..ITERS {
        let s = cyccnt();
        f();
        let e = cyccnt();
        let d = e.wrapping_sub(s);
        if d < best {
            best = d;
        }
    }
    best
}

#[entry]
fn main() -> ! {
    unsafe {
        for i in 0..=R_DONE {
            put(i, 0);
        }
        put(R_RESET, 1);

        // Enable the cycle counter: DEMCR.TRCENA then DWT_CTRL.CYCCNTENA.
        write_volatile(DEMCR, read_volatile(DEMCR) | (1 << 24));
        write_volatile(DWT_CTRL, read_volatile(DWT_CTRL) | 1);
        write_volatile(DWT_CYCCNT, 0);

        // Take &mut to the AXI buffers (single-threaded, one entry point).
        let in_a = &mut *addr_of_mut!(IN_A);
        let in_b = &mut *addr_of_mut!(IN_B);
        let out_l = &mut *addr_of_mut!(OUT_L);
        let out_r = &mut *addr_of_mut!(OUT_R);

        // A modest sine so the filters/reverb do real (non-denormal) work.
        for i in 0..BLOCK {
            let s = libm::sinf(core::f32::consts::TAU * 220.0 * i as f32 / SR) * 0.25;
            in_a[i] = s;
            in_b[i] = s;
        }

        put(R_BLOCK, BLOCK as u32);
        put(R_ITERS, ITERS);

        // Measurement-bracket overhead (two CYCCNT reads); subtract from each.
        let overhead = measure(|| {
            black_box(());
        });
        put(R_OVERHEAD, overhead);
        let net = |c: u32| c.saturating_sub(overhead);
        let mut stages = 0u32;

        // --- OnePole (block) ---
        {
            let mut f = OnePole::lowpass(SR, 1_000.0);
            let c = measure(|| {
                f.process(black_box(&in_a[..]), &mut out_l[..]);
                black_box(&out_l[..]);
            });
            put(R_ONEPOLE, net(c));
            stages |= S_ONEPOLE;
            put(R_STAGES, stages);
        }

        // --- Biquad (block) ---
        {
            let mut f = Biquad::lowpass(SR, 1_000.0, 0.707);
            let c = measure(|| {
                f.process(black_box(&in_a[..]), &mut out_l[..]);
                black_box(&out_l[..]);
            });
            put(R_BIQUAD, net(c));
            stages |= S_BIQUAD;
            put(R_STAGES, stages);
        }

        // --- DelayLine (fractional read, per sample) ---
        {
            let delay_buf = &mut *addr_of_mut!(DELAY_BUF);
            let mut dl = DelayLine::new(delay_buf);
            let c = measure(|| {
                for i in 0..BLOCK {
                    dl.write(black_box(in_a[i]));
                    out_l[i] = dl.read_frac(black_box(1234.5));
                }
                black_box(&out_l[..]);
            });
            put(R_DELAY, net(c));
            stages |= S_DELAY;
            put(R_STAGES, stages);
        }

        // --- Env (per sample) ---
        {
            let mut env = Env::new(SR, 1.0, 1.0);
            env.gate(true);
            let c = measure(|| {
                for o in out_l.iter_mut().take(BLOCK) {
                    *o = env.tick();
                }
                black_box(&out_l[..]);
            });
            put(R_ENV, net(c));
            stages |= S_ENV;
            put(R_STAGES, stages);
        }

        // --- FdnReverb (block, mono in / stereo out) ---
        {
            let rev_buf = &mut *addr_of_mut!(SHARED_REV);
            let mut rv = FdnReverb::new(rev_buf, SR, 2.0, 10_000.0, 0.5);
            // Warm the tail so the loop reads real state, not silence.
            for _ in 0..8 {
                rv.process(&in_a[..], &mut out_l[..], &mut out_r[..]);
            }
            let c = measure(|| {
                rv.process(black_box(&in_a[..]), &mut out_l[..], &mut out_r[..]);
                black_box((&out_l[..], &out_r[..]));
            });
            put(R_FDNREVERB, net(c));
            stages |= S_FDNREVERB;
            put(R_STAGES, stages);
        }

        // --- Freeze (frozen steady-state: per-sample loop read) ---
        {
            let freeze_buf = &mut *addr_of_mut!(SHARED_CAP);
            let mut fz = Freeze::new(freeze_buf, 256);
            // Record a full buffer, then freeze → measure the loop-read cost.
            for i in 0..CAP_LEN {
                fz.tick(in_a[i % BLOCK]);
            }
            fz.freeze();
            let c = measure(|| {
                for o in out_l.iter_mut().take(BLOCK) {
                    *o = fz.tick(0.0);
                }
                black_box(&out_l[..]);
            });
            put(R_FREEZE, net(c));
            stages |= S_FREEZE;
            put(R_STAGES, stages);
        }

        // --- PadDrone (block, frozen — the heavy path: freeze + reverb) ---
        {
            let pad_cap = &mut *addr_of_mut!(SHARED_CAP);
            let pad_gran = &mut *addr_of_mut!(PAD_GRAN);
            let pad_rev = &mut *addr_of_mut!(SHARED_REV);
            // Benched in the default Freeze mode (the freeze + reverb heavy path).
            let mut pad =
                PadDrone::new(pad_cap, pad_gran, pad_rev, SR, 2.0, 10_000.0, 0.05, 256, 1);
            // Prime the capture, freeze, warm the reverb tail.
            for _ in 0..(CAP_LEN / BLOCK) {
                pad.process(&in_a[..], &in_b[..], &mut out_l[..], &mut out_r[..]);
            }
            pad.set_freeze(true);
            for _ in 0..8 {
                pad.process(&in_a[..], &in_b[..], &mut out_l[..], &mut out_r[..]);
            }
            let c = measure(|| {
                pad.process(
                    black_box(&in_a[..]),
                    black_box(&in_b[..]),
                    &mut out_l[..],
                    &mut out_r[..],
                );
                black_box((&out_l[..], &out_r[..]));
            });
            put(R_PADDRONE, net(c));
            stages |= S_PADDRONE;
            put(R_STAGES, stages);
        }

        put(R_DONE, DONE_SENTINEL);
    }

    loop {
        cortex_m::asm::wfi();
    }
}
