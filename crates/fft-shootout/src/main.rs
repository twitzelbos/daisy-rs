#![no_std]
#![no_main]

//! Head-to-head DWT cycle bench for competing real-FFT implementations.
//!
//! Measures **cycles per forward real transform** for each implementation ×
//! size with the Cortex-M7 CYCCNT, writing results to a fixed DTCM array that a
//! Renode test (functional smoke only) or `probe-rs` (the authoritative ranking)
//! reads back.
//!
//! # What each tier's number means
//! - **Hardware (authoritative):** `probe-rs` reads the array → real
//!   cycles/transform. THIS is the ranking. See the README for the read command.
//! - **Renode (functional smoke ONLY):** Renode is a functional translator —
//!   CYCCNT advances with virtual time (≈ instruction count), NOT the real M7
//!   pipeline/cache/FPU latencies (see `feedback_renode_timing_fidelity`). So the
//!   Renode values are NOT a ranking; the robot only asserts every entrant ran to
//!   completion without faulting (stage bitmask) and CYCCNT advanced.
//!
//! # Operating point
//! Reset clock (HSI 64 MHz, 0 wait states, no cache), all scratch in DTCM — the
//! cleanest core-bound baseline. The 480 MHz + I/D-cache + ITCM/DTCM-placement
//! number is a follow-up that adds the full operating-point bring-up.
//!
//! # Entrants (all-Rust)
//! - `daisy_dsp::fft` — our mixed radix-4/2 (radix-2²), const-N specialized.
//! - `microfft` — radix-2, the Rust no_std reference.
//!
//! radix-4, Stockham, and the Q15 SIMD kernel slot in as they land.

use core::convert::TryInto;
use core::hint::black_box;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};

use cortex_m_rt::entry;
use panic_halt as _;

use daisy_dsp::fft::{cfft_n, RealFft};
use daisy_dsp::fft_q15::cfft_q15;

const SR: f32 = 48_000.0;
const ITERS: u32 = 32; // repeats per measurement; we keep the minimum

// --- DWT (ARMv7-M ARM §C1.8) --------------------------------------------------
const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32; // TRCENA @ bit 24
const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32; // CYCCNTENA @ bit 0
const DWT_CYCCNT: *mut u32 = 0xE000_1004 as *mut u32;

// --- Results array in DTCM (Renode test / probe-rs read these back) ----------
const RESULTS: *mut u32 = 0x2001_F000 as *mut u32;
const R_RESET: isize = 0; // 1 = main reached
const R_STAGES: isize = 1; // bitmask, one bit per completed bench
const R_ITERS: isize = 2;
const R_OVERHEAD: isize = 3; // measurement-bracket cost (subtracted from each)
                             // Cycles/transform, grouped by kind then size (256/512/1024/2048):
const R_MINE: isize = 4; // f32 REAL forward (ours), ..7
const R_MFFT: isize = 8; // microfft REAL forward, ..11
const R_MINE_C: isize = 12; // f32 COMPLEX (ours, cfft_n), ..15
const R_Q15_C: isize = 16; // Q15 COMPLEX (cfft_q15, DSP SIMD), ..19
const R_Q15_OK: isize = 20; // on-device Q15 correctness self-check
const R_LAST: isize = 20;
const R_DONE: isize = 24; // 0xD09E = all benches ran (clear of the value slots)
const DONE_SENTINEL: u32 = 0x0000_D09E;
const Q15_OK_SENTINEL: u32 = 0x00C0_FFEE;

// --- DTCM scratch (max N = 2048; the complex entrants use L up to 2048) ------
static mut IN: [f32; 2048] = [0.0; 2048];
static mut SRE: [f32; 2048] = [0.0; 2048];
static mut SIM: [f32; 2048] = [0.0; 2048];
static mut OUTR: [f32; 1025] = [0.0; 1025];
static mut OUTI: [f32; 1025] = [0.0; 1025];
static mut Q15BUF: [i32; 2048] = [0; 2048];

#[inline(always)]
fn cyccnt() -> u32 {
    unsafe { read_volatile(DWT_CYCCNT) }
}

#[inline(always)]
unsafe fn put(index: isize, value: u32) {
    write_volatile(RESULTS.offset(index), value);
}

/// Run `f` `ITERS` times, bracketed by CYCCNT; return the **minimum** delta.
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
        put(R_ITERS, ITERS);

        write_volatile(DEMCR, read_volatile(DEMCR) | (1 << 24));
        write_volatile(DWT_CTRL, read_volatile(DWT_CTRL) | 1);
        write_volatile(DWT_CYCCNT, 0);

        let inbuf = &mut *addr_of_mut!(IN);
        let sre = &mut *addr_of_mut!(SRE);
        let sim = &mut *addr_of_mut!(SIM);
        let outr = &mut *addr_of_mut!(OUTR);
        let outi = &mut *addr_of_mut!(OUTI);
        let q15buf = &mut *addr_of_mut!(Q15BUF);
        for (i, s) in inbuf.iter_mut().enumerate() {
            let v = libm::sinf(core::f32::consts::TAU * 440.0 * i as f32 / SR) * 0.5;
            *s = v;
            // Packed complex Q15 (re == im == v) for the fixed-point entrant.
            let q = (v * 32768.0) as i16 as i32;
            q15buf[i] = (q << 16) | (q & 0xffff);
        }

        let overhead = measure(|| {
            black_box(());
        });
        put(R_OVERHEAD, overhead);
        let net = |c: u32| c.saturating_sub(overhead);
        let mut stages = 0u32;
        let mut bit = 0u32;
        macro_rules! done {
            ($ridx:expr, $c:expr) => {{
                put($ridx, net($c));
                stages |= 1 << bit;
                bit += 1;
                put(R_STAGES, stages);
            }};
        }

        // --- Entrant 1: ours (const-N specialized real FFT) ------------------
        // ours does NOT modify `inbuf` (it reads into scratch), so it runs first
        // and leaves the buffer intact for the in-place entrants below.
        macro_rules! bench_mine {
            ($n:literal, $off:expr) => {{
                const N: usize = $n;
                const H: usize = N / 2;
                let c = measure(|| {
                    let mut f = RealFft::new(&mut sre[..H], &mut sim[..H]);
                    f.forward_n::<N>(&inbuf[..N], &mut outr[..H + 1], &mut outi[..H + 1]);
                    black_box((&outr[..H + 1], &outi[..H + 1]));
                });
                done!(R_MINE + $off, c);
            }};
        }
        bench_mine!(256, 0);
        bench_mine!(512, 1);
        bench_mine!(1024, 2);
        bench_mine!(2048, 3);

        // --- Entrant 2: microfft (radix-2, in-place) -------------------------
        macro_rules! bench_mfft {
            ($fn:ident, $n:literal, $off:expr) => {{
                let c = measure(|| {
                    let a: &mut [f32; $n] = (&mut inbuf[..$n]).try_into().unwrap();
                    let _ = microfft::real::$fn(a);
                });
                done!(R_MFFT + $off, c);
            }};
        }
        bench_mfft!(rfft_256, 256, 0);
        bench_mfft!(rfft_512, 512, 1);
        bench_mfft!(rfft_1024, 1024, 2);
        bench_mfft!(rfft_2048, 2048, 3);

        // --- Entrant 3: ours, f32 COMPLEX (cfft_n) — the apples-to-apples base
        // for the Q15 SIMD comparison (same complex transform, f32 vs fixed).
        macro_rules! bench_mine_c {
            ($n:literal, $off:expr) => {{
                let c = measure(|| {
                    cfft_n::<$n>(&mut sre[..$n], &mut sim[..$n], black_box(false));
                    black_box((&sre[..$n], &sim[..$n]));
                });
                done!(R_MINE_C + $off, c);
            }};
        }
        bench_mine_c!(256, 0);
        bench_mine_c!(512, 1);
        bench_mine_c!(1024, 2);
        bench_mine_c!(2048, 3);

        // --- Entrant 4: Q15 COMPLEX (cfft_q15) — the DSP-SIMD fixed-point path.
        macro_rules! bench_q15_c {
            ($n:literal, $off:expr) => {{
                let c = measure(|| {
                    cfft_q15(&mut q15buf[..$n]);
                    black_box(&q15buf[..$n]);
                });
                done!(R_Q15_C + $off, c);
            }};
        }
        bench_q15_c!(256, 0);
        bench_q15_c!(512, 1);
        bench_q15_c!(1024, 2);
        bench_q15_c!(2048, 3);

        // On-device correctness self-check for the Q15 DSP asm (the host oracle
        // can't run the asm): a DC input must put all energy in bin 0. If the
        // SMUSD/SMUADX/SHADD16/SHSUB16 path were wrong the spectrum would be
        // garbage and this would not hold.
        for v in q15buf[..64].iter_mut() {
            *v = 0x0000_4000; // re = 0.5 (Q15), im = 0
        }
        cfft_q15(&mut q15buf[..64]);
        let bin0_re = q15buf[0] as i16 as i32;
        let bin1_re = q15buf[1] as i16 as i32;
        let bin1_im = (q15buf[1] >> 16) as i16 as i32;
        if bin0_re > 8000 && bin1_re.abs() < 1000 && bin1_im.abs() < 1000 {
            put(R_Q15_OK, Q15_OK_SENTINEL);
        }

        let _ = (R_LAST, bit);
        put(R_DONE, DONE_SENTINEL);
    }

    loop {
        cortex_m::asm::wfi();
    }
}
