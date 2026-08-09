//! Fixed-point (Q15) complex FFT using the ARMv7E-M DSP **SIMD** instructions —
//! the fastest FFT path on the Cortex-M7, which has no floating-point SIMD.
//!
//! # Why this is fast
//!
//! A complex Q15 sample is packed into one `i32` — `re` in the low halfword,
//! `im` in the high halfword — and the packed DSP instructions do complex
//! arithmetic two lanes at once:
//!
//! - **Complex multiply** = `SMUSD` (→ `re·wr − im·wi`, the real part) + `SMUADX`
//!   (→ `re·wi + im·wr`, the imaginary part): each a dual-16 signed MAC.
//! - **Butterfly ± with the per-stage ÷2 scaling** = `SHADD16` / `SHSUB16`:
//!   halving packed add/sub, one instruction each. The ÷2 per stage keeps Q15
//!   from overflowing (radix-2, so the output is the DFT scaled by `1/L`, the
//!   CMSIS `arm_cfft_q15` convention).
//!
//! # Correctness strategy
//!
//! The SIMD ops are ARM-only, so the host f64-DFT oracle can't run the asm.
//! Instead every DSP instruction is one primitive ([`smusd`] etc.) that is inline
//! asm on `target_arch = "arm"` and a **faithful bit-level emulation** elsewhere.
//! The FFT itself is written *once* on those primitives: the host tests validate
//! the algorithm through the emulation (against the naive f64 DFT), and the exact
//! same source runs the real instructions on hardware / in Renode.
//!
//! # References
//! - ARM, "Cortex-M4/M7 DSP instructions" (SMUSD/SMUADX/SHADD16/SHSUB16), ARMv7-M
//!   Architecture Reference Manual (DDI 0403), §A7.
//! - CMSIS-DSP `arm_cfft_q15` — the Q15 radix scaling convention (1/N output).

use crate::twiddles::{TW_MASTER, TW_Q15};

// --- packed complex Q15 helpers ----------------------------------------------

/// Pack a complex Q15 into `[im:i16 high | re:i16 low]`.
#[inline(always)]
fn pack(re: i16, im: i16) -> i32 {
    ((im as i32) << 16) | ((re as i32) & 0xffff)
}

// Half-word extractors — used only by the emulated primitives (off-ARM) and the
// tests; on ARM the DSP instructions do the extraction internally.
/// Low halfword (real), sign-extended.
#[cfg(any(not(target_arch = "arm"), test))]
#[inline(always)]
fn lo(x: i32) -> i32 {
    (x as i16) as i32
}

/// High halfword (imag), sign-extended.
#[cfg(any(not(target_arch = "arm"), test))]
#[inline(always)]
fn hi(x: i32) -> i32 {
    x >> 16
}

/// Saturate a Q15-range integer to `i16`.
#[inline(always)]
fn sat16(v: i32) -> i16 {
    v.clamp(-32768, 32767) as i16
}

// --- DSP-instruction primitives (asm on ARMv7E-M; emulated elsewhere) --------
// The emulation is the exact integer function each instruction computes, so the
// host tests exercise the real bit-level logic — only the encoding differs on HW.

/// `SMUSD`: `Rn[15:0]·Rm[15:0] − Rn[31:16]·Rm[31:16]` (Q30).
#[inline(always)]
fn smusd(a: i32, b: i32) -> i32 {
    #[cfg(target_arch = "arm")]
    {
        let r: i32;
        unsafe {
            core::arch::asm!("smusd {0}, {1}, {2}", out(reg) r, in(reg) a, in(reg) b,
                options(pure, nomem, nostack));
        }
        r
    }
    #[cfg(not(target_arch = "arm"))]
    {
        lo(a) * lo(b) - hi(a) * hi(b)
    }
}

/// `SMUADX`: `Rn[15:0]·Rm[31:16] + Rn[31:16]·Rm[15:0]` (Q30).
#[inline(always)]
fn smuadx(a: i32, b: i32) -> i32 {
    #[cfg(target_arch = "arm")]
    {
        let r: i32;
        unsafe {
            core::arch::asm!("smuadx {0}, {1}, {2}", out(reg) r, in(reg) a, in(reg) b,
                options(pure, nomem, nostack));
        }
        r
    }
    #[cfg(not(target_arch = "arm"))]
    {
        lo(a) * hi(b) + hi(a) * lo(b)
    }
}

/// `SHADD16`: per-halfword signed halving add, `(a+b)>>1`.
#[inline(always)]
fn shadd16(a: i32, b: i32) -> i32 {
    #[cfg(target_arch = "arm")]
    {
        let r: i32;
        unsafe {
            core::arch::asm!("shadd16 {0}, {1}, {2}", out(reg) r, in(reg) a, in(reg) b,
                options(pure, nomem, nostack));
        }
        r
    }
    #[cfg(not(target_arch = "arm"))]
    {
        pack(((lo(a) + lo(b)) >> 1) as i16, ((hi(a) + hi(b)) >> 1) as i16)
    }
}

/// `SHSUB16`: per-halfword signed halving subtract, `(a−b)>>1`.
#[inline(always)]
fn shsub16(a: i32, b: i32) -> i32 {
    #[cfg(target_arch = "arm")]
    {
        let r: i32;
        unsafe {
            core::arch::asm!("shsub16 {0}, {1}, {2}", out(reg) r, in(reg) a, in(reg) b,
                options(pure, nomem, nostack));
        }
        r
    }
    #[cfg(not(target_arch = "arm"))]
    {
        pack(((lo(a) - lo(b)) >> 1) as i16, ((hi(a) - hi(b)) >> 1) as i16)
    }
}

/// Complex Q15 multiply `x·w` (both packed), result packed Q15 (saturated).
#[inline(always)]
fn cmul(x: i32, w: i32) -> i32 {
    pack(sat16(smusd(x, w) >> 15), sat16(smuadx(x, w) >> 15))
}

/// `W_m^k = exp(-j·2π·k/m)`, packed Q15, for any `m | TW_MASTER`.
#[inline(always)]
fn tw(k: usize, stride: usize) -> i32 {
    TW_Q15[(k * stride) & (TW_MASTER - 1)]
}

/// In-place radix-2 DIT complex FFT over packed Q15 (`re` low, `im` high). The
/// output is the DFT scaled by `1/L` (a ÷2 per stage prevents Q15 overflow — the
/// CMSIS `arm_cfft_q15` convention). `x.len()` must be a power of two ≤ 1024.
pub fn cfft_q15(x: &mut [i32]) {
    let l = x.len();
    assert!(l.is_power_of_two(), "Q15 FFT length must be a power of two");
    assert!(l <= TW_MASTER, "Q15 FFT length exceeds twiddle table");
    if l <= 1 {
        return;
    }

    bit_reverse(x);

    let mut m = 2usize;
    while m <= l {
        let half = m / 2;
        let stride = TW_MASTER / m;
        let mut base = 0;
        while base < l {
            for k in 0..half {
                let i = base + k;
                let j = i + half;
                // SAFETY: j = base + half + k < base + m ≤ l = x.len().
                unsafe {
                    let w = tw(k, stride);
                    let t = cmul(*x.get_unchecked(j), w);
                    let a = *x.get_unchecked(i);
                    // (a ± t)/2 in one packed halving op each.
                    *x.get_unchecked_mut(i) = shadd16(a, t);
                    *x.get_unchecked_mut(j) = shsub16(a, t);
                }
            }
            base += m;
        }
        m *= 2;
    }
}

/// Standard base-2 bit-reversal permutation over the packed array.
fn bit_reverse(x: &mut [i32]) {
    let n = x.len();
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            x.swap(i, j);
        }
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn q15f(v: f64) -> i16 {
        (v * 32768.0).round().clamp(-32768.0, 32767.0) as i16
    }

    // Naive O(N^2) complex DFT of a dequantized packed-Q15 input, in f64.
    fn naive_dft(x: &[i32]) -> (Vec<f64>, Vec<f64>) {
        let n = x.len();
        let re: Vec<f64> = x.iter().map(|&v| lo(v) as f64 / 32768.0).collect();
        let im: Vec<f64> = x.iter().map(|&v| hi(v) as f64 / 32768.0).collect();
        let (mut or, mut oi) = (std::vec![0.0; n], std::vec![0.0; n]);
        for k in 0..n {
            let (mut sr, mut si) = (0.0, 0.0);
            for t in 0..n {
                let a = -2.0 * std::f64::consts::PI * (k * t) as f64 / n as f64;
                let (c, s) = (a.cos(), a.sin());
                sr += re[t] * c - im[t] * s;
                si += re[t] * s + im[t] * c;
            }
            or[k] = sr;
            oi[k] = si;
        }
        (or, oi)
    }

    fn lcg(seed: &mut u64) -> f64 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*seed >> 40) as f64 / (1u64 << 24) as f64) * 2.0 - 1.0
    }

    #[test]
    fn cfft_q15_matches_naive_dft() {
        for &l in &[128usize, 256, 512, 1024] {
            let mut seed = 0x51_5a5a ^ l as u64;
            // Keep inputs at half scale so the twiddle products keep headroom.
            let x: Vec<i32> = (0..l)
                .map(|_| pack(q15f(lcg(&mut seed) * 0.5), q15f(lcg(&mut seed) * 0.5)))
                .collect();
            let (gr, gi) = naive_dft(&x);

            let mut y = x.clone();
            cfft_q15(&mut y);

            // Our output is the DFT scaled by 1/L; compare against gr/L, gi/L.
            let inv_l = 1.0 / l as f64;
            let mut max_err = 0.0f64;
            for k in 0..l {
                let yr = lo(y[k]) as f64 / 32768.0;
                let yi = hi(y[k]) as f64 / 32768.0;
                max_err = max_err.max((yr - gr[k] * inv_l).abs());
                max_err = max_err.max((yi - gi[k] * inv_l).abs());
            }
            // Q15 radix-2 with per-stage ÷2 + twiddle quantization over log2(L)
            // stages: a few ×10^-3 on the 1/L-scaled bins is expected.
            assert!(max_err < 6e-3, "L={l} Q15 FFT max err {max_err}");
        }
    }
}
