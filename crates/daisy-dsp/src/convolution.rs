//! Partitioned FFT convolution — the engine for **convolution reverb** (load an
//! impulse response) and the spatializer's HRIR path. `no_std`, no alloc (one
//! scratch region borrowed from the caller).
//!
//! Uniformly-partitioned **overlap-save** (UPOLS): the impulse response is split
//! into `P` partitions of `B` taps, each pre-transformed to a length-`2B`
//! spectrum. Each input block of `B` samples is transformed once; the output is
//! the sum over partitions of the (block-delayed) input spectra times the
//! partition spectra, inverse-transformed — a frequency-domain FIR delay line.
//! Cost is `O(log B)` per sample regardless of IR length, versus `O(L)` direct.
//!
//! Latency is one block (`B` samples). Drive it `B` samples at a time.
//!
//! # References
//! - W. Gardner, "Efficient Convolution without Input-Output Delay," *J. Audio
//!   Eng. Soc.* 43(3), 1995. (partitioned convolution)
//! - Overlap-save method: A. Oppenheim & R. Schafer, *Discrete-Time Signal
//!   Processing*, §8.7.

use crate::fft::RealFft;

/// A uniformly-partitioned overlap-save FFT convolver.
pub struct Convolver<'a> {
    b: usize, // partition / block size
    n: usize, // FFT size = 2·b
    s: usize, // spectrum length = n/2 + 1
    p: usize, // partitions
    head: usize,
    ir_re: &'a mut [f32], // P·S, precomputed IR partition spectra
    ir_im: &'a mut [f32],
    xh_re: &'a mut [f32], // P·S, ring of past input spectra
    xh_im: &'a mut [f32],
    win: &'a mut [f32],  // N, sliding input window ([old B | new B])
    work: &'a mut [f32], // 2N + 2S scratch (fft + acc + ifft)
}

impl<'a> Convolver<'a> {
    /// Scratch length [`new`](Self::new) needs for an IR of `ir_len` taps and
    /// partition/block size `b` (a power of two).
    pub fn required_scratch(ir_len: usize, b: usize) -> usize {
        let n = 2 * b;
        let s = n / 2 + 1;
        let p = ir_len.div_ceil(b).max(1);
        4 * p * s + n + (2 * n + 2 * s)
    }

    /// Build a convolver for `ir`, partition size `b` (power of two). `scratch`
    /// must be at least [`required_scratch`](Self::required_scratch) long.
    pub fn new(ir: &[f32], b: usize, scratch: &'a mut [f32]) -> Self {
        assert!(
            b.is_power_of_two() && b >= 8,
            "partition size must be a power of two >= 8"
        );
        let n = 2 * b;
        let s = n / 2 + 1;
        let p = ir.len().div_ceil(b).max(1);
        assert!(
            scratch.len() >= Self::required_scratch(ir.len(), b),
            "convolver scratch too small"
        );

        let (ir_re, rest) = scratch.split_at_mut(p * s);
        let (ir_im, rest) = rest.split_at_mut(p * s);
        let (xh_re, rest) = rest.split_at_mut(p * s);
        let (xh_im, rest) = rest.split_at_mut(p * s);
        let (win, work) = rest.split_at_mut(n);

        for v in ir_re.iter_mut() {
            *v = 0.0;
        }
        for v in ir_im.iter_mut() {
            *v = 0.0;
        }
        for v in xh_re.iter_mut() {
            *v = 0.0;
        }
        for v in xh_im.iter_mut() {
            *v = 0.0;
        }
        for v in win.iter_mut() {
            *v = 0.0;
        }

        // Pre-transform each IR partition: B taps in the first half of an N-frame,
        // zero-padded, forward-FFT into ir_re/ir_im[p·S..].
        {
            let (frame, rest) = work.split_at_mut(n);
            let (sre, rest) = rest.split_at_mut(n / 2);
            let (sim, _) = rest.split_at_mut(n / 2);
            for pp in 0..p {
                for (i, f) in frame.iter_mut().enumerate() {
                    let tap = pp * b + i;
                    *f = if i < b && tap < ir.len() {
                        ir[tap]
                    } else {
                        0.0
                    };
                }
                RealFft::new(sre, sim).forward(
                    frame,
                    &mut ir_re[pp * s..pp * s + s],
                    &mut ir_im[pp * s..pp * s + s],
                );
            }
        }

        Self {
            b,
            n,
            s,
            p,
            head: 0,
            ir_re,
            ir_im,
            xh_re,
            xh_im,
            win,
            work,
        }
    }

    /// Convolve one block: `in_block`/`out_block` are both `b` samples.
    pub fn process_block(&mut self, in_block: &[f32], out_block: &mut [f32]) {
        let (b, n, s, p) = (self.b, self.n, self.s, self.p);
        debug_assert_eq!(in_block.len(), b);
        debug_assert_eq!(out_block.len(), b);

        // Slide the window and append the new block ([old B | new B]).
        self.win.copy_within(b.., 0);
        self.win[b..].copy_from_slice(in_block);

        // Forward-FFT the window straight into the ring slot at `head`.
        let (sre, rest) = self.work.split_at_mut(n / 2);
        let (sim, rest) = rest.split_at_mut(n / 2);
        let (acc_re, rest) = rest.split_at_mut(s);
        let (acc_im, tout) = rest.split_at_mut(s);

        RealFft::new(sre, sim).forward(
            self.win,
            &mut self.xh_re[self.head * s..self.head * s + s],
            &mut self.xh_im[self.head * s..self.head * s + s],
        );

        // Frequency-domain FIR delay line: acc = Σ_p X[head-p] · H[p].
        for v in acc_re.iter_mut() {
            *v = 0.0;
        }
        for v in acc_im.iter_mut() {
            *v = 0.0;
        }
        for pp in 0..p {
            let ring = (self.head + p - pp) % p; // input from pp blocks ago
            let xo = ring * s;
            let ho = pp * s;
            for k in 0..s {
                let (xr, xi) = (self.xh_re[xo + k], self.xh_im[xo + k]);
                let (hr, hi) = (self.ir_re[ho + k], self.ir_im[ho + k]);
                acc_re[k] += xr * hr - xi * hi;
                acc_im[k] += xr * hi + xi * hr;
            }
        }

        // Inverse-FFT; overlap-save keeps the second half (the first B is aliased).
        RealFft::new(sre, sim).inverse(acc_re, acc_im, tout);
        out_block.copy_from_slice(&tout[b..n]);

        self.head = (self.head + 1) % p;
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn lcg(seed: &mut u64) -> f32 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*seed >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0
    }

    // Direct linear convolution oracle.
    fn direct(x: &[f32], h: &[f32]) -> Vec<f32> {
        let mut y = std::vec![0.0f32; x.len() + h.len() - 1];
        for (n, xn) in x.iter().enumerate() {
            for (k, hk) in h.iter().enumerate() {
                y[n + k] += xn * hk;
            }
        }
        y
    }

    #[test]
    fn matches_direct_convolution() {
        for &(irlen, b) in &[(700usize, 256usize), (1024, 128), (300, 64)] {
            let mut seed = 0xC0FFEE ^ (irlen as u64);
            let ir: Vec<f32> = (0..irlen).map(|_| lcg(&mut seed) * 0.3).collect();
            let nblocks = 24;
            let x: Vec<f32> = (0..nblocks * b).map(|_| lcg(&mut seed)).collect();
            let want = direct(&x, &ir);

            let mut scratch = std::vec![0.0f32; Convolver::required_scratch(irlen, b)];
            let mut conv = Convolver::new(&ir, b, &mut scratch);
            let mut got = Vec::new();
            let mut ob = std::vec![0.0f32; b];
            for blk in x.chunks_exact(b) {
                conv.process_block(blk, &mut ob);
                got.extend_from_slice(&ob);
            }

            // UPOLS is block-aligned (no extra latency): output block m ↔ input
            // block m. Compare the interior against the direct convolution.
            let mut max_err = 0.0f32;
            for i in b..(got.len() - b) {
                max_err = max_err.max((got[i] - want[i]).abs());
            }
            assert!(max_err < 2e-3, "irlen={irlen} b={b}: conv err {max_err}");
        }
    }

    #[test]
    fn impulse_response_is_reproduced() {
        // Convolving a unit impulse with the IR must return the IR itself.
        let b = 64usize;
        let mut seed = 0x1234u64;
        let ir: Vec<f32> = (0..200).map(|_| lcg(&mut seed) * 0.5).collect();
        let mut scratch = std::vec![0.0f32; Convolver::required_scratch(ir.len(), b)];
        let mut conv = Convolver::new(&ir, b, &mut scratch);

        let mut out = Vec::new();
        let mut ob = std::vec![0.0f32; b];
        for blk in 0..12 {
            let mut ib = std::vec![0.0f32; b];
            if blk == 0 {
                ib[0] = 1.0;
            }
            conv.process_block(&ib, &mut ob);
            out.extend_from_slice(&ob);
        }
        // Block-aligned: convolving an impulse returns the IR starting at out[0].
        for (k, &h) in ir.iter().enumerate() {
            assert!((out[k] - h).abs() < 2e-3, "tap {k}: {} vs {h}", out[k]);
        }
    }
}
