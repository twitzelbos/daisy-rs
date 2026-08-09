//! Fast Fourier transform — a `no_std`, allocation-free, f32 FFT tuned for the
//! Cortex-M7, with a real-input forward/inverse pair for audio spectral work.
//!
//! # Algorithm
//!
//! In-place, decimation-in-time complex FFT over **planar** `re[]`/`im[]` arrays
//! (separate real/imag arrays keep the inner loop FMA- and SIMD-friendly). The
//! transform is a **mixed radix-4 / radix-2** network — the *radix-2²* form: a
//! base-2 bit-reversal permutation followed by radix-4 butterflies (each the
//! fusion of two radix-2 decimation-in-time stages), with a single radix-2 stage
//! for the odd powers of two (512, 2048). Fusing halves the number of memory
//! passes over the data versus plain radix-2 — the dominant cost on the M7 once
//! the working set spills the register file — while the base-2 bit-reversal keeps
//! the permutation trivially correct for every N = 2^m. The first radix-4 stage
//! has unit twiddles and is multiply-free.
//!
//! For real signals, an N-point real transform is computed as an **N/2-point
//! complex** FFT plus an O(N) split step (Cooley–Tukey real-input packing), a 2×
//! saving over zero-padding to complex.
//!
//! All twiddle factors are read from a compile-time table (see `build.rs`); the
//! transform itself calls no `sin`/`cos` and allocates nothing — scratch is
//! borrowed from the caller ([`RealFft`]).
//!
//! # References
//! - J. W. Cooley & J. W. Tukey, "An Algorithm for the Machine Calculation of
//!   Complex Fourier Series," *Math. Comp.* 19 (1965), 297–301.
//! - P. Duhamel & M. Vetterli, "Fast Fourier transforms: a tutorial review and a
//!   state of the art," *Signal Processing* 19 (1990), 259–299. (radix-2²)
//! - H. Sorensen et al., "Real-valued fast Fourier transform algorithms," *IEEE
//!   Trans. ASSP* 35 (1987), 849–863. (real-input packing)
//!
//! Sizes 256/512/1024/2048 are the intended range (twiddle master table = 2048).

// Twiddle master table: TW_COS/TW_SIN of length TW_MASTER (= 2048), holding
// exp(-j·2π·i/TW_MASTER). See build.rs.
include!(concat!(env!("OUT_DIR"), "/twiddles.rs"));

const MASK: usize = TW_MASTER - 1;

/// `W_M^k = exp(-j·2π·k/M)` for any `M | TW_MASTER`, as `(re, im)`.
/// `stride = TW_MASTER / M`; the caller supplies `k * stride`.
#[inline(always)]
fn tw(index: usize) -> (f32, f32) {
    let i = index & MASK;
    (TW_COS[i], TW_SIN[i])
}

#[inline(always)]
fn cmul(ar: f32, ai: f32, br: f32, bi: f32) -> (f32, f32) {
    (ar * br - ai * bi, ar * bi + ai * br)
}

/// In-place complex FFT of `re`/`im` (equal length, a power of two ≤ 1024).
/// `inverse = true` computes the inverse transform (with the `1/L` scaling).
///
/// This is the low-level core; most callers want [`RealFft`]. It dispatches to
/// the const-specialized body ([`cfft_n`]) for the standard sizes, so even the
/// runtime path is monomorphized for 128…1024.
pub fn fft_in_place(re: &mut [f32], im: &mut [f32], inverse: bool) {
    match re.len() {
        128 => cfft_n::<128>(re, im, inverse),
        256 => cfft_n::<256>(re, im, inverse),
        512 => cfft_n::<512>(re, im, inverse),
        1024 => cfft_n::<1024>(re, im, inverse),
        l => cfft_body(re, im, inverse, l),
    }
}

/// Const-length complex FFT: `L` is a compile-time constant, so the stage loop,
/// the bit-reversal bound, and the twiddle strides fold to constants — per-size
/// monomorphization with no hand-written kernel per size. `re`/`im` must both
/// have length `L`.
pub fn cfft_n<const L: usize>(re: &mut [f32], im: &mut [f32], inverse: bool) {
    assert_eq!(re.len(), L, "cfft_n::<L>: re length must equal L");
    cfft_body(re, im, inverse, L);
}

#[inline]
fn cfft_body(re: &mut [f32], im: &mut [f32], inverse: bool, l: usize) {
    assert_eq!(l, im.len(), "re/im length mismatch");
    assert!(l.is_power_of_two(), "FFT length must be a power of two");
    assert!(l <= TW_MASTER / 2 || l == TW_MASTER, "FFT length exceeds twiddle table");
    if l <= 1 {
        return;
    }

    // Inverse via conjugation: ifft(x) = conj(fft(conj(x))) / L.
    if inverse {
        for v in im.iter_mut() {
            *v = -*v;
        }
    }

    bit_reverse(re, im, l);

    let m = l.trailing_zeros();
    // One radix-2 stage first when m is odd, so the remaining stages pair up
    // exactly into radix-4. Twiddles are unity here (multiply-free).
    let mut q = 1usize;
    if m & 1 == 1 {
        let mut i = 0;
        while i < l {
            let (ar, ai) = (re[i], im[i]);
            let (br, bi) = (re[i + 1], im[i + 1]);
            re[i] = ar + br;
            im[i] = ai + bi;
            re[i + 1] = ar - br;
            im[i + 1] = ai - bi;
            i += 2;
        }
        q = 2;
    }

    // Radix-4 stages: sub-transform size `q` grows ×4 each stage. Each butterfly
    // is two fused radix-2 DIT levels (block size = 4q).
    while q < l {
        radix4_stage(re, im, q, l);
        q *= 4;
    }

    if inverse {
        let s = 1.0 / l as f32;
        for v in re.iter_mut() {
            *v *= s;
        }
        for v in im.iter_mut() {
            *v = -*v * s;
        }
    }
}

/// One radix-4 (fused radix-2²) decimation-in-time stage combining size-`q`
/// sub-transforms into size-`4q`, over base-2 bit-reversed data.
#[inline]
fn radix4_stage(re: &mut [f32], im: &mut [f32], q: usize, l: usize) {
    let block = 4 * q;
    let stride_q = TW_MASTER / (2 * q); // W_{2q}
    let stride_a = TW_MASTER / (4 * q); // W_{4q}

    let mut base = 0;
    while base < l {
        for k in 0..q {
            let p0 = base + k;
            let p1 = p0 + q;
            let p2 = p1 + q;
            let p3 = p2 + q;

            let (tqr, tqi) = tw(k * stride_q);
            let (tar, tai) = tw(k * stride_a);

            // First fused level (twiddle W_{2q}^k on the odd members).
            let (b1r, b1i) = cmul(re[p1], im[p1], tqr, tqi);
            let (d1r, d1i) = cmul(re[p3], im[p3], tqr, tqi);
            let (x0r, x0i) = (re[p0], im[p0]);
            let (x2r, x2i) = (re[p2], im[p2]);
            let (u0r, u0i) = (x0r + b1r, x0i + b1i);
            let (u1r, u1i) = (x0r - b1r, x0i - b1i);
            let (u2r, u2i) = (x2r + d1r, x2i + d1i);
            let (u3r, u3i) = (x2r - d1r, x2i - d1i);

            // Second fused level (twiddle W_{4q}^k, and W_{4q}^{k+q} = -j·W_{4q}^k).
            let (c2r, c2i) = cmul(u2r, u2i, tar, tai);
            let (e3r, e3i) = cmul(u3r, u3i, tar, tai);
            re[p0] = u0r + c2r;
            im[p0] = u0i + c2i;
            re[p2] = u0r - c2r;
            im[p2] = u0i - c2i;
            // (-j)·e3 = (e3i, -e3r)
            re[p1] = u1r + e3i;
            im[p1] = u1i - e3r;
            re[p3] = u1r - e3i;
            im[p3] = u1i + e3r;
        }
        base += block;
    }
}

/// Standard base-2 bit-reversal permutation over planar arrays.
#[inline]
fn bit_reverse(re: &mut [f32], im: &mut [f32], n: usize) {
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
}

/// A real-input FFT/iFFT over borrowed scratch — no allocation.
///
/// One instance handles a fixed transform size `N`, inferred from the slice
/// lengths at each call: `forward` takes `N` real samples and produces the
/// `N/2 + 1` non-redundant complex bins; `inverse` takes those bins back to `N`
/// real samples. The scratch arrays are `N/2` long each and hold the packed
/// complex transform — place them in DTCM/AXI SRAM as the app chooses.
pub struct RealFft<'s> {
    sre: &'s mut [f32],
    sim: &'s mut [f32],
}

impl<'s> RealFft<'s> {
    /// Build over two scratch arrays of length `N/2` (the packed complex length).
    pub fn new(scratch_re: &'s mut [f32], scratch_im: &'s mut [f32]) -> Self {
        assert_eq!(scratch_re.len(), scratch_im.len(), "scratch length mismatch");
        assert!(scratch_re.len().is_power_of_two(), "N/2 must be a power of two");
        Self { sre: scratch_re, sim: scratch_im }
    }

    /// `N = 2 * scratch_len`.
    #[inline]
    pub fn n(&self) -> usize {
        self.sre.len() * 2
    }

    /// Forward real FFT: `input` (`N` samples) → `out_re`/`out_im` (`N/2 + 1`
    /// bins each). `out[0]` is DC (imag 0), `out[N/2]` is Nyquist (imag 0).
    pub fn forward(&mut self, input: &[f32], out_re: &mut [f32], out_im: &mut [f32]) {
        let n = self.n();
        real_forward(self.sre, self.sim, input, out_re, out_im, n);
    }

    /// Const-`N` forward real FFT — monomorphized end-to-end for a pinned size
    /// (complex core, split step, and all strides fold to constants). `N` must
    /// equal this instance's transform size (`2 × scratch length`).
    pub fn forward_n<const N: usize>(
        &mut self,
        input: &[f32],
        out_re: &mut [f32],
        out_im: &mut [f32],
    ) {
        assert_eq!(self.n(), N, "forward_n::<N>: N must equal 2 × scratch length");
        real_forward(self.sre, self.sim, input, out_re, out_im, N);
    }

    /// Inverse real FFT: `in_re`/`in_im` (`N/2 + 1` bins) → `output` (`N`
    /// samples). The bins above Nyquist are implied by conjugate symmetry.
    pub fn inverse(&mut self, in_re: &[f32], in_im: &[f32], output: &mut [f32]) {
        let n = self.n();
        real_inverse(self.sre, self.sim, in_re, in_im, output, n);
    }

    /// Const-`N` inverse real FFT — see [`forward_n`](Self::forward_n).
    pub fn inverse_n<const N: usize>(
        &mut self,
        in_re: &[f32],
        in_im: &[f32],
        output: &mut [f32],
    ) {
        assert_eq!(self.n(), N, "inverse_n::<N>: N must equal 2 × scratch length");
        real_inverse(self.sre, self.sim, in_re, in_im, output, N);
    }
}

/// Shared real-forward body. `n` is `const` on the `forward_n` path, so the
/// split loop and strides specialize; runtime on the `forward` path.
#[inline]
fn real_forward(
    sre: &mut [f32],
    sim: &mut [f32],
    input: &[f32],
    out_re: &mut [f32],
    out_im: &mut [f32],
    n: usize,
) {
    let l = n / 2;
    assert_eq!(sre.len(), l, "scratch length must be N/2");
    assert_eq!(input.len(), n, "input must be N samples");
    assert_eq!(out_re.len(), l + 1, "out_re must be N/2 + 1");
    assert_eq!(out_im.len(), l + 1, "out_im must be N/2 + 1");

    // Pack the N real samples as L = N/2 complex: even → re, odd → im.
    for i in 0..l {
        sre[i] = input[2 * i];
        sim[i] = input[2 * i + 1];
    }
    fft_in_place(sre, sim, false);

    // Split step: recover the real spectrum bins 0..=N/2 from the packed
    // transform Z (period L, so Z[L] = Z[0]). W_N^k is a strided lookup.
    let stride_n = TW_MASTER / n;
    for k in 0..=l {
        let kl = if k == l { 0 } else { k };
        let lk = if k == 0 { 0 } else { l - k };
        let (zr, zi) = (sre[kl], sim[kl]);
        let (zr2, zi2) = (sre[lk], sim[lk]);

        // Xe = (Z[k] + conj(Z[L-k])) / 2
        let xe_r = 0.5 * (zr + zr2);
        let xe_i = 0.5 * (zi - zi2);
        // Xo = (Z[k] - conj(Z[L-k])) / (2j)
        let ar = zr - zr2;
        let bi = zi + zi2;
        let xo_r = 0.5 * bi;
        let xo_i = -0.5 * ar;
        // X[k] = Xe + W_N^k · Xo
        let (wr, wi) = tw(k * stride_n);
        let (pr, pi) = cmul(xo_r, xo_i, wr, wi);
        out_re[k] = xe_r + pr;
        out_im[k] = xe_i + pi;
    }
}

/// Shared real-inverse body (see [`real_forward`]).
#[inline]
fn real_inverse(
    sre: &mut [f32],
    sim: &mut [f32],
    in_re: &[f32],
    in_im: &[f32],
    output: &mut [f32],
    n: usize,
) {
    let l = n / 2;
    assert_eq!(sre.len(), l, "scratch length must be N/2");
    assert_eq!(in_re.len(), l + 1, "in_re must be N/2 + 1");
    assert_eq!(in_im.len(), l + 1, "in_im must be N/2 + 1");
    assert_eq!(output.len(), n, "output must be N samples");

    // Undo the split step to rebuild the packed complex transform Z[0..L].
    let stride_n = TW_MASTER / n;
    for k in 0..l {
        let (xr, xi) = (in_re[k], in_im[k]);
        // conj(X[L-k]); X[L-k] for k=0 is X[L] (Nyquist), available in bin L.
        let lk = l - k;
        let (yr, yi) = (in_re[lk], -in_im[lk]);

        let xe_r = 0.5 * (xr + yr);
        let xe_i = 0.5 * (xi + yi);
        let t_r = 0.5 * (xr - yr);
        let t_i = 0.5 * (xi - yi);
        // Xo = conj(W_N^k) · t  (undo the forward W_N^k)
        let (wr, wi) = tw(k * stride_n);
        let (xo_r, xo_i) = cmul(t_r, t_i, wr, -wi);
        // Z[k] = Xe + j·Xo
        sre[k] = xe_r - xo_i;
        sim[k] = xe_i + xo_r;
    }

    fft_in_place(sre, sim, true);

    // Unpack: even samples ← re, odd samples ← im.
    for i in 0..l {
        output[2 * i] = sre[i];
        output[2 * i + 1] = sim[i];
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    const SIZES: [usize; 4] = [256, 512, 1024, 2048];

    // ---- oracle: the naive O(N^2) DFT in f64 (the mathematical definition) ----
    fn naive_dft(re: &[f32], im: &[f32]) -> (Vec<f64>, Vec<f64>) {
        let n = re.len();
        let mut or = std::vec![0.0f64; n];
        let mut oi = std::vec![0.0f64; n];
        for k in 0..n {
            let (mut sr, mut si) = (0.0f64, 0.0f64);
            for t in 0..n {
                let a = -2.0 * std::f64::consts::PI * (k as f64) * (t as f64) / (n as f64);
                let (c, s) = (a.cos(), a.sin());
                let (xr, xi) = (re[t] as f64, im[t] as f64);
                sr += xr * c - xi * s;
                si += xr * s + xi * c;
            }
            or[k] = sr;
            oi[k] = si;
        }
        (or, oi)
    }

    fn lcg(seed: &mut u64) -> f32 {
        // deterministic uniform in [-1, 1)
        *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((*seed >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0
    }

    fn max_abs_err(a: &[f32], b: &[f64]) -> f64 {
        a.iter().zip(b).map(|(x, y)| (*x as f64 - *y).abs()).fold(0.0, f64::max)
    }

    #[test]
    fn complex_fft_matches_naive_dft() {
        for &n in &SIZES {
            let mut seed = 0x1234_5678_9abc_def0 ^ n as u64;
            let re0: Vec<f32> = (0..n).map(|_| lcg(&mut seed)).collect();
            let im0: Vec<f32> = (0..n).map(|_| lcg(&mut seed)).collect();
            let (mut re, mut im) = (re0.clone(), im0.clone());
            fft_in_place(&mut re, &mut im, false);
            let (gr, gi) = naive_dft(&re0, &im0);
            // error grows ~sqrt(N) in f32; tolerance scaled accordingly.
            let tol = 3e-3 * (n as f64).sqrt();
            assert!(max_abs_err(&re, &gr) < tol, "N={n} re err {}", max_abs_err(&re, &gr));
            assert!(max_abs_err(&im, &gi) < tol, "N={n} im err {}", max_abs_err(&im, &gi));
        }
    }

    #[test]
    fn complex_roundtrip_is_identity() {
        for &n in &SIZES {
            let mut seed = 0xdead_beef ^ n as u64;
            let re0: Vec<f32> = (0..n).map(|_| lcg(&mut seed)).collect();
            let im0: Vec<f32> = (0..n).map(|_| lcg(&mut seed)).collect();
            let (mut re, mut im) = (re0.clone(), im0.clone());
            fft_in_place(&mut re, &mut im, false);
            fft_in_place(&mut re, &mut im, true);
            let er = max_abs_err(&re, &re0.iter().map(|&x| x as f64).collect::<Vec<_>>());
            let ei = max_abs_err(&im, &im0.iter().map(|&x| x as f64).collect::<Vec<_>>());
            assert!(er < 1e-4 && ei < 1e-4, "N={n} roundtrip err {er}/{ei}");
        }
    }

    #[test]
    fn real_fft_matches_naive_dft() {
        for &n in &SIZES {
            let mut seed = 0x0bad_c0de ^ n as u64;
            let input: Vec<f32> = (0..n).map(|_| lcg(&mut seed)).collect();
            let zeros = std::vec![0.0f32; n];
            let (gr, gi) = naive_dft(&input, &zeros);

            let (mut sre, mut sim) = (std::vec![0.0f32; n / 2], std::vec![0.0f32; n / 2]);
            let (mut outr, mut outi) = (std::vec![0.0f32; n / 2 + 1], std::vec![0.0f32; n / 2 + 1]);
            RealFft::new(&mut sre, &mut sim).forward(&input, &mut outr, &mut outi);

            let tol = 3e-3 * (n as f64).sqrt();
            for k in 0..=n / 2 {
                assert!((outr[k] as f64 - gr[k]).abs() < tol, "N={n} k={k} re");
                assert!((outi[k] as f64 - gi[k]).abs() < tol, "N={n} k={k} im");
            }
            // DC and Nyquist bins are purely real.
            assert!(outi[0].abs() < 1e-3 && outi[n / 2].abs() < 1e-3, "N={n} DC/Nyq imag");
        }
    }

    #[test]
    fn real_roundtrip_is_identity() {
        for &n in &SIZES {
            let mut seed = 0xfeed_face ^ n as u64;
            let input: Vec<f32> = (0..n).map(|_| lcg(&mut seed)).collect();
            let (mut sre, mut sim) = (std::vec![0.0f32; n / 2], std::vec![0.0f32; n / 2]);
            let (mut outr, mut outi) = (std::vec![0.0f32; n / 2 + 1], std::vec![0.0f32; n / 2 + 1]);
            let mut back = std::vec![0.0f32; n];
            {
                let mut fft = RealFft::new(&mut sre, &mut sim);
                fft.forward(&input, &mut outr, &mut outi);
                fft.inverse(&outr, &outi, &mut back);
            }
            let err = input
                .iter()
                .zip(&back)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0f32, f32::max);
            assert!(err < 1e-4, "N={n} real roundtrip err {err}");
        }
    }

    #[test]
    fn known_signals() {
        let n = 1024usize;
        let (mut sre, mut sim) = (std::vec![0.0f32; n / 2], std::vec![0.0f32; n / 2]);
        let (mut outr, mut outi) = (std::vec![0.0f32; n / 2 + 1], std::vec![0.0f32; n / 2 + 1]);

        // DC: all-ones → energy only in bin 0.
        let ones = std::vec![1.0f32; n];
        RealFft::new(&mut sre, &mut sim).forward(&ones, &mut outr, &mut outi);
        assert!((outr[0] - n as f32).abs() < 1e-1, "DC bin");
        for k in 1..=n / 2 {
            assert!(outr[k].abs() < 1e-1 && outi[k].abs() < 1e-1, "DC leak k={k}");
        }

        // A pure cosine at bin b → a single spectral line of height N/2 at b.
        let b = 17usize;
        let cos: Vec<f32> = (0..n)
            .map(|t| (2.0 * std::f32::consts::PI * b as f32 * t as f32 / n as f32).cos())
            .collect();
        RealFft::new(&mut sre, &mut sim).forward(&cos, &mut outr, &mut outi);
        for k in 0..=n / 2 {
            let mag = (outr[k] * outr[k] + outi[k] * outi[k]).sqrt();
            let expect = if k == b { n as f32 / 2.0 } else { 0.0 };
            assert!((mag - expect).abs() < 5e-1, "cos bin k={k} mag={mag}");
        }
    }

    #[test]
    fn parseval_energy_conservation() {
        // Σ|x|² == (1/N) Σ|X|² over the full complex spectrum.
        for &n in &SIZES {
            let mut seed = 0xa5a5_5a5a ^ n as u64;
            let input: Vec<f32> = (0..n).map(|_| lcg(&mut seed)).collect();
            let time_energy: f64 = input.iter().map(|&x| (x as f64) * (x as f64)).sum();

            let (mut sre, mut sim) = (std::vec![0.0f32; n / 2], std::vec![0.0f32; n / 2]);
            let (mut outr, mut outi) = (std::vec![0.0f32; n / 2 + 1], std::vec![0.0f32; n / 2 + 1]);
            RealFft::new(&mut sre, &mut sim).forward(&input, &mut outr, &mut outi);

            // full-spectrum energy: bins 1..N/2 counted twice (conjugate pair).
            let mut spec = (outr[0] * outr[0] + outi[0] * outi[0]) as f64
                + (outr[n / 2] * outr[n / 2] + outi[n / 2] * outi[n / 2]) as f64;
            for k in 1..n / 2 {
                spec += 2.0 * (outr[k] * outr[k] + outi[k] * outi[k]) as f64;
            }
            let freq_energy = spec / n as f64;
            let rel = (time_energy - freq_energy).abs() / time_energy.max(1e-9);
            assert!(rel < 1e-3, "N={n} Parseval rel err {rel}");
        }
    }

    #[test]
    fn linearity() {
        let n = 512usize;
        let mut seed = 0x1357_9bdf;
        let x: Vec<f32> = (0..n).map(|_| lcg(&mut seed)).collect();
        let y: Vec<f32> = (0..n).map(|_| lcg(&mut seed)).collect();
        let sum: Vec<f32> = x.iter().zip(&y).map(|(a, b)| 2.0 * a - 3.0 * b).collect();

        let (mut sre, mut sim) = (std::vec![0.0f32; n / 2], std::vec![0.0f32; n / 2]);
        let mk = |inp: &[f32], sre: &mut [f32], sim: &mut [f32]| {
            let (mut r, mut i) = (std::vec![0.0f32; n / 2 + 1], std::vec![0.0f32; n / 2 + 1]);
            RealFft::new(sre, sim).forward(inp, &mut r, &mut i);
            (r, i)
        };
        let (xr, xi) = mk(&x, &mut sre, &mut sim);
        let (yr, yi) = mk(&y, &mut sre, &mut sim);
        let (sr, si) = mk(&sum, &mut sre, &mut sim);
        for k in 0..=n / 2 {
            assert!((sr[k] - (2.0 * xr[k] - 3.0 * yr[k])).abs() < 1e-2, "lin re k={k}");
            assert!((si[k] - (2.0 * xi[k] - 3.0 * yi[k])).abs() < 1e-2, "lin im k={k}");
        }
    }

    #[test]
    fn const_n_bit_identical_to_runtime() {
        // The const-N specialized path must produce *exactly* the same bits as
        // the runtime path — same algorithm, only monomorphized.
        macro_rules! check {
            ($n:literal) => {{
                let n = $n;
                let mut seed = 0xc0ffee ^ n as u64;
                let input: Vec<f32> = (0..n).map(|_| lcg(&mut seed)).collect();
                let (mut sre, mut sim) = (std::vec![0.0f32; n / 2], std::vec![0.0f32; n / 2]);
                let (mut r0, mut i0) = (std::vec![0.0f32; n / 2 + 1], std::vec![0.0f32; n / 2 + 1]);
                let (mut r1, mut i1) = (std::vec![0.0f32; n / 2 + 1], std::vec![0.0f32; n / 2 + 1]);
                RealFft::new(&mut sre, &mut sim).forward(&input, &mut r0, &mut i0);
                RealFft::new(&mut sre, &mut sim).forward_n::<$n>(&input, &mut r1, &mut i1);
                assert_eq!(r0, r1, "N={n} forward re differs");
                assert_eq!(i0, i1, "N={n} forward im differs");

                let mut b0 = std::vec![0.0f32; n];
                let mut b1 = std::vec![0.0f32; n];
                RealFft::new(&mut sre, &mut sim).inverse(&r0, &i0, &mut b0);
                RealFft::new(&mut sre, &mut sim).inverse_n::<$n>(&r1, &i1, &mut b1);
                assert_eq!(b0, b1, "N={n} inverse differs");
            }};
        }
        check!(256);
        check!(512);
        check!(1024);
        check!(2048);
    }
}
