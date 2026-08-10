//! Short-time Fourier transform (STFT) — the phase-vocoder front end. `no_std`,
//! no alloc (all buffers owned inline; place the struct in AXI SRAM/SDRAM).
//!
//! Streaming analysis/modify/synthesis with a Hann window and 75%-overlap
//! (`hop = N/4`) weighted overlap-add. Analysis **and** synthesis windowing
//! (Hann², normalised by its COLA constant) gives perfect reconstruction for an
//! identity modification and stays click-free when the spectrum is altered — the
//! basis for spectral freeze, filtering, and (later) pitch-shift/time-stretch.
//!
//! The caller feeds one `hop`-sized block per [`process_hop`](Stft::process_hop)
//! and supplies a closure that may rewrite the `N/2 + 1` complex bins in place.
//!
//! # References
//! - J. Allen, "Short Term Spectral Analysis, Synthesis, and Modification by
//!   Discrete Fourier Transform," *IEEE Trans. ASSP* 25(3), 1977.
//! - M. Dolson, "The Phase Vocoder: A Tutorial," *Computer Music J.* 10(4), 1986.

use crate::fft::RealFft;
use core::f32::consts::TAU;

/// A streaming STFT over a frame of `N` samples (a power of two), hop `N/4`.
#[derive(Debug)]
pub struct Stft<const N: usize> {
    win: [f32; N],
    inbuf: [f32; N], // sliding window of the last N input samples
    frame: [f32; N], // scratch: windowed frame / inverse-transform result
    ola: [f32; N],   // overlap-add accumulator
    spec_re: [f32; N],
    spec_im: [f32; N],
    sre: [f32; N], // RealFft scratch (uses [..N/2])
    sim: [f32; N],
    hop: usize,
    norm: f32,
}

impl<const N: usize> Default for Stft<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Stft<N> {
    /// Build a streaming STFT with a Hann window and hop `N/4`.
    ///
    /// # Panics
    /// Panics if `N` is not a power of two ≥ 64.
    #[must_use]
    pub fn new() -> Self {
        assert!(
            N.is_power_of_two() && N >= 64,
            "STFT N must be a power of two >= 64"
        );
        let hop = N / 4;
        let mut win = [0.0f32; N];
        for (i, w) in win.iter_mut().enumerate() {
            *w = 0.5 - 0.5 * libm::cosf(TAU * i as f32 / N as f32);
        }
        // COLA constant for the Hann² weighting at this hop (steady-state gain).
        let mut norm = 0.0f32;
        let mut k = (N / 2) % hop;
        while k < N {
            norm += win[k] * win[k];
            k += hop;
        }
        Self {
            win,
            inbuf: [0.0; N],
            frame: [0.0; N],
            ola: [0.0; N],
            spec_re: [0.0; N],
            spec_im: [0.0; N],
            sre: [0.0; N],
            sim: [0.0; N],
            hop,
            norm,
        }
    }

    /// The hop size (samples per [`process_hop`](Self::process_hop) call).
    #[must_use]
    pub const fn hop(&self) -> usize {
        N / 4
    }

    /// Latency from input to reconstructed output (samples).
    #[must_use]
    pub const fn latency(&self) -> usize {
        N - N / 4
    }

    /// Process one hop. `in_hop`/`out_hop` are `hop` samples. `spectrum` receives
    /// the `N/2 + 1` complex bins (`re`, `im`) to rewrite in place; pass a no-op
    /// closure for identity reconstruction.
    pub fn process_hop(
        &mut self,
        in_hop: &[f32],
        out_hop: &mut [f32],
        mut spectrum: impl FnMut(&mut [f32], &mut [f32]),
    ) {
        let hop = self.hop;
        let half = N / 2;
        debug_assert_eq!(in_hop.len(), hop);
        debug_assert_eq!(out_hop.len(), hop);

        // Slide the input frame left by hop and append the new samples.
        self.inbuf.copy_within(hop.., 0);
        self.inbuf[N - hop..].copy_from_slice(in_hop);

        // Analysis window → forward FFT.
        for i in 0..N {
            self.frame[i] = self.inbuf[i] * self.win[i];
        }
        RealFft::new(&mut self.sre[..half], &mut self.sim[..half]).forward(
            &self.frame,
            &mut self.spec_re[..half + 1],
            &mut self.spec_im[..half + 1],
        );

        // Spectral modification (identity if the closure does nothing).
        spectrum(&mut self.spec_re[..half + 1], &mut self.spec_im[..half + 1]);

        // Inverse FFT → synthesis window → weighted overlap-add.
        RealFft::new(&mut self.sre[..half], &mut self.sim[..half]).inverse(
            &self.spec_re[..half + 1],
            &self.spec_im[..half + 1],
            &mut self.frame,
        );
        self.ola.copy_within(hop.., 0);
        self.ola[N - hop..].fill(0.0);
        let inv_norm = 1.0 / self.norm;
        for i in 0..N {
            self.ola[i] += self.frame[i] * self.win[i] * inv_norm;
        }
        out_hop.copy_from_slice(&self.ola[..hop]);
    }
}

/// Spectral freeze: capture one frame's magnitude spectrum and resynthesise it
/// indefinitely, advancing each bin's phase by its per-hop increment so the tone
/// sustains smoothly (a "true" frozen spectrum, unlike a time-domain loop).
///
/// Wraps an [`Stft`]; drive it hop-by-hop. While frozen it ignores the input.
#[derive(Debug)]
pub struct SpectralFreeze<const N: usize> {
    stft: Stft<N>,
    mag: [f32; N],   // frozen magnitude (uses [..N/2+1])
    phase: [f32; N], // running phase per bin
    frozen: bool,
    captured: bool,
}

impl<const N: usize> Default for SpectralFreeze<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> SpectralFreeze<N> {
    /// Build a spectral freeze wrapping a fresh [`Stft`].
    #[must_use]
    pub fn new() -> Self {
        Self {
            stft: Stft::new(),
            mag: [0.0; N],
            phase: [0.0; N],
            frozen: false,
            captured: false,
        }
    }

    /// The hop size (samples per [`process_hop`](Self::process_hop) call).
    #[must_use]
    pub const fn hop(&self) -> usize {
        N / 4
    }

    /// Arm/disarm the freeze. On the first hop after arming, the current spectrum
    /// is captured; subsequent hops resynthesise it.
    pub fn set_frozen(&mut self, frozen: bool) {
        self.frozen = frozen;
        if !frozen {
            self.captured = false;
        }
    }

    /// Process one hop. `in_hop`/`out_hop` are `hop` samples; while frozen the
    /// captured spectrum is resynthesised and the input ignored.
    pub fn process_hop(&mut self, in_hop: &[f32], out_hop: &mut [f32]) {
        let half = N / 2;
        let frozen = self.frozen;
        let captured = &mut self.captured;
        let mag = &mut self.mag;
        let phase = &mut self.phase;
        self.stft.process_hop(in_hop, out_hop, |re, im| {
            if !frozen {
                return;
            }
            if !*captured {
                // Capture magnitude + initial phase.
                for k in 0..=half {
                    mag[k] = libm::sqrtf(re[k] * re[k] + im[k] * im[k]);
                    phase[k] = libm::atan2f(im[k], re[k]);
                }
                *captured = true;
            } else {
                // Advance each bin's phase by its natural per-hop increment
                // (2π·k·hop/N) and rebuild the bin from the frozen magnitude.
                let hop = N / 4;
                for k in 0..=half {
                    phase[k] += TAU * (k * hop) as f32 / N as f32;
                    re[k] = mag[k] * libm::cosf(phase[k]);
                    im[k] = mag[k] * libm::sinf(phase[k]);
                }
            }
        });
    }
}

/// Principal argument: wrap a phase to `(-π, π]`.
#[inline]
fn wrap(x: f32) -> f32 {
    x - TAU * libm::roundf(x / TAU)
}

/// Real-time phase-vocoder **pitch shifter**: shifts pitch by `ratio` while
/// keeping input and output sample rates equal (no length change). Each bin's
/// *true* frequency is estimated from the inter-frame phase advance, the
/// spectrum is remapped to `ratio × k`, and the synthesis phase is accumulated
/// at the shifted frequency — the standard streaming phase vocoder.
///
/// Wraps an [`Stft`]; drive it hop-by-hop. `ratio` 2.0 = up an octave, 0.5 = down.
#[derive(Debug)]
pub struct SpectralPitchShifter<const N: usize> {
    stft: Stft<N>,
    ratio: f32,
    prev_phase: [f32; N], // previous analysis phase (uses [..N/2+1])
    sum_phase: [f32; N],  // accumulated synthesis phase
    mag: [f32; N],        // analysis magnitude
    freq: [f32; N],       // analysis true frequency (bins)
    syn_mag: [f32; N],    // synthesis magnitude
    syn_freq: [f32; N],   // synthesis true frequency (bins)
}

impl<const N: usize> SpectralPitchShifter<N> {
    /// Build a phase-vocoder pitch shifter wrapping a fresh [`Stft`], shifting by
    /// `ratio` (output/input pitch).
    #[must_use]
    pub fn new(ratio: f32) -> Self {
        Self {
            stft: Stft::new(),
            ratio: ratio.max(0.05),
            prev_phase: [0.0; N],
            sum_phase: [0.0; N],
            mag: [0.0; N],
            freq: [0.0; N],
            syn_mag: [0.0; N],
            syn_freq: [0.0; N],
        }
    }

    /// The hop size (samples per [`process_hop`](Self::process_hop) call).
    #[must_use]
    pub const fn hop(&self) -> usize {
        N / 4
    }

    /// Retune the shift ratio (output/input pitch).
    pub fn set_ratio(&mut self, ratio: f32) {
        self.ratio = ratio.max(0.05);
    }

    /// Shift by a semitone amount: `2^(semitones/12)`.
    pub fn set_semitones(&mut self, semitones: f32) {
        self.set_ratio(libm::exp2f(semitones / 12.0));
    }

    /// Process one hop. `in_hop`/`out_hop` are `hop` samples; the spectrum is
    /// pitch-shifted by `ratio` via the streaming phase vocoder.
    pub fn process_hop(&mut self, in_hop: &[f32], out_hop: &mut [f32]) {
        let half = N / 2;
        let hop = (N / 4) as f32;
        let n = N as f32;
        let ratio = self.ratio;
        let prev_phase = &mut self.prev_phase;
        let sum_phase = &mut self.sum_phase;
        let mag = &mut self.mag;
        let freq = &mut self.freq;
        let syn_mag = &mut self.syn_mag;
        let syn_freq = &mut self.syn_freq;

        self.stft.process_hop(in_hop, out_hop, |re, im| {
            // --- analysis: magnitude + true (instantaneous) frequency per bin ---
            for k in 0..=half {
                let m = libm::sqrtf(re[k] * re[k] + im[k] * im[k]);
                let phase = libm::atan2f(im[k], re[k]);
                // deviation of the actual phase advance from the bin's expected one
                let expected = TAU * k as f32 * hop / n;
                let dp = wrap(phase - prev_phase[k] - expected);
                prev_phase[k] = phase;
                mag[k] = m;
                freq[k] = k as f32 + dp * n / (TAU * hop); // true frequency, in bins
            }

            // --- pitch shift: remap bin k → ratio·k, scaling its true frequency ---
            for v in syn_mag[..=half].iter_mut() {
                *v = 0.0;
            }
            for v in syn_freq[..=half].iter_mut() {
                *v = 0.0;
            }
            for k in 0..=half {
                let j = libm::roundf(k as f32 * ratio) as usize;
                if j <= half {
                    syn_mag[j] += mag[k];
                    syn_freq[j] = freq[k] * ratio;
                }
            }

            // --- synthesis: accumulate phase at the shifted frequency ---
            for j in 0..=half {
                sum_phase[j] += TAU * hop * syn_freq[j] / n;
                let ph = sum_phase[j];
                re[j] = syn_mag[j] * libm::cosf(ph);
                im[j] = syn_mag[j] * libm::sinf(ph);
            }
        });
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use std::boxed::Box;
    use std::vec::Vec;

    fn lcg(seed: &mut u64) -> f32 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*seed >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0
    }

    // Drive the STFT hop-by-hop with `sig` and identity modification; return the
    // reconstructed stream.
    fn reconstruct<const N: usize>(sig: &[f32]) -> Vec<f32> {
        let mut s = Box::new(Stft::<N>::new());
        let hop = s.hop();
        let mut out = Vec::new();
        let mut ohop = std::vec![0.0f32; hop];
        for chunk in sig.chunks_exact(hop) {
            s.process_hop(chunk, &mut ohop, |_, _| {});
            out.extend_from_slice(&ohop);
        }
        out
    }

    #[test]
    fn identity_reconstruction_is_exact() {
        for_each_size();
        fn for_each_size() {
            check::<256>();
            check::<512>();
            check::<1024>();
        }
        fn check<const N: usize>() {
            let mut seed = 0xC0DE ^ N as u64;
            let sig: Vec<f32> = (0..N * 12).map(|_| lcg(&mut seed) * 0.5).collect();
            let rec = reconstruct::<N>(&sig);
            // Output lags input by the STFT latency; compare the overlapping tail.
            let lat = N - N / 4;
            let mut max_err = 0.0f32;
            for i in (2 * N)..(sig.len() - N) {
                max_err = max_err.max((rec[i] - sig[i - lat]).abs());
            }
            assert!(max_err < 1e-3, "N={N}: reconstruction err {max_err}");
        }
    }

    #[test]
    fn freeze_sustains_a_tone_after_input_stops() {
        const N: usize = 512;
        let mut fz = Box::new(SpectralFreeze::<N>::new());
        let hop = fz.hop();
        let mut ohop = std::vec![0.0f32; hop];
        // Feed a steady tone, arm the freeze, then feed silence — output must keep
        // sounding at roughly the same level.
        let mut ph2 = 0.0f32;
        let mut run = |fz: &mut SpectralFreeze<N>, silence: bool, ph: &mut f32| -> f32 {
            let mut inb = std::vec![0.0f32; hop];
            for v in inb.iter_mut() {
                *v = if silence { 0.0 } else { 0.5 * libm::sinf(*ph) };
                *ph += TAU * 440.0 / 48_000.0;
            }
            fz.process_hop(&inb, &mut ohop);
            ohop.iter().fold(0.0f32, |m, &x| m.max(x.abs()))
        };

        for _ in 0..40 {
            run(&mut fz, false, &mut ph2);
        }
        fz.set_frozen(true);
        for _ in 0..8 {
            run(&mut fz, false, &mut ph2); // capture + settle
        }
        // Now input goes silent; frozen output must persist.
        let mut sustained = 0.0f32;
        for _ in 0..40 {
            sustained = sustained.max(run(&mut fz, true, &mut ph2));
        }
        assert!(sustained > 0.1, "freeze did not sustain: peak {sustained}");
    }

    #[test]
    fn pitch_shift_moves_the_fundamental() {
        use crate::fft::RealFft;
        const N: usize = 1024;
        for &(ratio, f0) in &[(2.0f32, 300.0f32), (0.5, 600.0), (1.0, 440.0)] {
            let mut ps = Box::new(SpectralPitchShifter::<N>::new(ratio));
            let hop = ps.hop();
            let mut ph = 0.0f32;
            let mut out = Vec::new();
            let mut oh = std::vec![0.0f32; hop];
            for _ in 0..80 {
                let mut ih = std::vec![0.0f32; hop];
                for v in ih.iter_mut() {
                    *v = 0.5 * libm::sinf(ph);
                    ph += TAU * f0 / 48_000.0;
                }
                ps.process_hop(&ih, &mut oh);
                out.extend_from_slice(&oh);
            }
            // Dominant frequency of the settled tail.
            const M: usize = 4096;
            let mut buf = [0.0f32; M];
            for (i, b) in buf.iter_mut().enumerate() {
                let w = 0.5 - 0.5 * libm::cosf(TAU * i as f32 / M as f32);
                *b = out[out.len() - M + i] * w;
            }
            let (mut sre, mut sim) = (std::vec![0.0f32; M / 2], std::vec![0.0f32; M / 2]);
            let (mut re, mut im) = (std::vec![0.0f32; M / 2 + 1], std::vec![0.0f32; M / 2 + 1]);
            RealFft::new(&mut sre, &mut sim).forward(&buf, &mut re, &mut im);
            let (mut best, mut bp) = (1usize, 0.0f32);
            for k in 1..M / 2 {
                let p = re[k] * re[k] + im[k] * im[k];
                if p > bp {
                    bp = p;
                    best = k;
                }
            }
            let dom = best as f32 * 48_000.0 / M as f32;
            let cents = 1200.0 * libm::log2f(dom / (f0 * ratio));
            assert!(
                cents.abs() < 40.0,
                "ratio={ratio} f0={f0}: dom={dom}, want {}",
                f0 * ratio
            );
        }
    }
}
