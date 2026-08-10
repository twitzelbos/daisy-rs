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
    pub const fn hop(&self) -> usize {
        N / 4
    }

    /// Latency from input to reconstructed output (samples).
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
        for v in &mut self.ola[N - hop..] {
            *v = 0.0;
        }
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
    pub fn new() -> Self {
        Self {
            stft: Stft::new(),
            mag: [0.0; N],
            phase: [0.0; N],
            frozen: false,
            captured: false,
        }
    }

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
}
