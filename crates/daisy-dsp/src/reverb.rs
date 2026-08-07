//! A feedback-delay-network (FDN) reverb — the shared "room / atmosphere"
//! engine for the spatializer and the pad/drone generator.
//!
//! Deliberately economical (compute is at a premium on the M7):
//! - **4 delay lines** in one borrowed buffer (place it in SDRAM/AXI SRAM).
//! - A **Householder** feedback matrix `A = I − (2/N)·11ᵀ`, which is lossless
//!   (energy-preserving) yet costs only a sum + a subtract per line — **no
//!   matrix multiply**.
//! - One **one-pole** damping filter per line (HF decays faster — natural).
//! - Per-line decay gains + the damping cutoffs are computed **once** at
//!   construction; the per-sample loop has no `exp`/`sin`/`pow` and wraps the
//!   ring indices with a compare, not a `%`.
//!
//! ~40 flops/sample → a fraction of a percent of the M7. Mono in, decorrelated
//! **stereo** out (two orthogonal output taps), with a dry/wet mix.

use crate::filter::OnePole;

const N: usize = 4;

/// Loop lengths in samples — mutually prime (no shared resonances / flutter),
/// ~26–38 ms at 48 kHz. The reverb's character is tuned around 48 kHz.
const DELAYS: [usize; N] = [1231, 1439, 1607, 1801];

/// A 4-line FDN reverb over a caller-provided buffer.
pub struct FdnReverb<'a> {
    buf: &'a mut [f32],
    start: [usize; N],
    delay: [usize; N],
    pos: [usize; N],
    damp: [OnePole; N],
    fb: [f32; N],
    dry: f32,
    wet: f32,
}

impl<'a> FdnReverb<'a> {
    /// Minimum backing-buffer length (samples) — size the buffer to at least this.
    pub const REQUIRED_BUF: usize = DELAYS[0] + DELAYS[1] + DELAYS[2] + DELAYS[3];

    /// Build a reverb over `buf` (must be ≥ [`REQUIRED_BUF`](Self::REQUIRED_BUF)).
    ///
    /// - `rt60_s`: broadband decay time to −60 dB (seconds).
    /// - `damping_hz`: one-pole low-pass cutoff in the feedback (lower = darker,
    ///   shorter HF tail).
    /// - `mix`: dry↔wet, `0.0` = dry only, `1.0` = wet only.
    pub fn new(
        buf: &'a mut [f32],
        sample_rate: f32,
        rt60_s: f32,
        damping_hz: f32,
        mix: f32,
    ) -> Self {
        assert!(
            buf.len() >= Self::REQUIRED_BUF,
            "FdnReverb buffer too small: need >= {} samples",
            Self::REQUIRED_BUF
        );
        let mut start = [0usize; N];
        let mut fb = [0.0f32; N];
        let mut acc = 0usize;
        for i in 0..N {
            start[i] = acc;
            acc += DELAYS[i];
            // Per-line gain so a loop of DELAYS[i] samples decays 60 dB in rt60.
            fb[i] = libm::powf(10.0, -3.0 * DELAYS[i] as f32 / (rt60_s * sample_rate));
        }
        buf[..Self::REQUIRED_BUF].fill(0.0);
        Self {
            buf,
            start,
            delay: DELAYS,
            pos: [0; N],
            damp: [OnePole::lowpass(sample_rate, damping_hz); N],
            fb,
            dry: 1.0 - mix,
            wet: mix,
        }
    }

    /// Process a mono block into a decorrelated stereo pair. All three slices
    /// share length.
    pub fn process(&mut self, input: &[f32], out_l: &mut [f32], out_r: &mut [f32]) {
        for ((&x, ol), or) in input.iter().zip(out_l.iter_mut()).zip(out_r.iter_mut()) {
            // Read each line's delayed output (the slot about to be overwritten
            // holds the sample from exactly `delay` samples ago).
            let mut s = [0.0f32; N];
            for (i, si) in s.iter_mut().enumerate() {
                *si = self.buf[self.start[i] + self.pos[i]];
            }

            // Two orthogonal output taps → decorrelated L/R.
            let wet_l = 0.5 * (s[0] + s[1] - s[2] - s[3]);
            let wet_r = 0.5 * (s[0] - s[1] - s[2] + s[3]);

            // Damping + per-line decay in the feedback path.
            let mut t = [0.0f32; N];
            for (i, ti) in t.iter_mut().enumerate() {
                *ti = self.damp[i].tick(s[i]) * self.fb[i];
            }

            // Householder mix: v = t − (2/N)·Σt. Lossless, cheap.
            let h = (t[0] + t[1] + t[2] + t[3]) * (2.0 / N as f32);
            // Indexes four parallel arrays (buf via start+pos, pos, delay, t) and
            // advances each ring — an index loop is the clearest form here.
            #[allow(clippy::needless_range_loop)]
            for i in 0..N {
                self.buf[self.start[i] + self.pos[i]] = x + (t[i] - h);
                self.pos[i] += 1;
                if self.pos[i] == self.delay[i] {
                    self.pos[i] = 0;
                }
            }

            *ol = self.dry * x + self.wet * wet_l;
            *or = self.dry * x + self.wet * wet_r;
        }
    }

    /// Flush the delay lines and damping filters.
    pub fn reset(&mut self) {
        self.buf[..Self::REQUIRED_BUF].fill(0.0);
        self.pos = [0; N];
        for d in &mut self.damp {
            d.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Cheap, core-only sanity (the RT60 / decay properties are validated on the
    // host via the golden framework's property checks, which have alloc for the
    // multi-second impulse response).
    #[test]
    fn silence_in_silence_out() {
        let mut buf = [0.0f32; FdnReverb::REQUIRED_BUF];
        let mut rv = FdnReverb::new(&mut buf, 48_000.0, 1.5, 8000.0, 1.0);
        let zeros = [0.0f32; 256];
        let mut l = [0.0f32; 256];
        let mut r = [0.0f32; 256];
        rv.process(&zeros, &mut l, &mut r);
        assert!(l.iter().chain(&r).all(|&x| x == 0.0));
    }

    #[test]
    fn impulse_returns_energy_and_stays_finite() {
        let mut buf = [0.0f32; FdnReverb::REQUIRED_BUF];
        let mut rv = FdnReverb::new(&mut buf, 48_000.0, 1.5, 8000.0, 1.0);
        let mut input = [0.0f32; 4096];
        input[0] = 1.0;
        let mut l = [0.0f32; 4096];
        let mut r = [0.0f32; 4096];
        rv.process(&input, &mut l, &mut r);
        assert!(l.iter().chain(&r).all(|x| x.is_finite()));
        // Energy comes back after the shortest loop (1231 samples).
        assert!(l[DELAYS[0]..].iter().any(|x| x.abs() > 1e-6));
    }
}
