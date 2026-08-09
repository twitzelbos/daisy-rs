//! A feedback-delay-network (FDN) reverb — the shared "room / atmosphere"
//! engine for the spatializer and the pad/drone generator.
//!
//! Economical, but dense enough to be smooth on *real* (rich, transient)
//! material, not just steady tones:
//! - **Input diffusion** — a chain of 4 Schroeder all-pass filters smears
//!   transients before they hit the network, so the tail has no flutter echo.
//! - **8 delay lines** in one borrowed buffer (place it in SDRAM/AXI SRAM) —
//!   enough modal density that the tail is a wash, not a metallic ring. (A
//!   4-line FDN sounds fine on a sine but flutters on a choir.)
//! - A **Householder** feedback matrix `A = I − (2/N)·11ᵀ`, lossless yet just a
//!   sum + a subtract per line — **no matrix multiply**.
//! - One **one-pole** damping filter per line (HF decays faster — natural).
//! - Decay gains + damping cutoffs computed **once** at construction; the
//!   per-sample loop has no `exp`/`sin`/`pow` and wraps rings with a compare.
//!
//! Mono in, decorrelated **stereo** out (two orthogonal Hadamard taps), dry/wet.
//!
//! References (see `docs/references.md`): FDN + Householder feedback matrix —
//! Jot & Chaigne, "Digital delay networks for designing artificial
//! reverberators" (AES 1991); input-diffusion all-passes — Schroeder (JAES
//! 1962) & Dattorro, "Effect Design Part 1" (JAES 1997); delay-line modulation
//! (chorused tank, anti-metallic) — Dattorro 1997 / Griesinger; RT60 decay
//! gains `g = 10^(−3·D/(RT60·fs))` — Jot.

use crate::filter::OnePole;

const N: usize = 8;
const NDIFF: usize = 4;

/// FDN loop lengths (samples) — mutually prime primes, ~21–35 ms at 48 kHz.
const DELAYS: [usize; N] = [1009, 1103, 1201, 1301, 1399, 1499, 1601, 1699];

/// Input-diffuser all-pass delays (samples) — short, mutually prime.
const DIFF_DELAYS: [usize; NDIFF] = [149, 211, 293, 379];

/// All-pass diffusion coefficient.
const DIFF_G: f32 = 0.7;

/// An 8-line FDN reverb with input diffusion, over a caller-provided buffer.
pub struct FdnReverb<'a> {
    buf: &'a mut [f32],
    start: [usize; N],
    delay: [usize; N],
    pos: [usize; N],
    damp: [OnePole; N],
    fb: [f32; N],
    diff_start: [usize; NDIFF],
    diff_delay: [usize; NDIFF],
    diff_pos: [usize; NDIFF],
    // Per-line read-position modulation (magic-circle LFO: 2 mults/sample, no
    // sin) — wobbles each delay so the modes never lock into a metallic ring.
    mod_x: [f32; N],
    mod_y: [f32; N],
    mod_eps: [f32; N],
    mod_depth: f32,
    dry: f32,
    wet: f32,
}

const fn sum<const M: usize>(a: [usize; M]) -> usize {
    let mut s = 0;
    let mut i = 0;
    while i < M {
        s += a[i];
        i += 1;
    }
    s
}

impl<'a> FdnReverb<'a> {
    /// Minimum backing-buffer length (samples) — size the buffer to at least this.
    pub const REQUIRED_BUF: usize = sum(DELAYS) + sum(DIFF_DELAYS);

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
        let mut diff_start = [0usize; NDIFF];
        for k in 0..NDIFF {
            diff_start[k] = acc;
            acc += DIFF_DELAYS[k];
        }
        // Per-line modulation LFOs: slow (0.6–1.6 Hz), decorrelated rates/phases.
        let mut mod_x = [0.0f32; N];
        let mut mod_y = [0.0f32; N];
        let mut mod_eps = [0.0f32; N];
        for i in 0..N {
            let rate = 0.6 + 0.14 * i as f32;
            mod_eps[i] = 2.0 * libm::sinf(core::f32::consts::PI * rate / sample_rate);
            let ph = i as f32 * 0.79;
            mod_x[i] = libm::cosf(ph);
            mod_y[i] = libm::sinf(ph);
        }
        buf[..Self::REQUIRED_BUF].fill(0.0);
        Self {
            buf,
            start,
            delay: DELAYS,
            pos: [0; N],
            damp: [OnePole::lowpass(sample_rate, damping_hz); N],
            fb,
            diff_start,
            diff_delay: DIFF_DELAYS,
            diff_pos: [0; NDIFF],
            mod_x,
            mod_y,
            mod_eps,
            mod_depth: 5.0,
            dry: 1.0 - mix,
            wet: mix,
        }
    }

    /// Process a mono block into a decorrelated stereo pair. All three slices
    /// share length.
    pub fn process(&mut self, input: &[f32], out_l: &mut [f32], out_r: &mut [f32]) {
        for ((&x, ol), or) in input.iter().zip(out_l.iter_mut()).zip(out_r.iter_mut()) {
            // Input diffusion: a series of Schroeder all-passes smears transients
            // so the network never flutter-echoes.
            let mut xd = x;
            #[allow(clippy::needless_range_loop)]
            for k in 0..NDIFF {
                let idx = self.diff_start[k] + self.diff_pos[k];
                let bufout = self.buf[idx];
                let v = xd + DIFF_G * bufout;
                self.buf[idx] = v;
                xd = bufout - DIFF_G * v;
                self.diff_pos[k] += 1;
                if self.diff_pos[k] == self.diff_delay[k] {
                    self.diff_pos[k] = 0;
                }
            }

            // Read each line's delayed output at a slowly-modulated (fractional)
            // position — this chorusing keeps the modes from ringing metallically.
            let mut s = [0.0f32; N];
            #[allow(clippy::needless_range_loop)]
            for i in 0..N {
                self.mod_x[i] += self.mod_eps[i] * self.mod_y[i];
                self.mod_y[i] -= self.mod_eps[i] * self.mod_x[i];
                let d = self.delay[i];
                let mut rp = self.pos[i] as f32 + self.mod_depth * 0.5 * (self.mod_x[i] + 1.0);
                if rp >= d as f32 {
                    rp -= d as f32;
                }
                let i0 = rp as usize;
                let frac = rp - i0 as f32;
                let i1 = if i0 + 1 >= d { 0 } else { i0 + 1 };
                let base = self.start[i];
                s[i] = self.buf[base + i0] + (self.buf[base + i1] - self.buf[base + i0]) * frac;
            }

            // Two orthogonal Hadamard taps → decorrelated L/R.
            let wet_l = 0.354 * (s[0] + s[1] + s[2] + s[3] - s[4] - s[5] - s[6] - s[7]);
            let wet_r = 0.354 * (s[0] - s[1] + s[2] - s[3] - s[4] + s[5] - s[6] + s[7]);

            // Damping + per-line decay in the feedback path.
            let mut t = [0.0f32; N];
            for (i, ti) in t.iter_mut().enumerate() {
                *ti = self.damp[i].tick(s[i]) * self.fb[i];
            }

            // Householder mix: v = t − (2/N)·Σt. Lossless, cheap.
            let mut acc = 0.0f32;
            for &ti in &t {
                acc += ti;
            }
            let h = acc * (2.0 / N as f32);
            // Inject the diffused input into every line, advance the rings.
            #[allow(clippy::needless_range_loop)]
            for i in 0..N {
                self.buf[self.start[i] + self.pos[i]] = xd + (t[i] - h);
                self.pos[i] += 1;
                if self.pos[i] == self.delay[i] {
                    self.pos[i] = 0;
                }
            }

            *ol = self.dry * x + self.wet * wet_l;
            *or = self.dry * x + self.wet * wet_r;
        }
    }

    /// Flush the delay lines, diffusers, and damping filters.
    pub fn reset(&mut self) {
        self.buf[..Self::REQUIRED_BUF].fill(0.0);
        self.pos = [0; N];
        self.diff_pos = [0; NDIFF];
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
        // Energy comes back after the shortest loop.
        assert!(l[DELAYS[0]..].iter().any(|x| x.abs() > 1e-6));
    }
}
