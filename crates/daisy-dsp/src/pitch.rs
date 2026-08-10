//! Pitch effects — a real-time pitch shifter and a classic octaver. `no_std`.
//!
//! [`PitchShifter`] is the economical time-domain shifter: two crossfading read
//! taps sweep a delay line at the target speed ratio (no FFT), the workhorse for
//! shimmer, harmony, and octave voices. [`Octaver`] is the analog-style pedal
//! octaver — a flip-flop sub-octave plus a rectifier octave-up.
//!
//! # References
//! - S. Disch & U. Zölzer, "Modulation and delay line based digital audio
//!   effects" (crossfading delay-line pitch shifting), DAFx 1999.

use crate::delay::DelayLine;
use crate::dynamics::EnvelopeFollower;
use crate::filter::OnePole;
use core::f32::consts::PI;

#[inline]
fn frac(x: f32) -> f32 {
    x - libm::floorf(x)
}

/// A time-domain pitch shifter: two raised-cosine-crossfaded taps read the delay
/// line at `ratio` × the write speed, so the output pitch is `ratio` × the input
/// (2.0 = up an octave, 0.5 = down an octave). Borrowed delay storage sets the
/// crossfade window (longer buffer = smoother, more latency).
pub struct PitchShifter<'a> {
    line: DelayLine<'a>,
    ratio_term: f32, // (1 − ratio)/window, the per-sample phase increment
    window: f32,
    base: f32,
    phase: f32,
}

impl<'a> PitchShifter<'a> {
    pub fn new(buf: &'a mut [f32], ratio: f32) -> Self {
        let len = buf.len();
        assert!(len >= 64, "pitch buffer too small");
        let window = (len as f32 * 0.5).max(8.0);
        Self {
            line: DelayLine::new(buf),
            ratio_term: (1.0 - ratio.max(0.05)) / window,
            window,
            base: 2.0,
            phase: 0.0,
        }
    }

    /// Retune the shift ratio (output/input pitch).
    #[inline]
    pub fn set_ratio(&mut self, ratio: f32) {
        self.ratio_term = (1.0 - ratio.max(0.05)) / self.window;
    }

    /// Shift a semitone amount (±): `2^(semitones/12)`.
    pub fn set_semitones(&mut self, semitones: f32) {
        self.set_ratio(libm::exp2f(semitones / 12.0));
    }

    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        self.line.write(x);
        let p1 = self.phase;
        let p2 = frac(self.phase + 0.5);
        // Equal-power raised-cosine crossfade (g1²+g2² = 1).
        let g1 = libm::sinf(PI * p1);
        let g2 = libm::sinf(PI * p2);
        let d1 = self.base + p1 * self.window;
        let d2 = self.base + p2 * self.window;
        let out = g1 * self.line.read_frac(d1) + g2 * self.line.read_frac(d2);

        self.phase = frac(self.phase + self.ratio_term);
        out
    }

    pub fn reset(&mut self) {
        self.line.reset();
        self.phase = 0.0;
    }
}

/// A classic octaver: a **sub-octave** (a flip-flop toggled on rising zero
/// crossings → a square at half the input frequency, amplitude-tracked) plus an
/// **octave-up** (full-wave rectification, which doubles the fundamental, with
/// the DC removed). Each voice and the dry signal has its own level.
pub struct Octaver {
    flip: f32,
    prev: f32,
    env: EnvelopeFollower,
    hp: OnePole,
    dry: f32,
    sub: f32,
    up: f32,
}

impl Octaver {
    pub fn new(sample_rate: f32, dry: f32, sub_level: f32, up_level: f32) -> Self {
        Self {
            flip: 1.0,
            prev: 0.0,
            env: EnvelopeFollower::new(sample_rate, 0.002, 0.02),
            hp: OnePole::highpass(sample_rate, 30.0),
            dry,
            sub: sub_level,
            up: up_level,
        }
    }

    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        // Sub-octave: toggle on each rising zero crossing → half-frequency square.
        if self.prev <= 0.0 && x > 0.0 {
            self.flip = -self.flip;
        }
        self.prev = x;
        let e = self.env.tick(x);
        let sub = self.flip * e;

        // Octave-up: |x| has a fundamental at 2f; strip the DC the rectifier adds.
        let up = self.hp.tick(libm::fabsf(x));

        self.dry * x + self.sub * sub + self.up * up
    }

    pub fn reset(&mut self) {
        self.flip = 1.0;
        self.prev = 0.0;
        self.env.reset();
        self.hp.reset();
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fft::RealFft;
    use core::f32::consts::TAU;
    use std::vec::Vec;

    const FS: f32 = 48_000.0;

    // Dominant frequency of a signal via its largest FFT bin.
    fn dominant_freq(sig: &[f32]) -> f32 {
        const N: usize = 4096;
        let mut buf = [0.0f32; N];
        for (i, b) in buf.iter_mut().enumerate() {
            let w = 0.5 - 0.5 * libm::cosf(TAU * i as f32 / N as f32); // Hann
            *b = sig[sig.len() - N + i] * w;
        }
        let (mut sre, mut sim) = (std::vec![0.0f32; N / 2], std::vec![0.0f32; N / 2]);
        let (mut re, mut im) = (std::vec![0.0f32; N / 2 + 1], std::vec![0.0f32; N / 2 + 1]);
        RealFft::new(&mut sre, &mut sim).forward(&buf, &mut re, &mut im);
        let mut best = 1usize;
        let mut bestp = 0.0f32;
        for k in 1..N / 2 {
            let p = re[k] * re[k] + im[k] * im[k];
            if p > bestp {
                bestp = p;
                best = k;
            }
        }
        best as f32 * FS / N as f32
    }

    #[test]
    fn pitch_shifter_moves_the_fundamental() {
        for &(ratio, f0) in &[(2.0f32, 300.0f32), (0.5, 600.0), (1.5, 400.0)] {
            let mut buf = [0.0f32; 4096];
            let mut ps = PitchShifter::new(&mut buf, ratio);
            let out: Vec<f32> = (0..24_000)
                .map(|i| ps.tick(0.5 * libm::sinf(TAU * f0 * i as f32 / FS)))
                .collect();
            let dom = dominant_freq(&out);
            let cents = 1200.0 * libm::log2f(dom / (f0 * ratio));
            assert!(
                cents.abs() < 60.0,
                "ratio={ratio} f0={f0}: dom={dom}, want {}",
                f0 * ratio
            );
        }
    }

    #[test]
    fn octaver_sub_is_half_and_up_is_double() {
        let f0 = 400.0;
        // Sub-octave voice only.
        let mut sub = Octaver::new(FS, 0.0, 1.0, 0.0);
        let s: Vec<f32> = (0..12_000)
            .map(|i| sub.tick(0.5 * libm::sinf(TAU * f0 * i as f32 / FS)))
            .collect();
        let ds = dominant_freq(&s);
        assert!(
            (ds - f0 / 2.0).abs() < 15.0,
            "sub dominant {ds}, want {}",
            f0 / 2.0
        );

        // Octave-up voice only.
        let mut up = Octaver::new(FS, 0.0, 0.0, 1.0);
        let u: Vec<f32> = (0..12_000)
            .map(|i| up.tick(0.5 * libm::sinf(TAU * f0 * i as f32 / FS)))
            .collect();
        let du = dominant_freq(&u);
        assert!(
            (du - 2.0 * f0).abs() < 15.0,
            "up dominant {du}, want {}",
            2.0 * f0
        );
    }
}
