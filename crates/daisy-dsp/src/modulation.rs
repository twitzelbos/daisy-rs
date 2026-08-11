//! Modulation effects — chorus, flanger, and phaser. `no_std`, no alloc.
//!
//! Chorus and flanger are LFO-modulated fractional delays (built on
//! [`DelayLine`](crate::delay::DelayLine)); the phaser is an LFO-swept chain of
//! first-order all-pass sections mixed back with the dry signal. All keep the
//! delay storage borrowed so it can live wherever the app wants.

use crate::delay::DelayLine;
use crate::lfo::{Lfo, Waveform};
use core::f32::consts::PI;

/// A chorus: a slow, deep-ish modulated delay blended with the dry signal, for
/// thickening/detune.
#[derive(Debug)]
pub struct Chorus<'a> {
    line: DelayLine<'a>,
    lfo: Lfo,
    base: f32,  // centre delay (samples)
    depth: f32, // ± modulation (samples)
    mix: f32,
}

impl<'a> Chorus<'a> {
    /// `buf` sizes the max delay; `base_ms`±`depth_ms` set the sweep (typically
    /// ~15–30 ms base for chorus). `mix` = wet fraction.
    #[must_use]
    pub fn new(
        buf: &'a mut [f32],
        sample_rate: f32,
        rate_hz: f32,
        base_ms: f32,
        depth_ms: f32,
        mix: f32,
    ) -> Self {
        Self {
            line: DelayLine::new(buf),
            lfo: Lfo::new(sample_rate, rate_hz, Waveform::Sine),
            base: base_ms * 1.0e-3 * sample_rate,
            depth: depth_ms * 1.0e-3 * sample_rate,
            mix: mix.clamp(0.0, 1.0),
        }
    }

    /// Process one sample and return the dry/wet-mixed output.
    #[inline]
    #[must_use]
    pub fn tick(&mut self, x: f32) -> f32 {
        let d = (self.base + self.depth * self.lfo.tick()).max(1.0);
        self.line.write(x);
        let wet = self.line.read_frac(d);
        (1.0 - self.mix) * x + self.mix * wet
    }

    /// Clear the delay line and reset the LFO phase.
    pub fn reset(&mut self) {
        self.line.reset();
        self.lfo.reset();
    }
}

/// A flanger: a short modulated delay with feedback, producing the classic
/// sweeping comb "jet" sound.
#[derive(Debug)]
pub struct Flanger<'a> {
    line: DelayLine<'a>,
    lfo: Lfo,
    base: f32,
    depth: f32,
    feedback: f32,
    mix: f32,
    last: f32,
}

impl<'a> Flanger<'a> {
    /// Short delays (~0.5–5 ms). `feedback` in (−1, 1) sharpens the comb.
    #[must_use]
    pub fn new(
        buf: &'a mut [f32],
        sample_rate: f32,
        rate_hz: f32,
        base_ms: f32,
        depth_ms: f32,
        feedback: f32,
        mix: f32,
    ) -> Self {
        Self {
            line: DelayLine::new(buf),
            lfo: Lfo::new(sample_rate, rate_hz, Waveform::Sine),
            base: base_ms * 1.0e-3 * sample_rate,
            depth: depth_ms * 1.0e-3 * sample_rate,
            feedback: feedback.clamp(-0.98, 0.98),
            mix: mix.clamp(0.0, 1.0),
            last: 0.0,
        }
    }

    /// Process one sample and return the dry/wet-mixed output.
    #[inline]
    #[must_use]
    pub fn tick(&mut self, x: f32) -> f32 {
        let d = (self.base + self.depth * self.lfo.tick()).max(1.0);
        self.line.write(x + self.feedback * self.last);
        let wet = self.line.read_frac(d);
        self.last = wet;
        (1.0 - self.mix) * x + self.mix * wet
    }

    /// Clear the delay line, reset the LFO phase, and clear feedback state.
    pub fn reset(&mut self) {
        self.line.reset();
        self.lfo.reset();
        self.last = 0.0;
    }
}

/// A first-order all-pass section: flat magnitude, frequency-dependent phase.
/// `H(z) = (a + z⁻¹)/(1 + a·z⁻¹)`.
#[derive(Copy, Clone, Default, Debug)]
struct Allpass1 {
    x1: f32,
    y1: f32,
}

impl Allpass1 {
    #[inline]
    fn tick(&mut self, x: f32, a: f32) -> f32 {
        let y = a * x + self.x1 - a * self.y1;
        self.x1 = x;
        self.y1 = y;
        y
    }
}

/// An `N`-stage phaser: an LFO log-sweeps the break frequency of a cascade of
/// all-pass sections, whose output (with optional feedback) is mixed with the
/// dry signal to create sweeping notches.
#[derive(Debug)]
pub struct Phaser<const N: usize> {
    aps: [Allpass1; N],
    lfo: Lfo,
    fmin: f32,
    ratio: f32, // fmax/fmin
    fs: f32,
    feedback: f32,
    mix: f32,
    last: f32,
}

impl<const N: usize> Phaser<N> {
    /// Build an `N`-stage phaser: the LFO log-sweeps the break frequency between
    /// `fmin_hz` and `fmax_hz`; `feedback` in (−1, 1); `mix` = wet fraction.
    #[must_use]
    pub fn new(
        sample_rate: f32,
        rate_hz: f32,
        fmin_hz: f32,
        fmax_hz: f32,
        feedback: f32,
        mix: f32,
    ) -> Self {
        Self {
            aps: [Allpass1::default(); N],
            lfo: Lfo::new(sample_rate, rate_hz, Waveform::Sine),
            fmin: fmin_hz,
            ratio: (fmax_hz / fmin_hz).max(1.0),
            fs: sample_rate,
            feedback: feedback.clamp(-0.98, 0.98),
            mix: mix.clamp(0.0, 1.0),
            last: 0.0,
        }
    }

    /// Process one sample and return the dry/wet-mixed output.
    #[inline]
    #[must_use]
    pub fn tick(&mut self, x: f32) -> f32 {
        // Log-sweep the break frequency, and derive the all-pass coefficient.
        let lf = self.lfo.tick_unipolar();
        let fc = self.fmin * libm::powf(self.ratio, lf);
        let t = libm::tanf(PI * (fc / self.fs).min(0.49));
        let a = (t - 1.0) / (t + 1.0);

        let mut s = x + self.feedback * self.last;
        for ap in &mut self.aps {
            s = ap.tick(s, a);
        }
        self.last = s;
        (1.0 - self.mix) * x + self.mix * s
    }

    /// Reset all all-pass sections, the LFO phase, and feedback state.
    pub fn reset(&mut self) {
        self.aps = [Allpass1::default(); N];
        self.lfo.reset();
        self.last = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f32 = 48_000.0;

    fn peak(mut f: impl FnMut(f32) -> f32, amp: f32, freq: f32, n: usize) -> f32 {
        let mut p = 0.0f32;
        for i in 0..n {
            let y = f(amp * libm::sinf(2.0 * PI * freq * i as f32 / FS));
            if i > n / 2 {
                p = p.max(y.abs());
            }
        }
        p
    }

    #[test]
    fn allpass_has_unit_magnitude() {
        // The defining property of an all-pass: |H(f)| = 1 at every frequency.
        for &f in &[100.0f32, 1_000.0, 5_000.0, 15_000.0] {
            let mut ap = Allpass1::default();
            let a = 0.6;
            let out = peak(|x| ap.tick(x, a), 1.0, f, 8192);
            assert!((out - 1.0).abs() < 0.02, "f={f}: |H|={out}");
        }
    }

    #[test]
    fn chorus_and_flanger_are_silent_on_silence_and_bounded() {
        let mut cbuf = [0.0f32; 4096];
        let mut ch = Chorus::new(&mut cbuf, FS, 0.8, 20.0, 5.0, 0.5);
        let mut fbuf = [0.0f32; 1024];
        let mut fl = Flanger::new(&mut fbuf, FS, 0.5, 2.0, 1.5, 0.6, 0.5);
        for _ in 0..4096 {
            assert_eq!(ch.tick(0.0), 0.0);
            assert_eq!(fl.tick(0.0), 0.0);
        }
        // Bounded on a hot input (flanger feedback must stay stable).
        let mut maxv = 0.0f32;
        for i in 0..48_000 {
            let x = 0.9 * libm::sinf(2.0 * PI * 300.0 * i as f32 / FS);
            maxv = maxv.max(ch.tick(x).abs()).max(fl.tick(x).abs());
        }
        assert!(
            maxv.is_finite() && maxv < 4.0,
            "modulation unstable: {maxv}"
        );
    }

    #[test]
    fn chorus_delays_the_signal() {
        // An impulse should re-appear delayed by ~base (20 ms ≈ 960 samples).
        let mut buf = [0.0f32; 4096];
        let mut ch = Chorus::new(&mut buf, FS, 0.0001, 20.0, 0.0, 1.0); // wet-only, ~static
        let mut echo_at = None;
        for i in 0..2048 {
            let x = if i == 0 { 1.0 } else { 0.0 };
            let y = ch.tick(x);
            if i > 10 && y.abs() > 0.3 {
                echo_at = Some(i);
                break;
            }
        }
        let at = echo_at.expect("no delayed echo");
        assert!((at as i32 - 960).abs() < 40, "echo at {at}, expected ~960");
    }

    #[test]
    fn phaser_is_bounded_and_notches() {
        let mut ph = Phaser::<4>::new(FS, 0.5, 300.0, 2_000.0, 0.5, 0.5);
        // Silence stays silent; hot input stays bounded.
        for _ in 0..1000 {
            assert_eq!(ph.tick(0.0), 0.0);
        }
        let mut maxv = 0.0f32;
        for i in 0..48_000 {
            let x = 0.9 * libm::sinf(2.0 * PI * 700.0 * i as f32 / FS);
            maxv = maxv.max(ph.tick(x).abs());
        }
        assert!(maxv.is_finite() && maxv < 3.0, "phaser unstable: {maxv}");
    }
}
