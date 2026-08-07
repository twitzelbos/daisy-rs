//! Filters. All coefficients are computed at construction from a sample rate;
//! `tick` runs the per-sample recurrence. Validated against scipy (see
//! `tools/dsp-golden`): the f32 output tracks the ideal transfer function to
//! well within single-precision roundoff.

use core::f32::consts::PI;

/// A one-pole smoothing filter (leaky integrator). Cheap; ~6 dB/oct.
///
/// Low-pass: `y[n] = (1−a)·x[n] + a·y[n−1]`, with `a = exp(−2π·fc/fs)`.
/// High-pass is the complementary `x − lowpass(x)`.
#[derive(Copy, Clone)]
pub struct OnePole {
    a: f32, // pole coefficient
    b: f32, // 1 − a
    z: f32, // low-pass state
    high: bool,
}

impl OnePole {
    /// Low-pass at `cutoff` Hz.
    pub fn lowpass(sample_rate: f32, cutoff: f32) -> Self {
        let a = libm::expf(-2.0 * PI * cutoff / sample_rate);
        Self {
            a,
            b: 1.0 - a,
            z: 0.0,
            high: false,
        }
    }

    /// High-pass at `cutoff` Hz (complementary to the low-pass).
    pub fn highpass(sample_rate: f32, cutoff: f32) -> Self {
        let mut f = Self::lowpass(sample_rate, cutoff);
        f.high = true;
        f
    }

    /// Process one sample.
    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        self.z = self.b * x + self.a * self.z;
        if self.high {
            x - self.z
        } else {
            self.z
        }
    }

    /// Process a block into `out` (may alias `input`).
    pub fn process(&mut self, input: &[f32], out: &mut [f32]) {
        for (o, &x) in out.iter_mut().zip(input) {
            *o = self.tick(x);
        }
    }

    /// Flush the state.
    pub fn reset(&mut self) {
        self.z = 0.0;
    }
}

/// The response shape of a [`Biquad`].
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum BiquadKind {
    Lowpass,
    Highpass,
    Bandpass, // constant 0 dB peak gain
    Peaking,
}

/// A biquad (second-order) filter using the RBJ "Audio EQ Cookbook"
/// coefficients, run as Transposed Direct Form II.
#[derive(Copy, Clone)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

impl Biquad {
    /// Build any [`BiquadKind`] from `freq`/`q` (`gain_db` only used by
    /// `Peaking`). RBJ cookbook, normalised by `a0`.
    pub fn new(sample_rate: f32, kind: BiquadKind, freq: f32, q: f32, gain_db: f32) -> Self {
        let w0 = 2.0 * PI * freq / sample_rate;
        let cw = libm::cosf(w0);
        let sw = libm::sinf(w0);
        let alpha = sw / (2.0 * q);
        let a = libm::powf(10.0, gain_db / 40.0); // sqrt of linear gain, for peaking

        let (b0, b1, b2, a0, a1, a2) = match kind {
            BiquadKind::Lowpass => {
                let b1 = 1.0 - cw;
                (
                    (1.0 - cw) * 0.5,
                    b1,
                    (1.0 - cw) * 0.5,
                    1.0 + alpha,
                    -2.0 * cw,
                    1.0 - alpha,
                )
            }
            BiquadKind::Highpass => {
                let b1 = -(1.0 + cw);
                (
                    (1.0 + cw) * 0.5,
                    b1,
                    (1.0 + cw) * 0.5,
                    1.0 + alpha,
                    -2.0 * cw,
                    1.0 - alpha,
                )
            }
            BiquadKind::Bandpass => (alpha, 0.0, -alpha, 1.0 + alpha, -2.0 * cw, 1.0 - alpha),
            BiquadKind::Peaking => (
                1.0 + alpha * a,
                -2.0 * cw,
                1.0 - alpha * a,
                1.0 + alpha / a,
                -2.0 * cw,
                1.0 - alpha / a,
            ),
        };

        let inv = 1.0 / a0;
        Self {
            b0: b0 * inv,
            b1: b1 * inv,
            b2: b2 * inv,
            a1: a1 * inv,
            a2: a2 * inv,
            z1: 0.0,
            z2: 0.0,
        }
    }

    /// Low-pass at `freq` Hz, resonance `q` (0.707 = Butterworth).
    pub fn lowpass(sample_rate: f32, freq: f32, q: f32) -> Self {
        Self::new(sample_rate, BiquadKind::Lowpass, freq, q, 0.0)
    }
    /// High-pass at `freq` Hz, resonance `q`.
    pub fn highpass(sample_rate: f32, freq: f32, q: f32) -> Self {
        Self::new(sample_rate, BiquadKind::Highpass, freq, q, 0.0)
    }
    /// Band-pass centred at `freq` Hz (0 dB peak gain), bandwidth from `q`.
    pub fn bandpass(sample_rate: f32, freq: f32, q: f32) -> Self {
        Self::new(sample_rate, BiquadKind::Bandpass, freq, q, 0.0)
    }
    /// Peaking EQ: `±gain_db` at `freq`, width from `q`.
    pub fn peaking(sample_rate: f32, freq: f32, q: f32, gain_db: f32) -> Self {
        Self::new(sample_rate, BiquadKind::Peaking, freq, q, gain_db)
    }

    /// Process one sample (Transposed Direct Form II).
    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }

    /// Process a block into `out` (may alias `input`).
    pub fn process(&mut self, input: &[f32], out: &mut [f32]) {
        for (o, &x) in out.iter_mut().zip(input) {
            *o = self.tick(x);
        }
    }

    /// Flush the state.
    pub fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }
}
