//! Waveshaping distortion (tanh / hard-clip / cubic soft-clip) with optional
//! **antiderivative antialiasing (ADAA)** — the economical alternative to
//! oversampling. `no_std`, no alloc.
//!
//! A memoryless nonlinearity `f(x)` generates harmonics that alias. First-order
//! ADAA replaces `f(x[n])` with the average of `f` over the segment from the
//! previous to the current sample, computed from the antiderivative `F`:
//! `y[n] = (F(x[n]) − F(x[n−1])) / (x[n] − x[n−1])`. That band-limits the
//! nonlinearity for a few flops/sample instead of an oversampled FIR.
//!
//! # References
//! - J. Parker, V. Zavalishin & E. Le Bivic, "Reducing the Aliasing of Nonlinear
//!   Waveshaping Using Continuous-Time Convolution," DAFx 2016. (ADAA)

/// Distortion transfer curve.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Shape {
    /// `tanh(x)` — smooth, tube-like saturation.
    Tanh,
    /// Hard clip to `[-1, 1]`.
    HardClip,
    /// Cubic soft clip `1.5x − 0.5x³` (saturating to ±1 at |x| ≥ 1).
    Cubic,
}

const LN_2: f32 = core::f32::consts::LN_2;

impl Shape {
    /// The shaping function `f(x)`.
    #[inline]
    pub fn f(self, x: f32) -> f32 {
        match self {
            Shape::Tanh => libm::tanhf(x),
            Shape::HardClip => x.clamp(-1.0, 1.0),
            Shape::Cubic => {
                if x <= -1.0 {
                    -1.0
                } else if x >= 1.0 {
                    1.0
                } else {
                    1.5 * x - 0.5 * x * x * x
                }
            }
        }
    }

    /// The antiderivative `F(x)` with `F(0)=0`.
    #[inline]
    fn big_f(self, x: f32) -> f32 {
        match self {
            // ∫tanh = ln cosh; use the |x|−ln2 asymptote to avoid cosh overflow.
            Shape::Tanh => {
                let a = x.abs();
                if a > 15.0 {
                    a - LN_2
                } else {
                    libm::logf(libm::coshf(x))
                }
            }
            Shape::HardClip => {
                let a = x.abs();
                if a <= 1.0 {
                    0.5 * x * x
                } else {
                    a - 0.5
                }
            }
            Shape::Cubic => {
                let a = x.abs();
                if a <= 1.0 {
                    0.75 * x * x - 0.125 * x * x * x * x
                } else {
                    a - 0.375
                }
            }
        }
    }
}

/// A waveshaper: pre-`drive` gain into a [`Shape`], optionally antialiased (ADAA).
#[derive(Copy, Clone)]
pub struct Waveshaper {
    shape: Shape,
    drive: f32,
    adaa: bool,
    x1: f32, // previous driven input
    f1: f32, // F(x1)
}

impl Waveshaper {
    pub fn new(shape: Shape, drive: f32, adaa: bool) -> Self {
        Self {
            shape,
            drive,
            adaa,
            x1: 0.0,
            f1: 0.0,
        }
    }

    #[inline]
    pub fn set_drive(&mut self, drive: f32) {
        self.drive = drive;
    }

    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        let xin = x * self.drive;
        if !self.adaa {
            return self.shape.f(xin);
        }
        let d = xin - self.x1;
        let y = if d.abs() < 1.0e-4 {
            // Segment too short for a stable divided difference — evaluate f at
            // the midpoint (the ADAA limit as Δ→0).
            self.shape.f(0.5 * (xin + self.x1))
        } else {
            (self.shape.big_f(xin) - self.f1) / d
        };
        self.x1 = xin;
        self.f1 = self.shape.big_f(xin);
        y
    }

    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.f1 = 0.0;
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

    #[test]
    fn static_curve_matches_the_shaping_function() {
        for &sh in &[Shape::Tanh, Shape::HardClip, Shape::Cubic] {
            let mut ws = Waveshaper::new(sh, 2.0, true);
            for &x in &[-3.0f32, -0.5, 0.0, 0.3, 1.0, 5.0] {
                // A held input drives Δ→0, so ADAA settles to f(drive·x).
                let mut y = 0.0;
                for _ in 0..8 {
                    y = ws.tick(x);
                }
                let want = sh.f(2.0 * x);
                assert!((y - want).abs() < 1e-2, "{sh:?} x={x}: {y} vs {want}");
            }
        }
    }

    #[test]
    fn outputs_are_bounded() {
        for &sh in &[Shape::Tanh, Shape::HardClip, Shape::Cubic] {
            let mut ws = Waveshaper::new(sh, 10.0, true);
            for i in 0..10_000 {
                let x = 3.0 * libm::sinf(TAU * 1234.0 * i as f32 / FS);
                let y = ws.tick(x);
                assert!(y.abs() <= 1.05, "{sh:?} unbounded: {y}");
            }
        }
    }

    // Alias energy from driving a nonlinearity hard: f0 = 100·fs/N is periodic in
    // the frame (leakage-free) with non-integer fs/f0, so folded harmonics land on
    // non-multiple-of-100 bins. ADAA must reduce that vs the naive shaper.
    fn shaper_alias_energy(adaa: bool) -> f64 {
        const N: usize = 2048;
        let bin_per_h = 100usize;
        let f0 = bin_per_h as f32 * FS / N as f32;
        let mut ws = Waveshaper::new(Shape::HardClip, 4.0, adaa);
        let mut buf = [0.0f32; N];
        for (i, b) in buf.iter_mut().enumerate() {
            *b = ws.tick(0.9 * libm::sinf(TAU * f0 * i as f32 / FS));
        }
        let (mut sre, mut sim) = (std::vec![0.0f32; N / 2], std::vec![0.0f32; N / 2]);
        let (mut re, mut im) = (std::vec![0.0f32; N / 2 + 1], std::vec![0.0f32; N / 2 + 1]);
        RealFft::new(&mut sre, &mut sim).forward(&buf, &mut re, &mut im);
        let mut alias = 0.0f64;
        for k in 1..N / 2 {
            if k % bin_per_h != 0 {
                alias += (re[k] * re[k] + im[k] * im[k]) as f64;
            }
        }
        alias
    }

    #[test]
    fn adaa_reduces_aliasing() {
        let naive = shaper_alias_energy(false);
        let adaa = shaper_alias_energy(true);
        // First-order ADAA gives a clear reduction (several dB) without oversampling.
        assert!(
            adaa < naive * 0.6,
            "alias: ADAA {adaa:.3e} vs naive {naive:.3e}"
        );
    }
}
