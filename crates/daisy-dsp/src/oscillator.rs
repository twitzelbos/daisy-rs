//! Band-limited oscillator (sine / saw / square / triangle) using **polyBLEP**
//! — a cheap polynomial correction applied at each waveform discontinuity that
//! suppresses the aliasing a naive `2·phase−1` saw would produce, without the
//! cost of oversampling or wavetables. `no_std`, no alloc.
//!
//! # References
//! - V. Välimäki & A. Huovilainen, "Antialiasing Oscillators in Subtractive
//!   Synthesis," *IEEE Signal Processing Magazine* 24(2), 2007. (BLEP)
//! - T. Stilson & J. Smith, "Alias-Free Digital Synthesis of Classic Analog
//!   Waveforms," ICMC 1996.

use core::f32::consts::TAU;

/// Oscillator waveform.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Waveform {
    /// Pure sine.
    Sine,
    /// Band-limited sawtooth.
    Saw,
    /// Band-limited square.
    Square,
    /// Band-limited triangle.
    Triangle,
}

/// A single-voice band-limited oscillator. Drive it with [`tick`](Self::tick).
#[derive(Debug, Copy, Clone)]
pub struct Oscillator {
    phase: f32, // [0, 1)
    dt: f32,    // phase increment per sample = f/fs
    wave: Waveform,
    tri: f32, // triangle integrator state
}

/// polyBLEP residual for a unit-amplitude step discontinuity at phase 0/1.
#[inline]
fn poly_blep(mut t: f32, dt: f32) -> f32 {
    if t < dt {
        t /= dt;
        t + t - t * t - 1.0
    } else if t > 1.0 - dt {
        t = (t - 1.0) / dt;
        t * t + t + t + 1.0
    } else {
        0.0
    }
}

#[inline]
fn frac(x: f32) -> f32 {
    x - libm::floorf(x)
}

impl Oscillator {
    /// Build an oscillator at `freq` (Hz) with the given `wave`.
    #[must_use]
    pub fn new(sample_rate: f32, freq: f32, wave: Waveform) -> Self {
        Self {
            phase: 0.0,
            dt: freq / sample_rate,
            wave,
            tri: 0.0,
        }
    }

    /// Retune (Hz).
    #[inline]
    pub fn set_freq(&mut self, sample_rate: f32, freq: f32) {
        self.dt = freq / sample_rate;
    }

    /// Generate one sample in roughly `[-1, 1]`.
    #[inline]
    #[must_use]
    pub fn tick(&mut self) -> f32 {
        let p = self.phase;
        let dt = self.dt;
        let y = match self.wave {
            Waveform::Sine => libm::sinf(TAU * p),
            Waveform::Saw => (2.0 * p - 1.0) - poly_blep(p, dt),
            Waveform::Square => {
                let naive = if p < 0.5 { 1.0 } else { -1.0 };
                naive + poly_blep(p, dt) - poly_blep(frac(p + 0.5), dt)
            }
            Waveform::Triangle => {
                // Band-limited square, then leaky-integrate to a triangle. The
                // leak (0.999) blocks the integrator's DC drift.
                let naive = if p < 0.5 { 1.0 } else { -1.0 };
                let sq = naive + poly_blep(p, dt) - poly_blep(frac(p + 0.5), dt);
                self.tri = 0.999 * self.tri + 4.0 * dt * sq;
                self.tri
            }
        };
        self.phase += dt;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }
        y
    }

    /// Reset the phase and triangle-integrator state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.tri = 0.0;
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fft::RealFft;
    use std::vec::Vec;

    const FS: f32 = 48_000.0;

    /// Fundamental frequency estimate via mean zero-crossing spacing (rising).
    fn measured_freq(buf: &[f32]) -> f32 {
        let mut crossings = 0usize;
        let mut first = None;
        let mut last = 0usize;
        for i in 1..buf.len() {
            if buf[i - 1] <= 0.0 && buf[i] > 0.0 {
                if first.is_none() {
                    first = Some(i);
                }
                last = i;
                crossings += 1;
            }
        }
        let periods = (crossings.saturating_sub(1)) as f32;
        let span = (last - first.unwrap()) as f32;
        periods / (span / FS)
    }

    #[test]
    fn frequency_is_accurate_for_every_waveform() {
        for &w in &[
            Waveform::Sine,
            Waveform::Saw,
            Waveform::Square,
            Waveform::Triangle,
        ] {
            let mut osc = Oscillator::new(FS, 440.0, w);
            let buf: Vec<f32> = (0..48_000).map(|_| osc.tick()).collect();
            let f = measured_freq(&buf);
            assert!((f - 440.0).abs() < 2.0, "{w:?}: measured {f} Hz");
            assert!(
                buf.iter().all(|v| v.is_finite() && v.abs() < 2.0),
                "{w:?} out of range"
            );
        }
    }

    // Alias energy: choose f0 = 100·fs/N so the tone is exactly periodic in the
    // frame (leakage-free, no window) BUT fs/f0 = 20.48 is non-integer, so a
    // naive saw's folded harmonics land BETWEEN the harmonic bins (multiples of
    // 100). Summing the non-multiple-of-100 bins is then pure aliasing. polyBLEP
    // must crush it.
    fn saw_alias_energy(blep: bool) -> f64 {
        const N: usize = 2048;
        let bin_per_h = 100usize; // harmonics land on bins 100,200,…
        let f0 = bin_per_h as f32 * FS / N as f32; // 2343.75 Hz
        let mut phase = 0.0f32;
        let mut osc = Oscillator::new(FS, f0, Waveform::Saw);
        let mut buf = [0.0f32; N];
        for b in buf.iter_mut() {
            // f0 completes exactly 64 cycles in N samples, so the signal is
            // periodic in the FFT frame → NO window needed and no leakage; the
            // non-harmonic bins are then pure aliasing.
            *b = if blep {
                osc.tick()
            } else {
                let v = 2.0 * phase - 1.0;
                phase += f0 / FS;
                if phase >= 1.0 {
                    phase -= 1.0;
                }
                v
            };
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
    fn polyblep_saw_suppresses_aliasing() {
        let naive = saw_alias_energy(false);
        let blep = saw_alias_energy(true);
        // polyBLEP should cut the inter-harmonic (alias) energy by well over 10 dB.
        assert!(
            blep < naive * 0.1,
            "alias energy: polyBLEP {blep:.3e} vs naive {naive:.3e}"
        );
    }

    #[test]
    fn sine_is_spectrally_pure() {
        const N: usize = 2048;
        let f0 = 750.0; // 32 bins
        let bin = (f0 * N as f32 / FS) as usize;
        let mut osc = Oscillator::new(FS, f0, Waveform::Sine);
        let buf: Vec<f32> = (0..N).map(|_| osc.tick()).collect();
        let arr: [f32; N] = buf.try_into().unwrap();
        let (mut sre, mut sim) = (std::vec![0.0f32; N / 2], std::vec![0.0f32; N / 2]);
        let (mut re, mut im) = (std::vec![0.0f32; N / 2 + 1], std::vec![0.0f32; N / 2 + 1]);
        RealFft::new(&mut sre, &mut sim).forward(&arr, &mut re, &mut im);
        let power = |k: usize| (re[k] * re[k] + im[k] * im[k]) as f64;
        let fund = power(bin);
        let rest: f64 = (1..N / 2).filter(|&k| k != bin).map(power).sum();
        // ≥ 99% of the energy in the fundamental bin.
        assert!(
            fund / (fund + rest) > 0.99,
            "sine THD too high: {}",
            rest / fund
        );
    }
}
