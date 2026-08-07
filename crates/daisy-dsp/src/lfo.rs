//! A low-frequency oscillator for slow modulation — grain-position drift, pan,
//! filter movement, detune. **Designed for control-rate ticking** (once per
//! block), where the per-tick `sin` is negligible; the `Triangle` shape needs no
//! `libm` at all.
//!
//! Output is bipolar `[-1.0, 1.0]`; [`unipolar`](Lfo::unipolar) maps it to
//! `[0.0, 1.0]` for e.g. a filter cutoff sweep.

/// LFO waveform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Waveform {
    Sine,
    Triangle,
}

/// A phase-accumulator LFO.
pub struct Lfo {
    phase: f32, // [0.0, 1.0)
    inc: f32,
    wave: Waveform,
}

impl Lfo {
    /// Build an LFO at `freq_hz`, ticked at `sample_rate` (use the *block* rate
    /// if you tick once per block).
    pub fn new(sample_rate: f32, freq_hz: f32, wave: Waveform) -> Self {
        Self {
            phase: 0.0,
            inc: freq_hz / sample_rate,
            wave,
        }
    }

    /// Change the rate (Hz).
    pub fn set_freq(&mut self, sample_rate: f32, freq_hz: f32) {
        self.inc = freq_hz / sample_rate;
    }

    /// Current bipolar value without advancing.
    pub fn value(&self) -> f32 {
        match self.wave {
            Waveform::Sine => libm::sinf(core::f32::consts::TAU * self.phase),
            // Branchless triangle: −1 at the wrap, +1 at mid-cycle
            // (−1 → 0 → +1 → 0), peak-to-peak 2, centred on 0.
            Waveform::Triangle => 1.0 - 4.0 * (self.phase - 0.5).abs(),
        }
    }

    /// Advance one tick and return the new bipolar value in `[-1.0, 1.0]`.
    pub fn tick(&mut self) -> f32 {
        let v = self.value();
        self.phase += self.inc;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }
        v
    }

    /// Advance one tick and return a unipolar value in `[0.0, 1.0]`.
    pub fn tick_unipolar(&mut self) -> f32 {
        0.5 * (self.tick() + 1.0)
    }

    /// Map a bipolar value to `[0.0, 1.0]`.
    pub fn unipolar(v: f32) -> f32 {
        0.5 * (v + 1.0)
    }

    /// Reset the phase to 0.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f32 = 48_000.0;

    #[test]
    fn bounded_and_periodic() {
        for wave in [Waveform::Sine, Waveform::Triangle] {
            let mut lfo = Lfo::new(SR, 100.0, wave); // period = 480 samples
            let first: [f32; 480] = core::array::from_fn(|_| lfo.tick());
            for (n, &v) in first.iter().enumerate() {
                assert!(
                    (-1.0001..=1.0001).contains(&v),
                    "{wave:?} out of range at {n}: {v}"
                );
            }
            // After one full period, the value repeats.
            let next = lfo.tick();
            assert!((next - first[0]).abs() < 1e-3, "{wave:?} not periodic");
        }
    }

    #[test]
    fn sine_hits_the_corners() {
        // inc = 0.25 → quarter-cycle steps land on the sine's corners (small,
        // few-step args avoid accumulation / large-argument sinf drift).
        let mut lfo = Lfo::new(SR, SR / 4.0, Waveform::Sine);
        assert!(lfo.tick().abs() < 1e-6); // phase 0.00 → 0
        assert!((lfo.tick() - 1.0).abs() < 1e-6); // phase 0.25 → +1
        assert!(lfo.tick().abs() < 1e-5); // phase 0.50 → 0 (sinf(π) ≈ −8.7e-8)
        assert!((lfo.tick() + 1.0).abs() < 1e-6); // phase 0.75 → −1
    }

    #[test]
    fn triangle_hits_the_corners() {
        let mut lfo = Lfo::new(SR, SR / 4.0, Waveform::Triangle); // inc = 0.25
        assert!((lfo.tick() + 1.0).abs() < 1e-6); // phase 0.00 → −1
        assert!(lfo.tick().abs() < 1e-6); // phase 0.25 → 0
        assert!((lfo.tick() - 1.0).abs() < 1e-6); // phase 0.50 → +1
        assert!(lfo.tick().abs() < 1e-6); // phase 0.75 → 0
    }
}
