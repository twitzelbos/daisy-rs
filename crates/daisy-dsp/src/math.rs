//! Small scalar helpers. `no_std`; transcendentals via `libm`.

use core::f32::consts::PI;

/// Decibels → linear amplitude ratio: `10^(db/20)`.
#[inline]
pub fn db_to_lin(db: f32) -> f32 {
    libm::powf(10.0, db * (1.0 / 20.0))
}

/// Linear amplitude ratio → decibels: `20·log10(x)`.
#[inline]
pub fn lin_to_db(x: f32) -> f32 {
    20.0 * libm::log10f(x)
}

/// MIDI note number → frequency (Hz), A4 (note 69) = 440 Hz.
#[inline]
pub fn midi_to_freq(note: f32) -> f32 {
    440.0 * libm::exp2f((note - 69.0) * (1.0 / 12.0))
}

/// Normalised angular frequency `ω0 = 2π·f/fs` (radians/sample). Used by the
/// filter coefficient formulas; clamped just below π so `tan(ω0/2)` stays finite.
#[inline]
pub fn omega(sample_rate: f32, freq: f32) -> f32 {
    let w = 2.0 * PI * freq / sample_rate;
    // Keep strictly below π (Nyquist) so cos/sin-based designs stay well-formed.
    w.clamp(0.0, PI * 0.999)
}

/// Clamp to `[-1, 1]`.
#[inline]
pub fn clip(x: f32) -> f32 {
    x.clamp(-1.0, 1.0)
}

/// Linear interpolate: `a` at `t=0`, `b` at `t=1`.
#[inline]
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn db_round_trips() {
        assert!((db_to_lin(0.0) - 1.0).abs() < 1e-6);
        assert!((db_to_lin(6.0206) - 2.0).abs() < 1e-3); // +6.02 dB ≈ ×2
        assert!((lin_to_db(2.0) - 6.0206).abs() < 1e-3);
    }

    #[test]
    fn a440() {
        assert!((midi_to_freq(69.0) - 440.0).abs() < 1e-3);
        assert!((midi_to_freq(81.0) - 880.0).abs() < 1e-2); // one octave up
    }
}
