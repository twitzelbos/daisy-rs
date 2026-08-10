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

/// The four simultaneous outputs of an [`Svf`].
#[derive(Copy, Clone, Debug)]
pub struct SvfOut {
    pub lp: f32,
    pub bp: f32,
    pub hp: f32,
    pub notch: f32,
}

/// A 2-pole **state-variable filter** in the Zavalishin/Simper TPT (topology-
/// preserving, zero-delay-feedback) form. Unlike the classic Chamberlin SVF it
/// is stable and accurate right up to Nyquist, and one `tick` yields low-pass,
/// band-pass, high-pass and notch simultaneously — ideal for resonant sweeps,
/// wahs, and multimode tone controls.
///
/// Ref: A. Simper, "Solving the continuous SVF equations using trapezoidal
/// integration and equivalent currents" (Cytomic, 2020); V. Zavalishin, *The Art
/// of VA Filter Design*.
#[derive(Copy, Clone)]
pub struct Svf {
    k: f32, // damping = 1/Q
    a1: f32,
    a2: f32,
    a3: f32,
    ic1eq: f32, // integrator "equivalent current" states
    ic2eq: f32,
}

impl Svf {
    /// Build at `cutoff` Hz with quality factor `q` (0.5 = critically damped,
    /// 0.707 = Butterworth, higher = resonant).
    pub fn new(sample_rate: f32, cutoff: f32, q: f32) -> Self {
        // Prewarped integrator gain; clamp just below Nyquist so tan stays finite.
        let g = libm::tanf(PI * (cutoff / sample_rate).clamp(1.0e-5, 0.49));
        let k = 1.0 / q.max(1.0e-4);
        let a1 = 1.0 / (1.0 + g * (g + k));
        let a2 = g * a1;
        let a3 = g * a2;
        Self {
            k,
            a1,
            a2,
            a3,
            ic1eq: 0.0,
            ic2eq: 0.0,
        }
    }

    /// Process one sample, returning all four responses.
    #[inline]
    pub fn tick(&mut self, x: f32) -> SvfOut {
        let v3 = x - self.ic2eq;
        let v1 = self.a1 * self.ic1eq + self.a2 * v3;
        let v2 = self.ic2eq + self.a2 * self.ic1eq + self.a3 * v3;
        self.ic1eq = 2.0 * v1 - self.ic1eq;
        self.ic2eq = 2.0 * v2 - self.ic2eq;
        let lp = v2;
        let bp = v1;
        let hp = x - self.k * v1 - v2;
        SvfOut {
            lp,
            bp,
            hp,
            notch: lp + hp,
        }
    }

    /// Convenience single-output ticks.
    #[inline]
    pub fn lowpass(&mut self, x: f32) -> f32 {
        self.tick(x).lp
    }
    #[inline]
    pub fn highpass(&mut self, x: f32) -> f32 {
        self.tick(x).hp
    }
    #[inline]
    pub fn bandpass(&mut self, x: f32) -> f32 {
        self.tick(x).bp
    }

    /// Flush the integrator state.
    pub fn reset(&mut self) {
        self.ic1eq = 0.0;
        self.ic2eq = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f32 = 48_000.0;

    /// Steady-state magnitude response |H(f)|: drive a unit sine and measure the
    /// output peak over the tail (after the transient settles).
    fn magnitude(mut probe: impl FnMut(f32) -> f32, freq: f32) -> f32 {
        let n = 8192usize;
        let mut peak = 0.0f32;
        for i in 0..n {
            let x = libm::sinf(2.0 * PI * freq * i as f32 / FS);
            let y = probe(x);
            if i > n / 2 {
                peak = peak.max(y.abs());
            }
        }
        peak
    }

    #[test]
    fn svf_lowpass_tracks_the_analytic_2nd_order_response() {
        let fc = 1_000.0;
        for &q in &[0.5f32, 0.707, 2.0] {
            for &f in &[50.0f32, 250.0, fc, 4_000.0, 12_000.0] {
                let mut s = Svf::new(FS, fc, q);
                let meas = magnitude(|x| s.lowpass(x), f);
                // |H_lp| = 1 / sqrt((1-w²)² + (w/Q)²). The TPT/bilinear filter
                // realizes the analog prototype at the *prewarped* frequency, so
                // w = tan(πf/fs)/tan(πfc/fs), not the literal f/fc (they only
                // agree well below Nyquist).
                let w = libm::tanf(PI * f / FS) / libm::tanf(PI * fc / FS);
                let ideal = 1.0 / ((1.0 - w * w).powi(2) + (w / q).powi(2)).sqrt();
                let rel = (meas - ideal).abs() / ideal.max(1e-6);
                assert!(
                    rel < 0.06,
                    "q={q} f={f}: |H|={meas} ideal={ideal} rel={rel}"
                );
            }
        }
    }

    #[test]
    fn svf_lp_hp_dc_and_nyquist_limits() {
        let mut s = Svf::new(FS, 1_000.0, 0.707);
        // DC: run to steady state.
        let (mut lp, mut hp) = (0.0, 0.0);
        for _ in 0..8192 {
            let o = s.tick(1.0);
            lp = o.lp;
            hp = o.hp;
        }
        assert!((lp - 1.0).abs() < 1e-2, "LP DC gain {lp}");
        assert!(hp.abs() < 1e-2, "HP DC gain {hp}");
        // Near Nyquist the LP should be strongly attenuated, HP near unity.
        let mut s2 = Svf::new(FS, 1_000.0, 0.707);
        let lp_hi = magnitude(|x| s2.lowpass(x), 20_000.0);
        let mut s3 = Svf::new(FS, 1_000.0, 0.707);
        let hp_hi = magnitude(|x| s3.highpass(x), 20_000.0);
        assert!(lp_hi < 0.02, "LP@20k {lp_hi}");
        assert!(hp_hi > 0.9, "HP@20k {hp_hi}");
    }

    #[test]
    fn svf_notch_equals_lp_plus_hp_and_is_stable_at_high_q() {
        let mut s = Svf::new(FS, 3_000.0, 12.0);
        let mut seed = 0x1234u32;
        let mut maxv = 0.0f32;
        for _ in 0..20_000 {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let x = (seed >> 8) as f32 / (1 << 24) as f32 * 2.0 - 1.0;
            let o = s.tick(x);
            assert!((o.notch - (o.lp + o.hp)).abs() < 1e-4, "notch != lp+hp");
            assert!(o.lp.is_finite() && o.bp.is_finite() && o.hp.is_finite());
            maxv = maxv.max(o.bp.abs());
        }
        // High-Q resonance amplifies but must not blow up on bounded noise.
        assert!(maxv < 100.0, "SVF unstable at Q=12: peak {maxv}");
    }
}
