//! A Hann window as a small lookup table — grain envelopes (and, later, FFT).
//!
//! Economical: the raised cosine is evaluated **once** at construction into an
//! `N`-point table; the hot path ([`at`](HannWindow::at)) is a table lookup plus
//! one linear interpolation — no `cos` per sample. A Hann window at ≥50 % grain
//! overlap sums to a constant (the COLA property), so a granular cloud stays
//! click-free without amplitude ripple.

/// An `N`-point Hann window, sampled by a normalised phase in `[0.0, 1.0)`.
#[derive(Debug)]
pub struct HannWindow<const N: usize> {
    tab: [f32; N],
}

impl<const N: usize> HannWindow<N> {
    /// Build the table. Symmetric Hann: `w[i] = 0.5 − 0.5·cos(2π·i/(N−1))`, so
    /// the endpoints are exactly 0 and the centre is 1.
    #[must_use]
    pub fn new() -> Self {
        let mut tab = [0.0f32; N];
        let denom = (N - 1) as f32;
        for (i, w) in tab.iter_mut().enumerate() {
            *w = 0.5 - 0.5 * libm::cosf(core::f32::consts::TAU * i as f32 / denom);
        }
        Self { tab }
    }

    /// Window value at `phase` ∈ `[0.0, 1.0)` (linear-interpolated). Values at or
    /// past `1.0` clamp to the final (zero) sample.
    #[must_use]
    pub fn at(&self, phase: f32) -> f32 {
        let x = phase * (N - 1) as f32;
        let i = x as usize;
        if i >= N - 1 {
            return self.tab[N - 1];
        }
        let frac = x - i as f32;
        self.tab[i] + (self.tab[i + 1] - self.tab[i]) * frac
    }
}

impl<const N: usize> Default for HannWindow<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoints_zero_center_one() {
        let w = HannWindow::<1024>::new();
        assert!(w.at(0.0).abs() < 1e-6);
        assert!((w.at(0.5) - 1.0).abs() < 1e-3);
        assert!(w.at(0.999).abs() < 1e-2);
    }

    #[test]
    fn symmetric_and_bounded() {
        let w = HannWindow::<512>::new();
        for k in 0..=50 {
            let p = k as f32 / 100.0; // 0.0 .. 0.5
            let v = w.at(p);
            let vm = w.at(1.0 - p - 1.0 / 511.0); // mirror
            assert!((0.0..=1.0001).contains(&v));
            assert!((v - vm).abs() < 2e-2, "asymmetry at {p}: {v} vs {vm}");
        }
    }

    #[test]
    fn rises_then_falls() {
        let w = HannWindow::<256>::new();
        assert!(w.at(0.25) > w.at(0.05));
        assert!(w.at(0.5) > w.at(0.25));
        assert!(w.at(0.75) < w.at(0.5));
    }
}
