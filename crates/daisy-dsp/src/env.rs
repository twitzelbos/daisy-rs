//! A gated attack/release envelope — slow swells and drone holds.
//!
//! Deliberately trivial (compute is at a premium): one add + one clamp per
//! sample, no `exp`. Linear segments make it exactly predictable, so its tests
//! are closed-form rather than oracle-compared.
//!
//! `gate(true)` ramps the level up to `1.0` over `attack_s`; `gate(false)` ramps
//! it back down to `0.0` over `release_s`. Hold `gate(true)` for an infinite
//! "drone hold"; a long `attack_s` gives the signature ambient swell-in.

/// A linear AR (attack/release) envelope in `[0.0, 1.0]`.
#[derive(Debug, Clone, Copy)]
pub struct Env {
    attack_inc: f32,
    release_dec: f32,
    y: f32,
    gate: bool,
}

impl Env {
    /// Build an envelope. `attack_s`/`release_s` are the times to traverse the
    /// full `0↔1` range; both are clamped to at least one sample.
    #[must_use]
    pub fn new(sample_rate: f32, attack_s: f32, release_s: f32) -> Self {
        Self {
            attack_inc: 1.0 / (attack_s * sample_rate).max(1.0),
            release_dec: 1.0 / (release_s * sample_rate).max(1.0),
            y: 0.0,
            gate: false,
        }
    }

    /// Open (`true`) or close (`false`) the gate.
    pub fn gate(&mut self, on: bool) {
        self.gate = on;
    }

    /// Current level without advancing.
    #[must_use]
    pub fn value(&self) -> f32 {
        self.y
    }

    /// Advance one sample and return the new level.
    #[must_use]
    pub fn tick(&mut self) -> f32 {
        if self.gate {
            self.y = (self.y + self.attack_inc).min(1.0);
        } else {
            self.y = (self.y - self.release_dec).max(0.0);
        }
        self.y
    }

    /// Back to closed and silent.
    pub fn reset(&mut self) {
        self.y = 0.0;
        self.gate = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f32 = 48_000.0;

    #[test]
    fn attack_reaches_one_within_attack_time() {
        let mut e = Env::new(SR, 0.5, 1.0); // 0.5 s attack → 24000 samples
        e.gate(true);
        // Monotonic non-decreasing, bounded, and at 1.0 by the attack time.
        let mut prev = 0.0;
        for _ in 0..24_000 {
            let v = e.tick();
            assert!(v >= prev && v <= 1.0);
            prev = v;
        }
        assert!((e.value() - 1.0).abs() < 1e-4);
    }

    #[test]
    fn release_reaches_zero_within_release_time() {
        let mut e = Env::new(SR, 0.01, 0.25); // 0.25 s release → 12000 samples
        e.gate(true);
        for _ in 0..2_000 {
            let _ = e.tick();
        }
        assert!(e.value() > 0.99);
        e.gate(false);
        let mut prev = e.value();
        for _ in 0..12_000 {
            let v = e.tick();
            assert!(v <= prev && v >= 0.0);
            prev = v;
        }
        assert!(e.value() < 1e-4);
    }

    #[test]
    fn stays_clamped_when_held() {
        let mut e = Env::new(SR, 0.001, 0.001);
        e.gate(true);
        for _ in 0..1_000 {
            let _ = e.tick();
        }
        assert_eq!(e.value(), 1.0); // clamps, never overshoots
    }
}
