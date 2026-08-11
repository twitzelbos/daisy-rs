//! Deterministic pseudo-random source.
//!
//! A 32-bit **xorshift** generator (Marsaglia). Deterministic and portable —
//! the exact same `u32` sequence on any platform — which is what lets the
//! granular/spray effects be *reproducible*: same seed ⇒ same texture, and the
//! golden tests can pin the output bit-for-bit (the Python oracle replicates the
//! identical integer sequence).

/// xorshift32 PRNG.
#[derive(Debug, Copy, Clone)]
pub struct Prng {
    state: u32,
}

impl Prng {
    /// Seed the generator. A zero seed (which xorshift cannot escape) is
    /// replaced with a fixed non-zero constant.
    #[must_use]
    pub fn new(seed: u32) -> Self {
        Self {
            state: if seed == 0 { 0x9E37_79B9 } else { seed },
        }
    }

    /// Next raw 32-bit word. `x ^= x<<13; x ^= x>>17; x ^= x<<5`.
    #[inline]
    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.state = x;
        x
    }

    /// Next sample in `[-1.0, 1.0)`. Uses the top 24 bits, so the mapping is
    /// exact in `f32` (24-bit mantissa) — no rounding to disagree over.
    #[inline]
    pub fn next_f32(&mut self) -> f32 {
        let bits24 = self.next_u32() >> 8; // 0 .. 2^24 − 1
        (bits24 as f32) * (2.0 / 16_777_216.0) - 1.0
    }

    /// Fill a block with white noise in `[-1.0, 1.0)`.
    pub fn fill(&mut self, out: &mut [f32]) {
        for o in out.iter_mut() {
            *o = self.next_f32();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic() {
        let mut a = Prng::new(1);
        let mut b = Prng::new(1);
        for _ in 0..100 {
            assert_eq!(a.next_u32(), b.next_u32());
        }
    }

    #[test]
    fn in_range() {
        let mut p = Prng::new(42);
        for _ in 0..10_000 {
            let x = p.next_f32();
            assert!((-1.0..1.0).contains(&x));
        }
    }
}
