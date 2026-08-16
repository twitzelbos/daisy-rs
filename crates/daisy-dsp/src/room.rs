//! Room acoustics helpers for spatialization — the *externalization* half of a
//! binaural render (the HRIR gives *direction*; the room makes it sound "out
//! there" rather than at the eardrums).
//!
//! [`EarlyReflections`] models the first discrete wall/floor/ceiling bounces —
//! the strongest externalization cue, stronger than late reverb — as a sparse
//! multitap delay: mono in, decorrelated stereo out. Late reverb is
//! [`crate::reverb::FdnReverb`]; air-absorption is a [`crate::filter::OnePole`]
//! low-pass; distance is an amplitude/dry-wet balance the caller applies. See
//! `docs/binaural-spatializer.md` (phase 2).

use crate::delay::DelayLine;

/// One early-reflection tap: a fractional `delay` (samples) with independent
/// left/right gains, so taps can be panned across the ears for decorrelation.
#[derive(Copy, Clone, Debug)]
pub struct Tap {
    /// Delay from the direct sound, in samples (fractional; interpolated).
    pub delay: f32,
    /// Left-ear gain for this reflection.
    pub gain_l: f32,
    /// Right-ear gain for this reflection.
    pub gain_r: f32,
}

/// A sparse multitap delay producing early reflections: mono in → decorrelated
/// stereo out. Backing storage and the tap list are caller-owned (`no_std`, no
/// alloc); the longest tap `delay` must be `< buf.len()`.
#[derive(Debug)]
pub struct EarlyReflections<'a> {
    line: DelayLine<'a>,
    taps: &'a [Tap],
}

impl<'a> EarlyReflections<'a> {
    /// Wrap `buf` (delay storage) and `taps`. The buffer must be longer than the
    /// largest tap delay; a tap beyond the buffer is clamped by the delay line.
    #[must_use]
    pub fn new(buf: &'a mut [f32], taps: &'a [Tap]) -> Self {
        Self {
            line: DelayLine::new(buf),
            taps,
        }
    }

    /// Push one mono sample; return the summed `(left, right)` reflections.
    #[inline]
    pub fn tick(&mut self, x: f32) -> (f32, f32) {
        self.line.write(x);
        let mut l = 0.0f32;
        let mut r = 0.0f32;
        for t in self.taps {
            let s = self.line.read_frac(t.delay);
            l += s * t.gain_l;
            r += s * t.gain_r;
        }
        (l, r)
    }

    /// Process a mono block into a stereo pair (all three slices share length).
    pub fn process(&mut self, input: &[f32], out_l: &mut [f32], out_r: &mut [f32]) {
        for ((&x, ol), or) in input.iter().zip(out_l.iter_mut()).zip(out_r.iter_mut()) {
            let (l, r) = self.tick(x);
            *ol = l;
            *or = r;
        }
    }

    /// Clear the delay memory.
    pub fn reset(&mut self) {
        self.line.reset();
    }
}

/// Fill `taps` with a plausible small-room early-reflection pattern: delays
/// spread from `first_ms` to `spread_ms`, gains falling ~6 dB per doubling of
/// delay, and left/right emphasis alternated per tap for decorrelation (a
/// wider, more externalized image). `first_ms` should exceed the direct path's
/// own latency so reflections never precede the direct sound.
pub fn room_taps(taps: &mut [Tap], sample_rate: f32, first_ms: f32, spread_ms: f32) {
    let n = taps.len();
    for (i, t) in taps.iter_mut().enumerate() {
        // Delays spaced across [first_ms, spread_ms] with mild irregularity so
        // the reflections don't comb-filter into an audible pitch.
        let frac = if n > 1 {
            i as f32 / (n - 1) as f32
        } else {
            0.0
        };
        let jitter = 0.13 * libm::sinf(i as f32 * 2.399); // ± up to 13% of a step
        let ms = first_ms + (spread_ms - first_ms) * (frac + jitter).clamp(0.0, 1.0);
        let delay = ms * 1e-3 * sample_rate;
        // −6 dB per delay doubling relative to the first tap (gain ∝ 1/delay).
        let g = first_ms / ms;
        // Send each reflection predominantly to one ear, alternating — so the
        // two ears' taps sit at DIFFERENT delays (as real path-length
        // differences do), which is what decorrelates L/R and widens the image.
        // A small cross-feed keeps it from hard-panning.
        const CROSS: f32 = 0.25;
        let (gl, gr) = if i % 2 == 0 {
            (g, CROSS * g)
        } else {
            (CROSS * g, g)
        };
        t.delay = delay;
        t.gain_l = gl;
        t.gain_r = gr;
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;

    #[test]
    fn impulse_places_echoes_at_tap_delays() {
        let taps = [
            Tap {
                delay: 10.0,
                gain_l: 0.5,
                gain_r: 0.2,
            },
            Tap {
                delay: 25.0,
                gain_l: 0.1,
                gain_r: 0.4,
            },
        ];
        let mut buf = vec![0.0f32; 64];
        let mut er = EarlyReflections::new(&mut buf, &taps);

        let n = 40;
        let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
        let mut input = vec![0.0f32; n];
        input[0] = 1.0; // unit impulse
        er.process(&input, &mut l, &mut r);

        // Each tap appears at its delay with its gain; elsewhere silence.
        assert!((l[10] - 0.5).abs() < 1e-6, "L tap0 @10");
        assert!((r[10] - 0.2).abs() < 1e-6, "R tap0 @10");
        assert!((l[25] - 0.1).abs() < 1e-6, "L tap1 @25");
        assert!((r[25] - 0.4).abs() < 1e-6, "R tap1 @25");
        assert!(
            l[15].abs() < 1e-6 && r[15].abs() < 1e-6,
            "silent between taps"
        );
        assert!(l[0].abs() < 1e-6, "no zero-delay tap");
    }

    #[test]
    fn output_is_decorrelated_stereo() {
        // The generated room pattern must produce L ≠ R (decorrelation is what
        // widens/externalizes the image).
        let mut taps = [Tap {
            delay: 0.0,
            gain_l: 0.0,
            gain_r: 0.0,
        }; 8];
        room_taps(&mut taps, 48_000.0, 8.0, 40.0);
        // First tap after the direct path, last within the buffer, gains falling.
        assert!(taps[0].delay >= 8.0 * 48.0 - 1.0);
        assert!(taps[7].delay <= 40.0 * 48.0 + 1.0);

        let mut buf = vec![0.0f32; 4096];
        let mut er = EarlyReflections::new(&mut buf, &taps);
        let n = 2048;
        let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
        let mut input = vec![0.0f32; n];
        input[0] = 1.0;
        er.process(&input, &mut l, &mut r);

        let dot: f32 = l.iter().zip(&r).map(|(a, b)| a * b).sum();
        let el: f32 = l.iter().map(|a| a * a).sum::<f32>().sqrt();
        let er_: f32 = r.iter().map(|a| a * a).sum::<f32>().sqrt();
        let corr = dot / (el * er_ + 1e-20);
        assert!(el > 0.0 && er_ > 0.0, "both ears carry energy");
        assert!(corr < 0.85, "L/R should be decorrelated, got corr {corr}");
        assert!(l.iter().chain(&r).all(|v| v.is_finite()));
    }

    #[test]
    fn tap_beyond_buffer_is_clamped_not_panicking() {
        // A delay longer than the buffer must clamp (DelayLine caps at len−1),
        // never index out of bounds.
        let taps = [Tap {
            delay: 500.0,
            gain_l: 1.0,
            gain_r: 1.0,
        }];
        let mut buf = vec![0.0f32; 16];
        let mut er = EarlyReflections::new(&mut buf, &taps);
        let (mut l, mut r) = ([0.0f32; 8], [0.0f32; 8]);
        er.process(&[1.0; 8], &mut l, &mut r);
        assert!(l.iter().chain(&r).all(|v| v.is_finite()));
    }
}
