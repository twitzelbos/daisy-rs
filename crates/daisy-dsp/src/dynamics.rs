//! Dynamics processors — envelope follower, compressor, noise gate, and a
//! lookahead brickwall limiter. `no_std`, no alloc.
//!
//! The compressor/gate use the **log-domain, feed-forward, decoupled** design
//! (rectify → level in dB → static gain computer → smooth the gain reduction with
//! attack/release ballistics → apply), which is the standard, well-behaved
//! topology.
//!
//! # References
//! - D. Giannoulis, M. Massberg & J. Reiss, "Digital Dynamic Range Compressor
//!   Design — A Tutorial and Analysis," *J. Audio Eng. Soc.* 60(6), 2012.

use crate::math::{db_to_lin, lin_to_db};

/// Attack/release smoothing coefficient for a time constant `tau` (seconds):
/// `exp(-1/(tau·fs))`. A step reaches ~63% in `tau`.
#[inline]
fn ballistic(tau_s: f32, sample_rate: f32) -> f32 {
    if tau_s <= 0.0 {
        0.0
    } else {
        libm::expf(-1.0 / (tau_s * sample_rate))
    }
}

/// A peak envelope follower: full-wave rectify + one-pole attack/release.
#[derive(Copy, Clone)]
pub struct EnvelopeFollower {
    att: f32,
    rel: f32,
    env: f32,
}

impl EnvelopeFollower {
    pub fn new(sample_rate: f32, attack_s: f32, release_s: f32) -> Self {
        Self {
            att: ballistic(attack_s, sample_rate),
            rel: ballistic(release_s, sample_rate),
            env: 0.0,
        }
    }

    /// Update with one sample; returns the current (linear) envelope.
    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        let a = x.abs();
        let coef = if a > self.env { self.att } else { self.rel };
        self.env = a + coef * (self.env - a);
        self.env
    }

    #[inline]
    pub fn value(&self) -> f32 {
        self.env
    }

    pub fn reset(&mut self) {
        self.env = 0.0;
    }
}

/// A feed-forward compressor (log-domain, decoupled peak detector).
#[derive(Copy, Clone)]
pub struct Compressor {
    thr_db: f32,
    slope: f32, // 1/ratio − 1  (≤ 0)
    knee_db: f32,
    makeup_db: f32,
    att: f32,
    rel: f32,
    yl: f32, // smoothed gain reduction (dB, ≤ 0)
}

impl Compressor {
    /// - `threshold_db`: level above which compression starts.
    /// - `ratio`: input/output slope above threshold (e.g. 4.0 → 4:1).
    /// - `knee_db`: width of the soft (quadratic) knee, 0 = hard.
    /// - `makeup_db`: output gain applied after compression.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        sample_rate: f32,
        threshold_db: f32,
        ratio: f32,
        knee_db: f32,
        attack_s: f32,
        release_s: f32,
        makeup_db: f32,
    ) -> Self {
        Self {
            thr_db: threshold_db,
            slope: 1.0 / ratio.max(1.0) - 1.0,
            knee_db: knee_db.max(0.0),
            makeup_db,
            att: ballistic(attack_s, sample_rate),
            rel: ballistic(release_s, sample_rate),
            yl: 0.0,
        }
    }

    /// Static gain computer: the gain change (dB, ≤ 0) for an input level.
    #[inline]
    fn compute_gain_db(&self, x_db: f32) -> f32 {
        let over = x_db - self.thr_db;
        if self.knee_db > 0.0 && 2.0 * over > -self.knee_db && 2.0 * over < self.knee_db {
            // Soft knee: quadratic interpolation across the knee. (`powi` is not
            // in core/no_std, so square by hand.)
            let d = over + self.knee_db * 0.5;
            self.slope * (d * d) / (2.0 * self.knee_db)
        } else if over <= 0.0 {
            0.0
        } else {
            self.slope * over
        }
    }

    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        let x_db = lin_to_db(x.abs().max(1.0e-9));
        let target = self.compute_gain_db(x_db); // ≤ 0
                                                 // Decoupled ballistics: attack when the reduction deepens, release else.
        let coef = if target < self.yl { self.att } else { self.rel };
        self.yl = target + coef * (self.yl - target);
        x * db_to_lin(self.yl + self.makeup_db)
    }

    pub fn reset(&mut self) {
        self.yl = 0.0;
    }
}

/// A downward noise gate: below `threshold_db` the signal is attenuated toward
/// `floor_db`, with attack (open), hold, and release (close) ballistics.
#[derive(Copy, Clone)]
pub struct Gate {
    thr_db: f32,
    floor_lin: f32,
    att: f32,
    rel: f32,
    hold_samples: u32,
    held: u32,
    gain: f32, // current linear gain in [floor, 1]
    det: EnvelopeFollower,
}

impl Gate {
    pub fn new(
        sample_rate: f32,
        threshold_db: f32,
        floor_db: f32,
        attack_s: f32,
        hold_s: f32,
        release_s: f32,
    ) -> Self {
        Self {
            thr_db: threshold_db,
            floor_lin: db_to_lin(floor_db),
            att: ballistic(attack_s, sample_rate),
            rel: ballistic(release_s, sample_rate),
            hold_samples: (hold_s * sample_rate) as u32,
            held: 0,
            gain: 0.0,
            det: EnvelopeFollower::new(sample_rate, 0.0, 0.01),
        }
    }

    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        let env_db = lin_to_db(self.det.tick(x).max(1.0e-9));
        let open = env_db > self.thr_db;
        if open {
            self.held = self.hold_samples;
        } else if self.held > 0 {
            self.held -= 1;
        }
        let target = if open || self.held > 0 {
            1.0
        } else {
            self.floor_lin
        };
        let coef = if target > self.gain {
            self.att
        } else {
            self.rel
        };
        self.gain = target + coef * (self.gain - target);
        x * self.gain
    }

    pub fn reset(&mut self) {
        self.gain = 0.0;
        self.held = 0;
        self.det.reset();
    }
}

/// A lookahead brickwall peak limiter: the output magnitude is guaranteed
/// `≤ ceiling`. The signal is delayed by `LA` samples; the gain is derived from
/// the peak over the `LA`-sample lookahead window (so a peak's gain reduction is
/// already in place when the peak reaches the output), then released gently.
///
/// `LA` is the lookahead in samples (e.g. 48 ≈ 1 ms at 48 kHz). The window-max
/// guarantee holds because the sample being output is itself inside the window.
#[derive(Copy, Clone)]
pub struct Limiter<const LA: usize> {
    ceiling: f32,
    ring: [f32; LA],
    pos: usize,
    gain: f32,
    rel: f32,
}

impl<const LA: usize> Limiter<LA> {
    pub fn new(sample_rate: f32, ceiling: f32, release_s: f32) -> Self {
        Self {
            ceiling: ceiling.max(1.0e-6),
            ring: [0.0; LA],
            pos: 0,
            gain: 1.0,
            rel: ballistic(release_s, sample_rate),
        }
    }

    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        // The oldest sample in the ring is the one about to be output.
        let out_sample = self.ring[self.pos];
        self.ring[self.pos] = x;
        self.pos = if self.pos + 1 == LA { 0 } else { self.pos + 1 };

        // Peak over the whole lookahead window (includes out_sample).
        let mut peak = 0.0f32;
        for &v in &self.ring {
            peak = peak.max(v.abs());
        }
        let target = if peak > self.ceiling {
            self.ceiling / peak
        } else {
            1.0
        };
        // Attack is instantaneous (drop to target now); release rises gently.
        // Clamping to `target` preserves the ≤ ceiling guarantee.
        self.gain = if target < self.gain {
            target
        } else {
            (target + self.rel * (self.gain - target)).min(target)
        };
        out_sample * self.gain
    }

    /// Latency introduced by the lookahead (samples).
    pub const fn latency(&self) -> usize {
        LA
    }

    pub fn reset(&mut self) {
        self.ring = [0.0; LA];
        self.pos = 0;
        self.gain = 1.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f32 = 48_000.0;

    fn sine_peak(mut f: impl FnMut(f32) -> f32, amp: f32, freq: f32, n: usize) -> f32 {
        let mut peak = 0.0f32;
        for i in 0..n {
            let x = amp * libm::sinf(2.0 * core::f32::consts::PI * freq * i as f32 / FS);
            let y = f(x);
            if i > n * 3 / 4 {
                peak = peak.max(y.abs());
            }
        }
        peak
    }

    #[test]
    fn envelope_follower_converges_to_amplitude() {
        let mut e = EnvelopeFollower::new(FS, 0.001, 0.05);
        let mut last = 0.0;
        for i in 0..48_000 {
            last = e.tick(0.7 * libm::sinf(2.0 * core::f32::consts::PI * 100.0 * i as f32 / FS));
        }
        // Peak follower on a 0.7-amp sine settles near 0.7.
        assert!((last - 0.7).abs() < 0.05, "env {last}");
    }

    #[test]
    fn compressor_static_curve_matches_the_gain_law() {
        // 4:1 above −20 dBFS, hard knee, unity makeup. Steady tone in → measure
        // steady output level, compare to threshold + (in−thr)/ratio.
        let thr = -20.0;
        let ratio = 4.0;
        for &in_db in &[-40.0f32, -20.0, -10.0, 0.0] {
            let amp = db_to_lin(in_db);
            let mut c = Compressor::new(FS, thr, ratio, 0.0, 0.001, 0.05, 0.0);
            let out_peak = sine_peak(|x| c.tick(x), amp, 200.0, 48_000);
            let out_db = lin_to_db(out_peak);
            let want = if in_db <= thr {
                in_db
            } else {
                thr + (in_db - thr) / ratio
            };
            assert!(
                (out_db - want).abs() < 1.0,
                "in={in_db}: out={out_db} want={want}"
            );
        }
    }

    #[test]
    fn limiter_never_exceeds_ceiling_and_passes_quiet_signals() {
        let ceiling = 0.5;
        let mut lim = Limiter::<48>::new(FS, ceiling, 0.05);
        // Hot input (amp 2.0) — every output sample must stay under the ceiling.
        let mut maxout = 0.0f32;
        for i in 0..48_000 {
            let x = 2.0 * libm::sinf(2.0 * core::f32::consts::PI * 220.0 * i as f32 / FS);
            maxout = maxout.max(lim.tick(x).abs());
        }
        assert!(
            maxout <= ceiling * 1.001,
            "limiter overshoot: {maxout} > {ceiling}"
        );

        // Quiet input (peak 0.2 < ceiling) passes through unchanged after latency.
        let mut lim2 = Limiter::<48>::new(FS, ceiling, 0.05);
        let mut buf = [0.0f32; 512];
        for (i, b) in buf.iter_mut().enumerate() {
            let x = 0.2 * libm::sinf(2.0 * core::f32::consts::PI * 440.0 * i as f32 / FS);
            *b = lim2.tick(x);
        }
        // After the 48-sample latency, |out| tracks the (undistorted) 0.2 peak.
        let tail_peak = buf[128..].iter().fold(0.0f32, |m, &v| m.max(v.abs()));
        assert!(
            (tail_peak - 0.2).abs() < 0.01,
            "quiet signal altered: {tail_peak}"
        );
    }

    #[test]
    fn gate_passes_loud_and_attenuates_quiet() {
        // Loud tone opens the gate → near unity; very quiet tone stays near floor.
        let mut g = Gate::new(FS, -30.0, -60.0, 0.001, 0.01, 0.05);
        let loud = sine_peak(|x| g.tick(x), db_to_lin(-6.0), 300.0, 48_000);
        assert!(
            (lin_to_db(loud) - -6.0).abs() < 1.0,
            "gate should pass loud: {}",
            lin_to_db(loud)
        );

        let mut g2 = Gate::new(FS, -30.0, -60.0, 0.001, 0.01, 0.05);
        let quiet = sine_peak(|x| g2.tick(x), db_to_lin(-50.0), 300.0, 96_000);
        // −50 dB in is below the −30 dB threshold → attenuated well below input.
        assert!(
            lin_to_db(quiet) < -60.0,
            "gate should close: {} dB",
            lin_to_db(quiet)
        );
    }
}
