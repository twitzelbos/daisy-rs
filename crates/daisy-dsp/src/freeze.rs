//! Capture-and-sustain ("freeze") — the pad's infinite-sustain core.
//!
//! While unfrozen it continuously records the input into a borrowed ring (place
//! it in SDRAM — seconds of audio). On [`freeze`](Self::freeze) it snapshots the
//! most recent `cap_len` samples and loops them **forever**, seamlessly: the loop
//! seam is smoothed **once** at freeze time (an O(cap_len) rotate + a short
//! head/tail crossfade), so the per-sample steady state is just one buffer read
//! and a compare — no `%`, no windowing, no click.
//!
//! The seam works because the recorded samples are contiguous: after rotating
//! the oldest sample to index 0, sample `P` continues naturally from `P−1`, so
//! blending the tail `[P, cap_len)` into the head `[0, xfade)` makes the wrap
//! `P−1 → 0` continuous while the crossfade body stays smooth.

/// A looping capture ("freeze") over a caller-provided buffer.
#[derive(Debug)]
pub struct Freeze<'a> {
    buf: &'a mut [f32],
    cap_len: usize,
    xfade: usize,
    /// Effective loop period once frozen: `cap_len - xfade`.
    period: usize,
    write_pos: usize,
    play_pos: usize,
    frozen: bool,
}

impl<'a> Freeze<'a> {
    /// Build over `buf`; the freeze loop is the whole buffer (`buf.len()`
    /// samples). `xfade_samples` is the seam crossfade length (clamped to at most
    /// a quarter of the buffer). A longer buffer = a longer, more evolving loop.
    pub fn new(buf: &'a mut [f32], xfade_samples: usize) -> Self {
        let cap_len = buf.len();
        assert!(cap_len >= 8, "Freeze buffer too small");
        let xfade = xfade_samples.clamp(1, cap_len / 4);
        buf.fill(0.0);
        Self {
            buf,
            cap_len,
            xfade,
            period: cap_len - xfade,
            write_pos: 0,
            play_pos: 0,
            frozen: false,
        }
    }

    /// Whether the loop is currently held.
    pub fn is_frozen(&self) -> bool {
        self.frozen
    }

    /// Snapshot the most recent `cap_len` samples and start looping them. The
    /// one-time seam smoothing lives here; the hot path stays a single read.
    pub fn freeze(&mut self) {
        if self.frozen {
            return;
        }
        // Rotate so the oldest recorded sample sits at index 0 → contiguous loop.
        self.buf[..self.cap_len].rotate_left(self.write_pos);
        // Crossfade the tail [period, cap_len) into the head [0, xfade) so the
        // wrap is seamless. f rises 0→1 across the fade: out[0]=tail (continues
        // from period-1), out[xfade]=original head.
        let (period, xfade) = (self.period, self.xfade);
        for i in 0..xfade {
            let f = i as f32 / xfade as f32;
            self.buf[i] = f * self.buf[i] + (1.0 - f) * self.buf[period + i];
        }
        self.play_pos = 0;
        self.frozen = true;
    }

    /// Release the hold and resume recording.
    pub fn unfreeze(&mut self) {
        self.frozen = false;
    }

    /// Process one sample: records + passes through when live; returns the next
    /// looped sample when frozen.
    pub fn tick(&mut self, x: f32) -> f32 {
        if self.frozen {
            let y = self.buf[self.play_pos];
            self.play_pos += 1;
            if self.play_pos == self.period {
                self.play_pos = 0;
            }
            y
        } else {
            self.buf[self.write_pos] = x;
            self.write_pos += 1;
            if self.write_pos == self.cap_len {
                self.write_pos = 0;
            }
            x
        }
    }

    /// Flush the capture and return to live/recording from the buffer start.
    pub fn reset(&mut self) {
        self.buf.fill(0.0);
        self.write_pos = 0;
        self.play_pos = 0;
        self.frozen = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn live_passes_through() {
        let mut buf = [0.0f32; 64];
        let mut fz = Freeze::new(&mut buf, 8);
        for i in 0..40 {
            let x = (i as f32) * 0.01;
            assert_eq!(fz.tick(x), x);
        }
    }

    #[test]
    fn frozen_output_is_periodic_and_finite() {
        let mut buf = [0.0f32; 256];
        let xfade = 32;
        let mut fz = Freeze::new(&mut buf, xfade);
        // Record a full buffer of a smooth tone.
        for i in 0..256 {
            let t = i as f32 / 256.0;
            fz.tick(libm::sinf(core::f32::consts::TAU * 3.0 * t));
        }
        fz.freeze();
        let period = 256 - xfade;
        // Collect two loop periods.
        let mut out = [0.0f32; 448]; // 2 * period
        for o in out.iter_mut() {
            *o = fz.tick(0.0);
        }
        assert!(out.iter().all(|v| v.is_finite()));
        // Exact periodicity: sample n == sample n+period.
        for n in 0..period {
            assert!((out[n] - out[n + period]).abs() < 1e-6);
        }
    }

    #[test]
    fn seam_has_no_click() {
        let mut buf = [0.0f32; 512];
        let xfade = 64;
        let mut fz = Freeze::new(&mut buf, xfade);
        // A tone whose period does NOT divide the loop, so a naive butt-join
        // would click at the seam; the crossfade must smooth it.
        for i in 0..512 {
            let t = i as f32;
            fz.tick(libm::sinf(0.11 * t));
        }
        fz.freeze();
        let period = 512 - xfade;
        let mut out = [0.0f32; 900];
        for o in out.iter_mut() {
            *o = fz.tick(0.0);
        }
        // Largest step within a body must bound the step across the wrap: no
        // sample-to-sample discontinuity bigger than the signal's own slope.
        let max_body_step = (1..period)
            .map(|n| (out[n] - out[n - 1]).abs())
            .fold(0.0f32, f32::max);
        let wrap_step = (out[period] - out[period - 1]).abs();
        assert!(
            wrap_step <= max_body_step * 1.5,
            "wrap step {wrap_step} vs body {max_body_step}"
        );
    }
}
