//! A delay line over caller-provided storage.
//!
//! The backing buffer is **borrowed** (`&mut [f32]`), so the same type works
//! with a stack array (short delays) or an SDRAM slice (seconds of delay) — the
//! app decides where the memory lives. Length = the buffer length; the maximum
//! delay is `len − 1` samples.

/// A ring-buffer delay line. Write the newest sample with [`write`](Self::write)
/// (or [`tick`](Self::tick)); read a delayed sample with [`read`](Self::read) /
/// [`read_frac`](Self::read_frac). `read(0)` is the sample just written.
pub struct DelayLine<'a> {
    buf: &'a mut [f32],
    write: usize,
}

impl<'a> DelayLine<'a> {
    /// Wrap `buf` (cleared to silence). Max delay = `buf.len() − 1`.
    pub fn new(buf: &'a mut [f32]) -> Self {
        buf.fill(0.0);
        Self { buf, write: 0 }
    }

    /// Length of the backing buffer (one more than the max delay).
    #[inline]
    pub fn len(&self) -> usize {
        self.buf.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    /// Push a new sample; it becomes `read(0)`.
    #[inline]
    pub fn write(&mut self, x: f32) {
        self.buf[self.write] = x;
        self.write = if self.write + 1 == self.buf.len() {
            0
        } else {
            self.write + 1
        };
    }

    /// Read the sample written `delay` samples ago. `delay = 0` is the most
    /// recent write; `delay` is clamped to `len − 1`.
    #[inline]
    pub fn read(&self, delay: usize) -> f32 {
        let n = self.buf.len();
        let d = delay.min(n - 1);
        // Most recent write is at (write − 1); go back another `d`.
        let idx = (self.write + n - 1 - d) % n;
        self.buf[idx]
    }

    /// Read at a fractional delay, linearly interpolating between the two
    /// nearest integer taps. `delay` is clamped to `[0, len − 1]`.
    #[inline]
    pub fn read_frac(&self, delay: f32) -> f32 {
        let max = (self.buf.len() - 1) as f32;
        let d = delay.clamp(0.0, max);
        let i = d as usize;
        let frac = d - i as f32;
        let a = self.read(i);
        let b = self.read((i + 1).min(self.buf.len() - 1));
        a + (b - a) * frac
    }

    /// Convenience: push `x`, then return the sample `delay` samples back.
    #[inline]
    pub fn tick(&mut self, x: f32, delay: usize) -> f32 {
        self.write(x);
        self.read(delay)
    }

    /// Clear the buffer and reset the write head.
    pub fn reset(&mut self) {
        self.buf.fill(0.0);
        self.write = 0;
    }
}
