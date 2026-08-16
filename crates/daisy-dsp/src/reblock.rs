//! Block-size adaptation for streaming audio — a fixed-capacity, `no_std`,
//! no-alloc mono sample FIFO.
//!
//! The codec callback delivers a fixed chunk of frames (48 on the Daisy) but a
//! block processor — the FFT [`Convolver`](crate::convolution::Convolver) — must
//! be driven a power-of-two block at a time (64). Since 48 and 64 are not
//! multiples, the remainder that carries between callbacks varies (0..63), so a
//! small ring FIFO is the clean adapter: push the callback's samples in, pull
//! whole `B`-sample blocks out when enough have accumulated, and the remainder
//! carries automatically.
//!
//! For the HRIR / binaural path the composition is: **one input FIFO** (mono
//! source) feeding the convolver, and **two output FIFOs** (the convolver's L/R
//! result) drained back into the codec output. There is a priming latency — the
//! output FIFOs do not hold a full callback's worth until a couple of blocks
//! have been processed — during which the output side reads empty; callers treat
//! a `false` from [`SampleFifo::pop`] as "emit silence this block".
//!
//! ```
//! use daisy_dsp::reblock::SampleFifo;
//!
//! // Adapt 48-sample callback chunks to 64-sample convolver blocks.
//! let mut fifo = SampleFifo::<128>::new();
//! let chunk = [0.0f32; 48];
//! fifo.extend(&chunk);          // callback pushes 48
//! let mut block = [0.0f32; 64];
//! assert!(!fifo.pop(&mut block)); // only 48 buffered — not a full 64-block yet
//! fifo.extend(&chunk);          // next callback: 96 buffered
//! assert!(fifo.pop(&mut block)); // now a full 64-block comes out (32 carry)
//! assert_eq!(fifo.len(), 32);
//! ```

/// A fixed-capacity mono sample ring buffer for block-size adaptation.
///
/// `CAP` must be large enough to hold the largest carry plus one callback chunk
/// without overflow — for the 48-in / 64-block case, `CAP = 128` is ample
/// (max carry 63 + a 48-chunk = 111).
#[derive(Debug)]
pub struct SampleFifo<const CAP: usize> {
    buf: [f32; CAP],
    head: usize, // index of the next sample to read
    len: usize,  // samples currently buffered
}

impl<const CAP: usize> SampleFifo<CAP> {
    /// An empty FIFO.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buf: [0.0; CAP],
            head: 0,
            len: 0,
        }
    }

    /// Samples currently buffered.
    #[must_use]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether no samples are buffered.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Total capacity.
    #[must_use]
    pub fn capacity(&self) -> usize {
        CAP
    }

    /// Free space remaining.
    #[must_use]
    pub fn free(&self) -> usize {
        CAP - self.len
    }

    /// Push one sample. Returns `false` (dropping the sample) if the FIFO is
    /// full — a full FIFO signals the consumer is not draining fast enough.
    pub fn push(&mut self, x: f32) -> bool {
        if self.len == CAP {
            return false;
        }
        let tail = (self.head + self.len) % CAP;
        self.buf[tail] = x;
        self.len += 1;
        true
    }

    /// Push a slice, returning how many samples were accepted (fewer than
    /// `xs.len()` only if the FIFO filled).
    pub fn extend(&mut self, xs: &[f32]) -> usize {
        let mut n = 0;
        for &x in xs {
            if !self.push(x) {
                break;
            }
            n += 1;
        }
        n
    }

    /// If at least `out.len()` samples are buffered, remove that many into `out`
    /// (oldest first) and return `true`. Otherwise leave `out` untouched and the
    /// FIFO unchanged, returning `false`.
    pub fn pop(&mut self, out: &mut [f32]) -> bool {
        if self.len < out.len() {
            return false;
        }
        for slot in out.iter_mut() {
            *slot = self.buf[self.head];
            self.head = (self.head + 1) % CAP;
            self.len -= 1;
        }
        true
    }

    /// Drop all buffered samples.
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }
}

impl<const CAP: usize> Default for SampleFifo<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    #[test]
    fn push_pop_roundtrip_preserves_order() {
        let mut f = SampleFifo::<16>::new();
        assert_eq!(f.extend(&[1.0, 2.0, 3.0, 4.0]), 4);
        assert_eq!(f.len(), 4);
        let mut out = [0.0f32; 3];
        assert!(f.pop(&mut out));
        assert_eq!(out, [1.0, 2.0, 3.0]);
        assert_eq!(f.len(), 1);
        // Not enough for another 3-block.
        assert!(!f.pop(&mut out));
        assert_eq!(f.len(), 1);
    }

    #[test]
    fn overflow_drops_and_reports_short_count() {
        let mut f = SampleFifo::<4>::new();
        assert_eq!(f.extend(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]), 4); // only 4 fit
        assert_eq!(f.len(), 4);
        assert!(!f.push(9.0)); // full
        let mut out = [0.0f32; 4];
        assert!(f.pop(&mut out));
        assert_eq!(out, [1.0, 2.0, 3.0, 4.0]); // 5,6 were dropped, not wrapped in
    }

    #[test]
    fn wraps_around_capacity() {
        let mut f = SampleFifo::<4>::new();
        f.extend(&[1.0, 2.0, 3.0]);
        let mut out = [0.0f32; 2];
        f.pop(&mut out); // head now at 2, len 1
        f.extend(&[4.0, 5.0, 6.0]); // wraps past the end
        assert_eq!(f.len(), 4);
        let mut all = [0.0f32; 4];
        assert!(f.pop(&mut all));
        assert_eq!(all, [3.0, 4.0, 5.0, 6.0]);
    }

    // The real workload: 48-sample callback chunks in, 64-sample convolver
    // blocks out, and the same on the way back — assert not one sample is lost
    // or duplicated across the 48/64 boundary over many cycles (an identity
    // "convolver" makes the whole path a pure delay).
    #[test]
    fn reblock_48_to_64_is_sample_exact() {
        const IN: usize = 48;
        const B: usize = 64;
        let mut in_fifo = SampleFifo::<128>::new();
        let mut out_fifo = SampleFifo::<256>::new();

        // A recognisable ramp so any drop/dup/reorder shows up.
        let total = IN * 400;
        let src: Vec<f32> = (0..total).map(|i| i as f32).collect();

        let mut produced: Vec<f32> = Vec::new();
        let mut block = [0.0f32; B];
        let mut chunk_out = [0.0f32; IN];

        for chunk in src.chunks_exact(IN) {
            in_fifo.extend(chunk);
            // Drain as many whole blocks as are ready through the identity stage.
            while in_fifo.pop(&mut block) {
                out_fifo.extend(&block);
            }
            // Emit one callback's worth if primed (else this callback is silent).
            if out_fifo.pop(&mut chunk_out) {
                produced.extend_from_slice(&chunk_out);
            }
        }

        // Everything produced must be a contiguous prefix of the source (a pure
        // delay), with no gaps: produced[i] == src[i] for all produced samples.
        assert!(!produced.is_empty());
        for (i, &v) in produced.iter().enumerate() {
            assert_eq!(v, src[i], "sample {i} diverged");
        }
        // And the pipeline should keep up: latency is bounded (< 2 blocks), so we
        // lose at most a couple of callbacks of output to priming.
        assert!(produced.len() >= total - 2 * B - IN);
    }
}
