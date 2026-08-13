//! DSP-in-the-loop: route the USB-audio loopback through a `daisy-dsp` core so
//! it can be validated on real M7 silicon by its frequency response (see
//! `tools/usb-loopback/`). Enabled by the `dsp-loop` feature.
//!
//! First core: a stereo biquad low-pass. The host golden (a matching RBJ biquad
//! run over the same input) is generated from the SAME public parameters below,
//! so `analyze.py spectral` compares the device's realized response against the
//! reference. Because the USB *audio* path is spectrally faithful but not
//! sample-accurate (macOS SRCs each stream), this is a spectral check, not
//! bit-exact — the bit-exact path is the CDC block-exchange test (separate).

use daisy_dsp::filter::Biquad;

/// Sample rate of the UAC streams.
pub const SAMPLE_RATE: f32 = 48_000.0;
/// Low-pass cutoff — kept well inside the band so the roll-off is unambiguous.
pub const CUTOFF_HZ: f32 = 1_000.0;
/// Butterworth Q (maximally flat, no resonant peak → no clipping risk).
pub const Q: f32 = 0.707;

/// Per-channel biquad state for the stereo loopback.
pub struct DspLoop {
    left: Biquad,
    right: Biquad,
}

impl DspLoop {
    #[must_use]
    pub fn new() -> Self {
        Self {
            left: Biquad::lowpass(SAMPLE_RATE, CUTOFF_HZ, Q),
            right: Biquad::lowpass(SAMPLE_RATE, CUTOFF_HZ, Q),
        }
    }

    /// Filter an interleaved i16-LE stereo buffer in place (L,R,L,R…). A partial
    /// trailing frame (buffer not a multiple of 4 bytes) is left untouched.
    pub fn process_i16le_stereo(&mut self, buf: &mut [u8]) {
        for frame in buf.chunks_exact_mut(4) {
            let l = i16::from_le_bytes([frame[0], frame[1]]) as f32 / 32768.0;
            let r = i16::from_le_bytes([frame[2], frame[3]]) as f32 / 32768.0;
            let lo = (self.left.tick(l).clamp(-1.0, 1.0) * 32767.0) as i16;
            let ro = (self.right.tick(r).clamp(-1.0, 1.0) * 32767.0) as i16;
            frame[0..2].copy_from_slice(&lo.to_le_bytes());
            frame[2..4].copy_from_slice(&ro.to_le_bytes());
        }
    }
}

impl Default for DspLoop {
    fn default() -> Self {
        Self::new()
    }
}
