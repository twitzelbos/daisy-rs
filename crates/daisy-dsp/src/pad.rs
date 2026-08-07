//! `PadDrone` — the "ambient bed from my input" MVP (pad/drone generator,
//! phase 1: *freeze + reverb*).
//!
//! Feed it a mono-summed signal; hit **freeze** and the current sound blooms
//! into an infinitely sustained, reverberant bed you can play over. It composes
//! two [`crate::DspProcessor`]-era primitives — [`Freeze`] (infinite sustain) and
//! [`FdnReverb`] (the ambience) — behind the one [`DspProcessor`] contract, so
//! the three-tier test framework and the Hothouse app drive it uniformly.
//!
//! Signal path (per the design doc):
//! ```text
//!   in ─┬─────────────────────────────────► dry ──┐
//!       └─► [freeze] ─► ×env(swell) ─► [reverb] ─► pad ─┴─► out (stereo)
//! ```
//! The reverb is built **wet-only**; `PadDrone` owns the dry↔pad blend. The
//! frozen layer is multiplied by a slow attack/release [`Env`] so engaging freeze
//! *swells in* and releasing it *tails out*, rather than clicking.
//!
//! Economical by construction: per block it sums to mono once, ticks freeze+env
//! (a read + an add each), and runs the FDN once — no allocation, all scratch on
//! the stack (`≤ MAX_BLOCK`).

use crate::env::Env;
use crate::freeze::Freeze;
use crate::reverb::FdnReverb;
use crate::{DspProcessor, MAX_BLOCK};

/// Ambient pad/drone generator: freeze the input into a reverberant bed.
pub struct PadDrone<'a> {
    freeze: Freeze<'a>,
    reverb: FdnReverb<'a>,
    env: Env,
    dry: f32,
    pad: f32,
}

impl<'a> PadDrone<'a> {
    /// Build over two borrowed buffers: `capture_buf` for the freeze loop (size
    /// it for the loop length you want — seconds, in SDRAM) and `reverb_buf`
    /// (≥ [`FdnReverb::REQUIRED_BUF`]).
    ///
    /// - `sample_rate`: Hz.
    /// - `rt60_s` / `damping_hz`: the reverb tail (size/tone of the ambience).
    /// - `swell_s`: attack/release of the freeze swell.
    /// - `xfade_samples`: freeze loop-seam crossfade length.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        capture_buf: &'a mut [f32],
        reverb_buf: &'a mut [f32],
        sample_rate: f32,
        rt60_s: f32,
        damping_hz: f32,
        swell_s: f32,
        xfade_samples: usize,
    ) -> Self {
        Self {
            freeze: Freeze::new(capture_buf, xfade_samples),
            // Wet-only: PadDrone owns the dry/pad blend below.
            reverb: FdnReverb::new(reverb_buf, sample_rate, rt60_s, damping_hz, 1.0),
            env: Env::new(sample_rate, swell_s, swell_s),
            dry: 1.0,
            pad: 0.6,
        }
    }

    /// Engage (`true`) or release (`false`) the freeze/hold. Also gates the
    /// swell envelope so the bed fades in and out.
    pub fn set_freeze(&mut self, on: bool) {
        if on {
            self.freeze.freeze();
        } else {
            self.freeze.unfreeze();
        }
        self.env.gate(on);
    }

    /// Whether the bed is currently held.
    pub fn is_frozen(&self) -> bool {
        self.freeze.is_frozen()
    }

    /// Dry (input) level, `0.0..`. Default `1.0`.
    pub fn set_dry(&mut self, level: f32) {
        self.dry = level;
    }

    /// Pad (frozen + reverb) level, `0.0..`. Default `0.6`.
    pub fn set_pad(&mut self, level: f32) {
        self.pad = level;
    }
}

impl DspProcessor for PadDrone<'_> {
    fn process(&mut self, in_l: &[f32], in_r: &[f32], out_l: &mut [f32], out_r: &mut [f32]) {
        let n = in_l.len();
        debug_assert!(n <= MAX_BLOCK && in_r.len() == n && out_l.len() == n && out_r.len() == n);

        // Freeze the mono sum, swell it, then feed the reverb — all in one pass.
        let mut mono = [0.0f32; MAX_BLOCK];
        for i in 0..n {
            let m = 0.5 * (in_l[i] + in_r[i]);
            mono[i] = self.freeze.tick(m) * self.env.tick();
        }

        let mut wet_l = [0.0f32; MAX_BLOCK];
        let mut wet_r = [0.0f32; MAX_BLOCK];
        self.reverb
            .process(&mono[..n], &mut wet_l[..n], &mut wet_r[..n]);

        for i in 0..n {
            out_l[i] = self.dry * in_l[i] + self.pad * wet_l[i];
            out_r[i] = self.dry * in_r[i] + self.pad * wet_r[i];
        }
    }

    fn reset(&mut self) {
        self.freeze.reset();
        self.reverb.reset();
        self.env.reset();
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec;
    use std::vec::Vec;

    const SR: f32 = 48_000.0;

    fn make<'a>(cap: &'a mut [f32], rev: &'a mut [f32]) -> PadDrone<'a> {
        PadDrone::new(cap, rev, SR, 2.0, 10_000.0, 0.05, 64)
    }

    // Drive the processor block-by-block over a mono tone in both channels.
    fn run(pad: &mut PadDrone, samples: usize, gen: impl Fn(usize) -> f32) -> (Vec<f32>, Vec<f32>) {
        let (mut ol, mut or) = (Vec::new(), Vec::new());
        let mut i = 0;
        while i < samples {
            let n = MAX_BLOCK.min(samples - i);
            let il: Vec<f32> = (0..n).map(|k| gen(i + k)).collect();
            let ir = il.clone();
            let mut bl = vec![0.0f32; n];
            let mut br = vec![0.0f32; n];
            pad.process(&il, &ir, &mut bl, &mut br);
            ol.extend_from_slice(&bl);
            or.extend_from_slice(&br);
            i += n;
        }
        (ol, or)
    }

    #[test]
    fn silence_stays_silent_and_finite() {
        let mut cap = vec![0.0f32; 4096];
        let mut rev = vec![0.0f32; FdnReverb::REQUIRED_BUF];
        let mut pad = make(&mut cap, &mut rev);
        let (ol, or) = run(&mut pad, 2048, |_| 0.0);
        assert!(ol.iter().chain(&or).all(|x| x.is_finite()));
        assert!(ol.iter().chain(&or).all(|&x| x.abs() < 1e-6));
    }

    #[test]
    fn freeze_sustains_after_input_stops() {
        let mut cap = vec![0.0f32; 4096];
        let mut rev = vec![0.0f32; FdnReverb::REQUIRED_BUF];
        let mut pad = make(&mut cap, &mut rev);
        // Play a tone to fill the capture buffer, then freeze and go silent.
        let tone = |n: usize| libm::sinf(core::f32::consts::TAU * 220.0 * n as f32 / SR);
        run(&mut pad, 4096, tone);
        pad.set_freeze(true);
        assert!(pad.is_frozen());
        let (ol, or) = run(&mut pad, 8192, |_| 0.0); // no input now
        assert!(ol.iter().chain(&or).all(|x| x.is_finite()));
        // The bed sustains: real energy after the input has stopped.
        let tail_energy: f32 = ol[4096..].iter().map(|x| x * x).sum();
        assert!(
            tail_energy > 1e-3,
            "pad did not sustain (energy {tail_energy})"
        );
        // And stays bounded (stable feedback).
        assert!(ol.iter().chain(&or).all(|x| x.abs() < 8.0));
    }

    #[test]
    fn dry_passes_when_not_frozen() {
        let mut cap = vec![0.0f32; 4096];
        let mut rev = vec![0.0f32; FdnReverb::REQUIRED_BUF];
        let mut pad = make(&mut cap, &mut rev);
        pad.set_pad(0.0); // pad muted → pure dry path
        let (ol, _or) = run(&mut pad, 256, |n| if n == 0 { 1.0 } else { 0.0 });
        assert_eq!(ol[0], 1.0);
        assert!(ol[1..].iter().all(|&x| x == 0.0));
    }
}
