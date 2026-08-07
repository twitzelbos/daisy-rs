//! `PadDrone` — the "ambient bed from my input" generator (pad/drone).
//!
//! Feed it a mono-summed signal; hit **freeze/hold** and the current sound
//! blooms into an infinitely sustained, reverberant bed you can play over. The
//! texture source is switchable between two engines:
//!
//! - [`Texture::Freeze`] — capture-and-loop the exact moment ([`Freeze`]).
//! - [`Texture::Granular`] — a cloud of overlapping grains ([`Granular`]),
//!   evolving and lush, keeps the timbre, works on chords.
//!
//! Both feed the shared [`FdnReverb`] ambience; `PadDrone` owns the dry↔pad blend
//! and, within the pad, the balance between the texture's own (stereo) output and
//! the reverb. A slow [`Env`] swells the pad in on engage and tails it out.
//!
//! Signal path:
//! ```text
//!   in ─┬───────────────────────────────────────────────► dry ──┐
//!       └─►[freeze | granular]─►×env─┬────────────────► ×(1-mix) ─┤
//!                                    └─►Σ→mono→[reverb]─► ×mix ───┴─► ×pad ─► out
//! ```
//!
//! Economical: per block it sums to mono once, runs **only the active** texture,
//! one FDN pass, and a handful of mults — no allocation, all scratch on the stack
//! (`≤ MAX_BLOCK`). The two texture engines each own a capture buffer (place them
//! in SDRAM); only the active one is processed.

use crate::env::Env;
use crate::freeze::Freeze;
use crate::granular::Granular;
use crate::reverb::FdnReverb;
use crate::{DspProcessor, MAX_BLOCK};

/// Which texture engine feeds the reverb.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Texture {
    /// Capture-and-loop the current moment (seamless, exact).
    Freeze,
    /// A cloud of overlapping windowed grains (evolving, lush).
    Granular,
}

/// Ambient pad/drone generator: freeze or granulate the input into a
/// reverberant bed.
pub struct PadDrone<'a> {
    freeze: Freeze<'a>,
    granular: Granular<'a>,
    reverb: FdnReverb<'a>,
    env: Env,
    mode: Texture,
    dry: f32,
    pad: f32,
    rev_mix: f32, // within the pad: 0 = texture only, 1 = reverb only
}

impl<'a> PadDrone<'a> {
    /// Build over three borrowed buffers: `freeze_buf` for the freeze loop,
    /// `granular_buf` (**power of two ≥ 4096**) for the grain cloud — both sized
    /// for the loop/history length you want (seconds, in SDRAM) — and
    /// `reverb_buf` (≥ [`FdnReverb::REQUIRED_BUF`]).
    ///
    /// - `rt60_s` / `damping_hz`: the reverb tail (size/tone of the ambience).
    /// - `swell_s`: attack/release of the pad swell.
    /// - `xfade_samples`: freeze loop-seam crossfade length.
    /// - `seed`: granular spray seed (determinism).
    ///
    /// Starts in [`Texture::Freeze`].
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        freeze_buf: &'a mut [f32],
        granular_buf: &'a mut [f32],
        reverb_buf: &'a mut [f32],
        sample_rate: f32,
        rt60_s: f32,
        damping_hz: f32,
        swell_s: f32,
        xfade_samples: usize,
        seed: u32,
    ) -> Self {
        Self {
            freeze: Freeze::new(freeze_buf, xfade_samples),
            granular: Granular::new(granular_buf, sample_rate, seed),
            // Wet-only: PadDrone owns the dry/pad blend below.
            reverb: FdnReverb::new(reverb_buf, sample_rate, rt60_s, damping_hz, 1.0),
            env: Env::new(sample_rate, swell_s, swell_s),
            mode: Texture::Freeze,
            dry: 1.0,
            pad: 0.6,
            rev_mix: 0.7,
        }
    }

    /// Select the texture engine. Switching starts the newly-active engine's
    /// capture from its current buffer contents (play, then freeze, to fill it).
    pub fn set_mode(&mut self, mode: Texture) {
        self.mode = mode;
    }

    /// The active texture engine.
    pub fn mode(&self) -> Texture {
        self.mode
    }

    /// Engage the freeze/hold on the active texture and swell the pad in
    /// (`true`) / release it (`false`).
    pub fn set_freeze(&mut self, on: bool) {
        match self.mode {
            Texture::Freeze => {
                if on {
                    self.freeze.freeze();
                } else {
                    self.freeze.unfreeze();
                }
            }
            Texture::Granular => self.granular.set_frozen(on),
        }
        self.env.gate(on);
    }

    /// Whether the active texture is currently held.
    pub fn is_frozen(&self) -> bool {
        match self.mode {
            Texture::Freeze => self.freeze.is_frozen(),
            Texture::Granular => self.granular.is_frozen(),
        }
    }

    /// Mutable access to the granular engine for tuning (density, grain length,
    /// pitch, pan spread).
    pub fn granular_mut(&mut self) -> &mut Granular<'a> {
        &mut self.granular
    }

    /// Dry (input) level, `0.0..`. Default `1.0`.
    pub fn set_dry(&mut self, level: f32) {
        self.dry = level;
    }

    /// Pad (texture + reverb) level, `0.0..`. Default `0.6`.
    pub fn set_pad(&mut self, level: f32) {
        self.pad = level;
    }

    /// Within the pad, balance the texture's direct output against the reverb,
    /// `0.0` (texture only) … `1.0` (reverb only). Default `0.7`.
    pub fn set_reverb_mix(&mut self, mix: f32) {
        self.rev_mix = mix.clamp(0.0, 1.0);
    }
}

impl DspProcessor for PadDrone<'_> {
    fn process(&mut self, in_l: &[f32], in_r: &[f32], out_l: &mut [f32], out_r: &mut [f32]) {
        let n = in_l.len();
        debug_assert!(n <= MAX_BLOCK && in_r.len() == n && out_l.len() == n && out_r.len() == n);

        // Mono-sum the input once.
        let mut mono = [0.0f32; MAX_BLOCK];
        for i in 0..n {
            mono[i] = 0.5 * (in_l[i] + in_r[i]);
        }

        // Active texture → stereo (freeze is mono, duplicated; granular native).
        let mut tl = [0.0f32; MAX_BLOCK];
        let mut tr = [0.0f32; MAX_BLOCK];
        match self.mode {
            Texture::Freeze => {
                for i in 0..n {
                    let f = self.freeze.tick(mono[i]);
                    tl[i] = f;
                    tr[i] = f;
                }
            }
            Texture::Granular => {
                self.granular
                    .process(&mono[..n], &mut tl[..n], &mut tr[..n]);
            }
        }

        // Swell the texture and build the mono reverb send.
        let mut send = [0.0f32; MAX_BLOCK];
        for i in 0..n {
            let e = self.env.tick();
            tl[i] *= e;
            tr[i] *= e;
            send[i] = 0.5 * (tl[i] + tr[i]);
        }

        let mut wet_l = [0.0f32; MAX_BLOCK];
        let mut wet_r = [0.0f32; MAX_BLOCK];
        self.reverb
            .process(&send[..n], &mut wet_l[..n], &mut wet_r[..n]);

        // Blend: dry input + pad·((1−mix)·texture + mix·reverb).
        let (dm, rm) = (1.0 - self.rev_mix, self.rev_mix);
        for i in 0..n {
            out_l[i] = self.dry * in_l[i] + self.pad * (dm * tl[i] + rm * wet_l[i]);
            out_r[i] = self.dry * in_r[i] + self.pad * (dm * tr[i] + rm * wet_r[i]);
        }
    }

    fn reset(&mut self) {
        self.freeze.reset();
        self.granular.reset();
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

    fn make<'a>(fz: &'a mut [f32], gr: &'a mut [f32], rev: &'a mut [f32]) -> PadDrone<'a> {
        PadDrone::new(fz, gr, rev, SR, 2.0, 10_000.0, 0.05, 64, 0xC0FFEE)
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

    fn tone(hz: f32) -> impl Fn(usize) -> f32 {
        move |n| libm::sinf(core::f32::consts::TAU * hz * n as f32 / SR)
    }

    #[test]
    fn silence_stays_silent_and_finite() {
        let mut fz = vec![0.0f32; 4096];
        let mut gr = vec![0.0f32; 1 << 14];
        let mut rev = vec![0.0f32; FdnReverb::REQUIRED_BUF];
        let mut pad = make(&mut fz, &mut gr, &mut rev);
        let (ol, or) = run(&mut pad, 2048, |_| 0.0);
        assert!(ol.iter().chain(&or).all(|x| x.is_finite()));
        assert!(ol.iter().chain(&or).all(|&x| x.abs() < 1e-6));
    }

    #[test]
    fn freeze_mode_sustains_after_input_stops() {
        let mut fz = vec![0.0f32; 4096];
        let mut gr = vec![0.0f32; 1 << 14];
        let mut rev = vec![0.0f32; FdnReverb::REQUIRED_BUF];
        let mut pad = make(&mut fz, &mut gr, &mut rev);
        assert_eq!(pad.mode(), Texture::Freeze);
        run(&mut pad, 4096, tone(220.0));
        pad.set_freeze(true);
        assert!(pad.is_frozen());
        let (ol, or) = run(&mut pad, 8192, |_| 0.0);
        assert!(ol.iter().chain(&or).all(|x| x.is_finite()));
        let tail: f32 = ol[4096..].iter().map(|x| x * x).sum();
        assert!(tail > 1e-3, "freeze pad did not sustain (energy {tail})");
        assert!(ol.iter().chain(&or).all(|x| x.abs() < 8.0));
    }

    #[test]
    fn granular_mode_sustains_after_input_stops() {
        let mut fz = vec![0.0f32; 4096];
        let mut gr = vec![0.0f32; 1 << 14];
        let mut rev = vec![0.0f32; FdnReverb::REQUIRED_BUF];
        let mut pad = make(&mut fz, &mut gr, &mut rev);
        pad.set_mode(Texture::Granular);
        assert_eq!(pad.mode(), Texture::Granular);
        run(&mut pad, 1 << 14, tone(147.0)); // fill the granular history
        pad.set_freeze(true);
        assert!(pad.is_frozen());
        let (ol, or) = run(&mut pad, 24_000, |_| 0.0);
        assert!(ol.iter().chain(&or).all(|x| x.is_finite()));
        let tail: f32 = ol[12_000..].iter().map(|x| x * x).sum();
        assert!(tail > 1e-3, "granular pad did not sustain (energy {tail})");
        assert!(ol.iter().chain(&or).all(|x| x.abs() < 8.0));
    }

    #[test]
    fn dry_passes_when_pad_muted() {
        let mut fz = vec![0.0f32; 4096];
        let mut gr = vec![0.0f32; 1 << 14];
        let mut rev = vec![0.0f32; FdnReverb::REQUIRED_BUF];
        let mut pad = make(&mut fz, &mut gr, &mut rev);
        pad.set_pad(0.0); // pad muted → pure dry path
        let (ol, _or) = run(&mut pad, 256, |n| if n == 0 { 1.0 } else { 0.0 });
        assert_eq!(ol[0], 1.0);
        assert!(ol[1..].iter().all(|&x| x == 0.0));
    }
}
