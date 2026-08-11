//! `SamplePad` — a **multisampled** rompler voice: the Atmosphere-style path.
//!
//! Plays back **recorded** samples (a choir "ah", a pad — borrowed read-only,
//! flash/SDRAM), **crossfade-looped** for infinite sustain and **pitched** to a
//! chord. Multisampling: the pad holds a keymap of [`Zone`]s (one recording per
//! pitch) and plays each note from the **nearest-pitch** zone, so nothing is
//! stretched more than a semitone or two — the formants stay natural (stretching
//! one sample across octaves is the classic rompler "chipmunk" artifact).
//!
//! Per voice it fractionally reads its zone's loop (linear interp) and advances
//! at `target_freq / zone.base_freq` (× sample-rate ratio); a short live
//! crossfade at the loop seam keeps the sustain click-free at any rate.
//!
//! A real choir recording is *already* an ensemble, so the default is one voice
//! per note with no detune — extra detuned copies of an ensemble just swirl.

use crate::noise::Prng;

/// Maximum simultaneous voices (chord notes × ensemble).
pub const MAX_VOICES: usize = 24;

/// One keymap zone: a recording and the musical pitch it was recorded at, plus
/// its sustain-loop region.
#[derive(Debug, Clone, Copy)]
pub struct Zone<'a> {
    /// The recorded sample data (read-only).
    pub sample: &'a [f32],
    /// The musical pitch (Hz) the sample was recorded at.
    pub base_freq: f32,
    /// Start index of the sustain loop within `sample`.
    pub loop_start: usize,
    /// Length of the sustain loop (samples).
    pub loop_len: usize,
}

impl<'a> Zone<'a> {
    /// A zone looping the whole sample.
    #[must_use]
    pub fn new(sample: &'a [f32], base_freq: f32) -> Self {
        let loop_len = sample.len();
        Self {
            sample,
            base_freq,
            loop_start: 0,
            loop_len,
        }
    }

    /// Restrict the sustain loop to `[start, start+len)` (e.g. skip an attack).
    #[must_use]
    pub fn with_loop(mut self, start: usize, len: usize) -> Self {
        let end = (start + len).min(self.sample.len());
        self.loop_start = start.min(self.sample.len().saturating_sub(2));
        self.loop_len = end.saturating_sub(self.loop_start).max(2);
        self
    }
}

#[derive(Debug, Clone, Copy)]
struct Voice {
    zone: u8,
    phase: f32, // position within the zone's loop, [0, loop_len)
    rate: f32,  // samples advanced per output sample (pitch × sr ratio)
    pan_l: f32,
    pan_r: f32,
}

impl Voice {
    const SILENT: Self = Self {
        zone: 0,
        phase: 0.0,
        rate: 1.0,
        pan_l: 0.0,
        pan_r: 0.0,
    };
}

/// A multisampled sample-playback pad over a borrowed keymap of [`Zone`]s.
#[derive(Debug)]
pub struct SamplePad<'a> {
    zones: &'a [Zone<'a>],
    sr_ratio: f32, // sample_rate / output_rate
    xfade: usize,
    voices: [Voice; MAX_VOICES],
    n_voices: usize,
    rng: Prng,
    voices_per_note: usize,
    detune_cents: f32,
    level: f32,
}

#[inline(always)]
fn interp(s: &[f32], p: f32) -> f32 {
    let i = p as usize;
    if i + 1 >= s.len() {
        return s[s.len() - 1];
    }
    let frac = p - i as f32;
    s[i] + (s[i + 1] - s[i]) * frac
}

impl<'a> SamplePad<'a> {
    /// Build over a keymap of `zones` (all at `sample_rate`), played at
    /// `output_rate`; `xfade` is the loop-seam crossfade length.
    ///
    /// # Panics
    /// Panics if `zones` is empty.
    #[must_use]
    pub fn new(
        zones: &'a [Zone<'a>],
        sample_rate: f32,
        output_rate: f32,
        xfade: usize,
        seed: u32,
    ) -> SamplePad<'a> {
        assert!(!zones.is_empty(), "SamplePad needs at least one zone");
        SamplePad {
            zones,
            sr_ratio: sample_rate / output_rate,
            xfade,
            voices: [Voice::SILENT; MAX_VOICES],
            n_voices: 0,
            rng: Prng::new(seed),
            voices_per_note: 1,
            detune_cents: 0.0,
            level: 0.5,
        }
    }

    /// Ensemble voices per note (default 1 — a real choir sample is already an
    /// ensemble). >1 with detune fattens a *dry/single* sample.
    pub fn set_voices_per_note(&mut self, n: usize) {
        self.voices_per_note = n.max(1);
    }

    /// Ensemble detune spread (cents, default 0).
    pub fn set_detune(&mut self, cents: f32) {
        self.detune_cents = cents.max(0.0);
    }

    /// Output level (default 0.5).
    pub fn set_level(&mut self, level: f32) {
        self.level = level;
    }

    /// Nearest zone to `freq` by octave distance.
    fn nearest_zone(&self, freq: f32) -> usize {
        let mut best = 0usize;
        let mut best_d = f32::INFINITY;
        for (i, z) in self.zones.iter().enumerate() {
            let d = libm::fabsf(libm::log2f(freq / z.base_freq));
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        best
    }

    /// Voice the pad to a chord (note frequencies, Hz).
    pub fn set_chord(&mut self, notes: &[f32]) {
        self.n_voices = 0;
        'notes: for &f in notes {
            for _ in 0..self.voices_per_note {
                if self.n_voices >= MAX_VOICES {
                    break 'notes;
                }
                let cents = self.detune_cents * self.rng.next_f32();
                let ftuned = f * libm::powf(2.0, cents / 1200.0);
                let zi = self.nearest_zone(ftuned);
                let z = &self.zones[zi];
                let pan = 0.5 + 0.35 * self.rng.next_f32();
                self.voices[self.n_voices] = Voice {
                    zone: zi as u8,
                    phase: self.rng.next_f32().abs() * z.loop_len as f32,
                    rate: (ftuned / z.base_freq) * self.sr_ratio,
                    pan_l: libm::cosf(pan * core::f32::consts::FRAC_PI_2),
                    pan_r: libm::sinf(pan * core::f32::consts::FRAC_PI_2),
                };
                self.n_voices += 1;
            }
        }
    }

    /// Silence.
    pub fn clear(&mut self) {
        self.n_voices = 0;
    }

    /// Render a stereo block.
    pub fn process(&mut self, out_l: &mut [f32], out_r: &mut [f32]) {
        let n = out_l.len();
        for i in 0..n {
            let (mut l, mut r) = (0.0f32, 0.0f32);
            for v in &mut self.voices[..self.n_voices] {
                let z = &self.zones[v.zone as usize];
                let ls = z.loop_start as f32;
                let ll = z.loop_len as f32;
                let xf = self.xfade.min(z.loop_len / 4).max(1) as f32;
                let tail_off = (z.loop_start + z.loop_len) as f32 - xf;
                let main = interp(z.sample, ls + v.phase);
                // Crossfade the loop tail into the head over the first `xfade`
                // samples after the wrap → seamless sustain at any rate.
                let s = if v.phase < xf {
                    let g = 1.0 - v.phase / xf;
                    g * interp(z.sample, tail_off + v.phase) + (1.0 - g) * main
                } else {
                    main
                };
                l += s * v.pan_l;
                r += s * v.pan_r;
                v.phase += v.rate;
                if v.phase >= ll {
                    v.phase -= ll;
                }
            }
            out_l[i] = l * self.level;
            out_r[i] = r * self.level;
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec;
    use std::vec::Vec;

    const SR: f32 = 48_000.0;

    fn sine(freq: f32) -> Vec<f32> {
        (0..SR as usize)
            .map(|i| libm::sinf(core::f32::consts::TAU * freq * i as f32 / SR))
            .collect()
    }

    fn run(p: &mut SamplePad, n: usize) -> (Vec<f32>, Vec<f32>) {
        let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
        let mut i = 0;
        while i < n {
            let b = 64.min(n - i);
            p.process(&mut l[i..i + b], &mut r[i..i + b]);
            i += b;
        }
        (l, r)
    }

    #[test]
    fn plays_a_chord_finite_and_bounded() {
        let (s220, s440) = (sine(220.0), sine(440.0));
        let zones = [Zone::new(&s220, 220.0), Zone::new(&s440, 440.0)];
        let mut pad = SamplePad::new(&zones, SR, SR, 2048, 1);
        pad.set_chord(&[220.0, 277.18, 329.63]);
        let (l, r) = run(&mut pad, 24_000);
        assert!(l.iter().chain(&r).all(|x| x.is_finite()));
        assert!(l.iter().chain(&r).all(|x| x.abs() < 4.0));
        let e: f32 = l[8_000..].iter().map(|x| x * x).sum();
        assert!(e > 1.0, "no output (energy {e})");
    }

    #[test]
    fn picks_nearest_zone() {
        // A 400 Hz note is closer to the 440 zone than the 220 zone → rate ~0.9.
        let (s220, s440) = (sine(220.0), sine(440.0));
        let zones = [Zone::new(&s220, 220.0), Zone::new(&s440, 440.0)];
        let mut pad = SamplePad::new(&zones, SR, SR, 2048, 2);
        pad.set_chord(&[400.0]);
        // Nearest is zone 1 (440): rate = 400/440 ≈ 0.909, not 400/220 ≈ 1.82.
        assert_eq!(pad.voices[0].zone, 1);
        assert!((pad.voices[0].rate - 400.0 / 440.0).abs() < 1e-4);
    }

    #[test]
    fn silent_until_voiced() {
        let s = sine(220.0);
        let zones = [Zone::new(&s, 220.0)];
        let mut pad = SamplePad::new(&zones, SR, SR, 2048, 3);
        let (l, r) = run(&mut pad, 1000);
        assert!(l.iter().chain(&r).all(|&x| x == 0.0));
    }
}
