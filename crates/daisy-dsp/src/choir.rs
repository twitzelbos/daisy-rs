//! `ChoirVoice` — an additive vocal/choir synth voice (the "choir of angels").
//!
//! Feed it a chord (a set of note frequencies) and it sings it as a choir. What
//! makes additive sines read as *voices* rather than an organ:
//!
//! - **Formants** — each harmonic is scaled by a vowel resonance envelope
//!   ("ah"/"ooh"), so the timbre has a mouth/vowel colour.
//! - **Per-singer vibrato** — every singer has its own ~5–6 Hz, ~1.5 % pitch
//!   vibrato. A single vibrato *beats*; a choir's many **decorrelated** vibratos
//!   average into a lush ensemble shimmer instead.
//! - **Detuned ensemble** — several singers per note, spread ±cents, panned
//!   across the field → the "many humans" richness.
//! - **Breath** — a slow per-singer amplitude drift for life.
//!
//! Economical: partials are plain phase-accumulator oscillators reading a shared
//! sine table (no `sin` per sample); vibrato/breath update at **control rate**
//! (once per `process` block), so the per-sample cost is a table lookup + a
//! couple of MACs per partial. Deterministic — the ensemble spread comes from a
//! seeded [`Prng`], so two units sound identical and tests reproduce.

use crate::filter::OnePole;
use crate::noise::Prng;

const LUT_N: usize = 512;

/// Interpolated sine-table lookup. A free function (not a `&self` method) so the
/// per-sample loop can read the table while iterating `partials`/`singers`
/// mutably — those are disjoint fields.
#[inline(always)]
fn lut_at(lut: &[f32; LUT_N], phase: f32) -> f32 {
    let x = phase * LUT_N as f32;
    let i = x as usize;
    let frac = x - i as f32;
    let a = lut[i & (LUT_N - 1)];
    let b = lut[(i + 1) & (LUT_N - 1)];
    a + (b - a) * frac
}
/// Oscillator-bank capacity (notes × singers × harmonics).
pub const MAX_PARTIALS: usize = 256;
/// Singer capacity (notes × singers-per-note).
pub const MAX_SINGERS: usize = 24;

/// A vowel, as a set of formant resonances `(centre_hz, gain, bandwidth_hz)`.
#[derive(Clone, Copy)]
pub enum Vowel {
    /// Open, present — the classic choir "ah".
    Ah,
    /// Rounder, darker, more ethereal — "ooh".
    Ooh,
}

impl Vowel {
    fn formants(self) -> &'static [(f32, f32, f32)] {
        match self {
            Vowel::Ah => &[
                (700.0, 1.0, 90.0),
                (1150.0, 0.65, 100.0),
                (2600.0, 0.4, 120.0),
                (3300.0, 0.28, 150.0),
                (4300.0, 0.12, 180.0),
            ],
            Vowel::Ooh => &[
                (350.0, 1.0, 60.0),
                (800.0, 0.5, 90.0),
                (2700.0, 0.16, 120.0),
                (3300.0, 0.1, 150.0),
            ],
        }
    }

    /// Resonance gain at `freq` (sum of the formant peaks). No floor: harmonics
    /// far from every formant are strongly suppressed, so the spectrum is *peaked
    /// to the vowel* like a voice — not a full harmonic stack (which reads as an
    /// organ).
    fn gain(self, freq: f32) -> f32 {
        let mut g = 0.0;
        for &(fc, a, bw) in self.formants() {
            let d = (freq - fc) / bw;
            g += a / (1.0 + d * d);
        }
        g
    }
}

#[derive(Clone, Copy)]
struct Partial {
    phase: f32,    // [0, 1)
    base_inc: f32, // harmonic·fundamental / sr
    inc: f32,      // base_inc · current vibrato factor
    amp: f32,      // formant·source amplitude
    cur_amp: f32,  // amp · current breath factor
    pan_l: f32,
    pan_r: f32,
    singer: u8,
}

impl Partial {
    const SILENT: Self = Self {
        phase: 0.0,
        base_inc: 0.0,
        inc: 0.0,
        amp: 0.0,
        cur_amp: 0.0,
        pan_l: 0.0,
        pan_r: 0.0,
        singer: 0,
    };
}

#[derive(Clone, Copy)]
struct Singer {
    vib_phase: f32,
    vib_inc: f32, // vibrato rate / sr
    vib_depth: f32,
    breath_phase: f32,
    breath_inc: f32,
    breath_depth: f32,
}

impl Singer {
    const SILENT: Self = Self {
        vib_phase: 0.0,
        vib_inc: 0.0,
        vib_depth: 0.0,
        breath_phase: 0.0,
        breath_inc: 0.0,
        breath_depth: 0.0,
    };
}

/// An additive choir voice. Build once, `set_chord` to (re)voice, `process` to
/// render a stereo block.
pub struct ChoirVoice {
    sr: f32,
    lut: [f32; LUT_N],
    partials: [Partial; MAX_PARTIALS],
    n_partials: usize,
    singers: [Singer; MAX_SINGERS],
    n_singers: usize,
    rng: Prng,
    vowel: Vowel,
    voices_per_note: usize,
    detune_cents: f32,
    level: f32,
    // Breath: formant-band noise that tracks the choir's level — the "air" that
    // distinguishes voices from a pure (organ-like) harmonic stack.
    breath_rng: Prng,
    air_hp_l: OnePole,
    air_lp_l: OnePole,
    air_hp_r: OnePole,
    air_lp_r: OnePole,
    amp_env: f32,
    breath: f32,
}

impl ChoirVoice {
    /// Build a choir at `sample_rate`. `seed` fixes the ensemble spread.
    pub fn new(sample_rate: f32, seed: u32) -> Self {
        let mut lut = [0.0f32; LUT_N];
        for (i, s) in lut.iter_mut().enumerate() {
            *s = libm::sinf(core::f32::consts::TAU * i as f32 / LUT_N as f32);
        }
        Self {
            sr: sample_rate,
            lut,
            partials: [Partial::SILENT; MAX_PARTIALS],
            n_partials: 0,
            singers: [Singer::SILENT; MAX_SINGERS],
            n_singers: 0,
            rng: Prng::new(seed),
            vowel: Vowel::Ah,
            voices_per_note: 3,
            detune_cents: 9.0,
            level: 0.2,
            breath_rng: Prng::new(seed ^ 0x5555_AAAA),
            air_hp_l: OnePole::highpass(sample_rate, 500.0),
            air_lp_l: OnePole::lowpass(sample_rate, 4000.0),
            air_hp_r: OnePole::highpass(sample_rate, 500.0),
            air_lp_r: OnePole::lowpass(sample_rate, 4000.0),
            amp_env: 0.0,
            breath: 0.16,
        }
    }

    /// Breath-noise amount, `0.0` = pure (organ-like) … default `0.16`.
    pub fn set_breath(&mut self, amount: f32) {
        self.breath = amount.max(0.0);
    }

    /// Vowel colour (default [`Vowel::Ah`]). Applies on the next `set_chord`.
    pub fn set_vowel(&mut self, vowel: Vowel) {
        self.vowel = vowel;
    }

    /// Singers per note (ensemble size), clamped so notes×singers ≤ capacity.
    pub fn set_voices_per_note(&mut self, n: usize) {
        self.voices_per_note = n.max(1);
    }

    /// Ensemble detune spread in cents (default 9).
    pub fn set_detune(&mut self, cents: f32) {
        self.detune_cents = cents.max(0.0);
    }

    /// Output level (default 0.2).
    pub fn set_level(&mut self, level: f32) {
        self.level = level;
    }

    /// Voice the choir to sing `notes` (fundamental frequencies, Hz). Rebuilds
    /// the oscillator bank: notes → detuned singers → formant-shaped harmonics.
    pub fn set_chord(&mut self, notes: &[f32]) {
        self.n_partials = 0;
        self.n_singers = 0;
        let vowel = self.vowel;
        let nyq = self.sr * 0.5;

        'notes: for &f0 in notes {
            for _ in 0..self.voices_per_note {
                if self.n_singers >= MAX_SINGERS {
                    break 'notes;
                }
                // One singer: vibrato, breath, detune, pan.
                let cents = self.detune_cents * self.rng.next_f32(); // [-c, c)
                let detune = libm::powf(2.0, cents / 1200.0);
                let fund = f0 * detune;
                let vib_rate = 4.5 + 1.25 * (self.rng.next_f32() + 1.0); // 4.5–7.0 Hz, well spread
                let breath_rate = 0.1 + 0.175 * (self.rng.next_f32() + 1.0); // 0.1–0.45 Hz
                let pan = 0.5 + 0.35 * self.rng.next_f32(); // 0.15–0.85
                let singer = self.n_singers as u8;
                self.singers[self.n_singers] = Singer {
                    vib_phase: 0.5 * (self.rng.next_f32() + 1.0),
                    vib_inc: vib_rate / self.sr,
                    // Gentle — heavy vibrato + reverb = a Leslie/rotary swirl.
                    vib_depth: 0.009,
                    breath_phase: 0.5 * (self.rng.next_f32() + 1.0),
                    breath_inc: breath_rate / self.sr,
                    breath_depth: 0.08,
                };
                self.n_singers += 1;

                let (pan_l, pan_r) = (
                    libm::cosf(pan * core::f32::consts::FRAC_PI_2),
                    libm::sinf(pan * core::f32::consts::FRAC_PI_2),
                );

                // Harmonics, formant-shaped over a steep glottal-source roll-off
                // (~−9 dB/oct, like a real voice — a bright source reads as an
                // organ). Skip harmonics the vowel suppresses.
                let mut h = 1usize;
                while (h as f32) * fund < nyq.min(9500.0) {
                    let fh = h as f32 * fund;
                    let amp = vowel.gain(fh) / libm::powf(h as f32, 1.5);
                    if amp < 0.02 && h > 1 {
                        h += 1;
                        continue;
                    }
                    if self.n_partials >= MAX_PARTIALS {
                        break 'notes;
                    }
                    self.partials[self.n_partials] = Partial {
                        phase: 0.5 * (self.rng.next_f32() + 1.0),
                        base_inc: fh / self.sr,
                        inc: fh / self.sr,
                        amp,
                        cur_amp: amp,
                        pan_l,
                        pan_r,
                        singer,
                    };
                    self.n_partials += 1;
                    h += 1;
                }
            }
        }
    }

    /// Render a stereo block. Vibrato/breath advance once per call (control
    /// rate); keep blocks ≤ a few ms so that rate is well above audio-band.
    pub fn process(&mut self, out_l: &mut [f32], out_r: &mut [f32]) {
        let n = out_l.len();
        debug_assert_eq!(out_r.len(), n);

        // Control tick: advance each singer's vibrato + breath and fold them
        // into every partial's per-sample increment / amplitude.
        let mut vibf = [1.0f32; MAX_SINGERS];
        let mut brf = [1.0f32; MAX_SINGERS];
        for s in 0..self.n_singers {
            let sg = &mut self.singers[s];
            sg.vib_phase += sg.vib_inc * n as f32;
            if sg.vib_phase >= 1.0 {
                sg.vib_phase -= 1.0;
            }
            sg.breath_phase += sg.breath_inc * n as f32;
            if sg.breath_phase >= 1.0 {
                sg.breath_phase -= 1.0;
            }
            vibf[s] = 1.0 + sg.vib_depth * lut_at(&self.lut, sg.vib_phase);
            brf[s] = 1.0 + sg.breath_depth * lut_at(&self.lut, sg.breath_phase);
        }
        for p in &mut self.partials[..self.n_partials] {
            p.inc = p.base_inc * vibf[p.singer as usize];
            p.cur_amp = p.amp * brf[p.singer as usize];
        }

        for i in 0..n {
            let (mut l, mut r) = (0.0f32, 0.0f32);
            for p in &mut self.partials[..self.n_partials] {
                let s = lut_at(&self.lut, p.phase) * p.cur_amp;
                l += s * p.pan_l;
                r += s * p.pan_r;
                p.phase += p.inc;
                if p.phase >= 1.0 {
                    p.phase -= 1.0;
                }
            }
            // Breath: formant-band noise scaled by a follower of the choir's own
            // level, decorrelated L/R → "air" while singing, silent when not.
            self.amp_env += 0.0006 * ((l.abs() + r.abs()) - self.amp_env);
            let air = self.breath * self.amp_env;
            let nl = self
                .air_lp_l
                .tick(self.air_hp_l.tick(self.breath_rng.next_f32()));
            let nr = self
                .air_lp_r
                .tick(self.air_hp_r.tick(self.breath_rng.next_f32()));
            out_l[i] = (l + air * nl) * self.level;
            out_r[i] = (r + air * nr) * self.level;
        }
    }

    /// Silence the choir (drop all partials).
    pub fn clear(&mut self) {
        self.n_partials = 0;
        self.n_singers = 0;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::boxed::Box;
    use std::vec;

    const SR: f32 = 48_000.0;

    fn run(c: &mut ChoirVoice, samples: usize) -> (std::vec::Vec<f32>, std::vec::Vec<f32>) {
        let (mut l, mut r) = (vec![0.0f32; samples], vec![0.0f32; samples]);
        let mut i = 0;
        while i < samples {
            let n = 64.min(samples - i);
            c.process(&mut l[i..i + n], &mut r[i..i + n]);
            i += n;
        }
        (l, r)
    }

    #[test]
    fn sings_a_chord_finite_and_bounded() {
        let mut c = Box::new(ChoirVoice::new(SR, 1));
        c.set_chord(&[164.81, 207.65, 246.94]); // E major triad
        let (l, r) = run(&mut c, 24_000);
        assert!(l.iter().chain(&r).all(|x| x.is_finite()));
        assert!(
            l.iter().chain(&r).all(|x| x.abs() < 4.0),
            "choir clipped/blew up"
        );
        let energy: f32 = l[8_000..].iter().map(|x| x * x).sum();
        assert!(energy > 1.0, "choir produced no tone (energy {energy})");
    }

    #[test]
    fn deterministic_for_a_seed() {
        let mut a = Box::new(ChoirVoice::new(SR, 42));
        let mut b = Box::new(ChoirVoice::new(SR, 42));
        a.set_chord(&[220.0, 277.18, 329.63]);
        b.set_chord(&[220.0, 277.18, 329.63]);
        let (la, _) = run(&mut a, 8_000);
        let (lb, _) = run(&mut b, 8_000);
        assert_eq!(la, lb, "same seed must reproduce");
    }

    #[test]
    fn silent_until_voiced() {
        let mut c = Box::new(ChoirVoice::new(SR, 3));
        let (l, r) = run(&mut c, 1_000);
        assert!(l.iter().chain(&r).all(|&x| x == 0.0));
    }

    #[test]
    fn formants_shape_the_spectrum() {
        // A partial near the first "ah" formant (~700 Hz) should be louder than
        // one in the spectral valley (~1800 Hz) for the same harmonic order.
        let c = ChoirVoice::new(SR, 0);
        assert!(c.vowel_gain(700.0) > c.vowel_gain(1800.0));
    }

    impl ChoirVoice {
        fn vowel_gain(&self, f: f32) -> f32 {
            self.vowel.gain(f)
        }
    }
}
