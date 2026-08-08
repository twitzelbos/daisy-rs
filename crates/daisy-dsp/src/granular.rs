//! Granular texture core — the pad/drone generator's "cloud from my input".
//!
//! Continuously records the input into a borrowed ring (place it in SDRAM), then
//! plays a **cloud of overlapping windowed grains** drawn from recent history at
//! sprayed positions and pans. That gives infinite, evolving sustain that keeps
//! the input's timbre and works on chords — no pitch tracking. [`freeze`] holds
//! the buffer so the cloud grazes a *fixed* moment forever.
//!
//! # Economical by construction
//! - A **power-of-two** ring → index wrap is a mask, never a `%`.
//! - Grain windows come from **one shared** [`HannWindow`] LUT (a lookup + lerp),
//!   not a `cos` per grain-sample.
//! - Randomness (position spray, pan) is a **seeded** [`Prng`] evaluated only at
//!   grain *spawn* (a few times per ms), so the per-sample cost is just the
//!   active grains' `read + window + pan-sum`. Seeding makes the texture
//!   **deterministic** — tier-A goldens reproduce and two units sound identical.
//! - A fixed voice pool ([`MAX_GRAINS`]); density is tuned to stay under it.
//!
//! Mono in, decorrelated **stereo** out (per-grain equal-power pan).

use crate::noise::Prng;
use crate::window::HannWindow;

/// Maximum simultaneous grains (voice pool).
pub const MAX_GRAINS: usize = 12;
const WIN_N: usize = 1024;

#[derive(Clone, Copy)]
struct Grain {
    active: bool,
    pos: f32,      // fractional read index into the ring, [0, cap)
    rate: f32,     // playback rate (pitch)
    prog: f32,     // window progress, [0, 1)
    prog_inc: f32, // 1 / grain_len
    gain_l: f32,
    gain_r: f32,
}

impl Grain {
    const SILENT: Self = Self {
        active: false,
        pos: 0.0,
        rate: 1.0,
        prog: 0.0,
        prog_inc: 0.0,
        gain_l: 0.0,
        gain_r: 0.0,
    };
}

/// A granular cloud over a caller-provided **power-of-two** ring buffer.
pub struct Granular<'a> {
    buf: &'a mut [f32],
    mask: usize, // cap - 1
    capf: f32,   // cap as f32
    write_pos: usize,
    frozen: bool,
    rng: Prng,
    win: HannWindow<WIN_N>,
    grains: [Grain; MAX_GRAINS],
    spawn_interval: usize, // mean samples between grain spawns (density)
    spawn_timer: usize,
    next_spawn: usize,  // jittered target for the current interval
    grain_len: f32,     // samples
    backoff_min: usize, // nearest a grain reads behind the write head
    spray: usize,       // random extra backoff range
    pitch: f32,         // base playback rate
    detune: f32,        // ± random per-grain pitch spread (chorus)
    pan_spread: f32,    // 0 = centre, 1 = full L↔R
}

impl<'a> Granular<'a> {
    /// Build over `buf`, whose length **must be a power of two** (≥ 4096). The
    /// defaults give a lush ~80 ms-grain cloud at ~6× overlap; tune with the
    /// setters.
    pub fn new(buf: &'a mut [f32], sample_rate: f32, seed: u32) -> Self {
        let cap = buf.len();
        assert!(
            cap >= 4096 && cap.is_power_of_two(),
            "Granular buffer must be a power of two ≥ 4096"
        );
        buf.fill(0.0);
        let grain_len = 0.08 * sample_rate; // 80 ms
        let spawn_interval = (grain_len as usize / 6).max(1); // ~6× overlap
        Self {
            mask: cap - 1,
            capf: cap as f32,
            buf,
            write_pos: 0,
            frozen: false,
            rng: Prng::new(seed),
            win: HannWindow::new(),
            grains: [Grain::SILENT; MAX_GRAINS],
            spawn_interval,
            spawn_timer: 0,
            next_spawn: spawn_interval,
            grain_len,
            backoff_min: grain_len as usize,
            spray: cap / 2,
            pitch: 1.0,
            detune: 0.0,
            pan_spread: 1.0,
        }
    }

    /// Grain length in seconds (texture grain size).
    pub fn set_grain_len(&mut self, sample_rate: f32, seconds: f32) {
        self.grain_len = (seconds * sample_rate).max(1.0);
        self.backoff_min = self.grain_len as usize;
    }

    /// Grains per second (density). Higher = thicker cloud (capped by the pool).
    pub fn set_density(&mut self, sample_rate: f32, grains_per_s: f32) {
        self.spawn_interval = ((sample_rate / grains_per_s.max(1.0)) as usize).max(1);
        self.next_spawn = self.spawn_interval;
    }

    /// Base playback rate (`1.0` = original pitch/timbre; `2.0` = octave up).
    pub fn set_pitch(&mut self, ratio: f32) {
        self.pitch = ratio;
    }

    /// Per-grain random pitch spread (chorus), e.g. `0.004` ≈ ±7 cents. Turns the
    /// phase-beating of identical-pitch grains into a smooth chorus.
    pub fn set_detune(&mut self, amount: f32) {
        self.detune = amount.max(0.0);
    }

    /// Stereo spread of the grain spray, `0.0` (mono) … `1.0` (full width).
    pub fn set_pan_spread(&mut self, spread: f32) {
        self.pan_spread = spread.clamp(0.0, 1.0);
    }

    /// Hold the buffer: the cloud keeps grazing a fixed moment (infinite sustain
    /// of the current sound) and recording stops.
    pub fn set_frozen(&mut self, frozen: bool) {
        self.frozen = frozen;
    }

    /// Whether the buffer is held.
    pub fn is_frozen(&self) -> bool {
        self.frozen
    }

    #[inline(always)]
    fn read_frac(&self, pos: f32) -> f32 {
        let i = pos as usize;
        let frac = pos - i as f32;
        let a = self.buf[i & self.mask];
        let b = self.buf[(i + 1) & self.mask];
        a + (b - a) * frac
    }

    /// Spawn a grain into a free slot (silently drops if the pool is full).
    fn spawn(&mut self) {
        let slot = match self.grains.iter().position(|g| !g.active) {
            Some(s) => s,
            None => return,
        };
        // Read position: `backoff` samples behind the write head, sprayed.
        let backoff = self.backoff_min + (self.rng.next_u32() as usize % self.spray.max(1));
        let start = (self.write_pos + self.buf.len() - (backoff & self.mask)) & self.mask;
        // Equal-power pan around centre, sprayed by pan_spread.
        let pan = 0.5 + 0.5 * self.pan_spread * self.rng.next_f32(); // rng is [-1,1)
        let (angle_s, angle_c) = (
            libm::sinf(pan * core::f32::consts::FRAC_PI_2),
            libm::cosf(pan * core::f32::consts::FRAC_PI_2),
        );
        self.grains[slot] = Grain {
            active: true,
            pos: start as f32,
            rate: self.pitch * (1.0 + self.detune * self.rng.next_f32()),
            prog: 0.0,
            prog_inc: 1.0 / self.grain_len,
            gain_l: angle_c,
            gain_r: angle_s,
        };
    }

    /// Process a mono block into a decorrelated stereo pair (all three slices
    /// share length).
    pub fn process(&mut self, input: &[f32], out_l: &mut [f32], out_r: &mut [f32]) {
        for ((&x, ol), or) in input.iter().zip(out_l.iter_mut()).zip(out_r.iter_mut()) {
            if !self.frozen {
                self.buf[self.write_pos] = x;
                self.write_pos = (self.write_pos + 1) & self.mask;
            }

            self.spawn_timer += 1;
            if self.spawn_timer >= self.next_spawn {
                self.spawn_timer = 0;
                self.spawn();
                // Jitter the next spawn time (±50%) so the grain rate has no
                // fixed period — a periodic scheduler buzzes audibly.
                let j = self.spawn_interval as f32 * 0.5 * self.rng.next_f32();
                self.next_spawn = (self.spawn_interval as f32 + j).max(1.0) as usize;
            }

            let (mut l, mut r, mut wsum) = (0.0f32, 0.0f32, 0.0f32);
            for gi in 0..MAX_GRAINS {
                let mut g = self.grains[gi];
                if !g.active {
                    continue;
                }
                let w = self.win.at(g.prog);
                let s = self.read_frac(g.pos) * w;
                l += s * g.gain_l;
                r += s * g.gain_r;
                wsum += w;
                g.pos += g.rate;
                if g.pos >= self.capf {
                    g.pos -= self.capf;
                }
                g.prog += g.prog_inc;
                if g.prog >= 1.0 {
                    g.active = false;
                }
                self.grains[gi] = g;
            }

            // Overlap-add gain normalisation: divide by the running window sum so
            // the output is a weighted AVERAGE of the grains, not their sum. This
            // makes the level constant no matter how the overlap count varies
            // (jittered spawns, grains dying) — without it the amplitude flutters.
            let norm = 1.0 / wsum.max(0.1);
            *ol = l * norm;
            *or = r * norm;
        }
    }

    /// Flush the buffer, grains, and scheduler.
    pub fn reset(&mut self) {
        self.buf.fill(0.0);
        self.write_pos = 0;
        self.frozen = false;
        self.spawn_timer = 0;
        self.next_spawn = self.spawn_interval;
        self.grains = [Grain::SILENT; MAX_GRAINS];
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec;
    use std::vec::Vec;

    const SR: f32 = 48_000.0;

    // A deterministic, guitar-like source: Karplus-Strong plucked string. Far
    // more representative than a sine for a texture engine, and reproducible
    // (seeded) so the granular determinism test is meaningful.
    fn karplus_strong(freq: f32, n: usize, seed: u32) -> Vec<f32> {
        let p = (SR / freq) as usize;
        let mut ring = vec![0.0f32; p];
        let mut rng = Prng::new(seed);
        for r in ring.iter_mut() {
            *r = rng.next_f32(); // noise burst excites the string
        }
        let mut out = vec![0.0f32; n];
        let mut idx = 0;
        let mut prev = 0.0f32;
        for o in out.iter_mut() {
            let cur = ring[idx];
            let filtered = 0.5 * (cur + prev); // one-zero lowpass = string decay
            *o = filtered;
            ring[idx] = filtered;
            prev = filtered;
            idx = (idx + 1) % p;
        }
        out
    }

    fn run(g: &mut Granular, input: &[f32]) -> (Vec<f32>, Vec<f32>) {
        let mut l = vec![0.0f32; input.len()];
        let mut r = vec![0.0f32; input.len()];
        // Block-by-block, like the real-time contract.
        let mut i = 0;
        while i < input.len() {
            let n = 64.min(input.len() - i);
            g.process(&input[i..i + n], &mut l[i..i + n], &mut r[i..i + n]);
            i += n;
        }
        (l, r)
    }

    #[test]
    fn deterministic_for_a_seed() {
        let src = karplus_strong(196.0, 48_000, 1); // low G, 1 s
        let mut b1 = vec![0.0f32; 1 << 15];
        let mut b2 = vec![0.0f32; 1 << 15];
        let mut g1 = Granular::new(&mut b1, SR, 0xC0FFEE);
        let mut g2 = Granular::new(&mut b2, SR, 0xC0FFEE);
        let (l1, r1) = run(&mut g1, &src);
        let (l2, r2) = run(&mut g2, &src);
        assert_eq!(l1, l2, "left channel not reproducible for a fixed seed");
        assert_eq!(r1, r2, "right channel not reproducible for a fixed seed");
    }

    #[test]
    fn finite_bounded_and_textured() {
        let src = karplus_strong(196.0, 48_000, 7);
        let mut buf = vec![0.0f32; 1 << 15];
        let mut g = Granular::new(&mut buf, SR, 42);
        let (l, r) = run(&mut g, &src);
        assert!(l.iter().chain(&r).all(|x| x.is_finite()));
        assert!(
            l.iter().chain(&r).all(|x| x.abs() < 8.0),
            "grain cloud blew up"
        );
        // A dense overlapping cloud produces continuous energy, not a few spikes.
        let energy: f32 = l[SR as usize / 2..].iter().map(|x| x * x).sum();
        assert!(
            energy > 1e-2,
            "granular produced no texture (energy {energy})"
        );
    }

    #[test]
    fn freeze_sustains_after_input_stops() {
        let src = karplus_strong(147.0, 24_000, 3); // 0.5 s of a low D
        let mut buf = vec![0.0f32; 1 << 15];
        let mut g = Granular::new(&mut buf, SR, 99);
        run(&mut g, &src);
        g.set_frozen(true);
        assert!(g.is_frozen());
        let silence = vec![0.0f32; 48_000];
        let (l, r) = run(&mut g, &silence); // no input now
        assert!(l.iter().chain(&r).all(|x| x.is_finite()));
        // The cloud keeps grazing the held buffer → sustained energy.
        let tail: f32 = l[24_000..].iter().map(|x| x * x).sum();
        assert!(
            tail > 1e-2,
            "frozen granular did not sustain (energy {tail})"
        );
    }

    #[test]
    fn silence_in_silence_out() {
        let mut buf = vec![0.0f32; 1 << 15];
        let mut g = Granular::new(&mut buf, SR, 5);
        let (l, r) = run(&mut g, &vec![0.0f32; 8192]);
        assert!(l.iter().chain(&r).all(|&x| x == 0.0));
    }
}
