# Pad / drone generator — "ambient bed from my input"

Feed it your guitar (or any mono input) and it blooms into sustained, evolving
pads and drones you can play over — the ambient-atmosphere class (à la Aerospace
Atmosphere / Hologram Microcosm / Walrus Slö). The signature is *infinite sustain
+ slow evolution + lush ambience*: one chord becomes a soundscape.

## The core question: where does the pad's sound come from?
Three approaches; build the **granular core first** — it's the most robust and
the most "atmosphere":

- **A — Granular / freeze (recommended core):** capture the input into a buffer,
  then play a *cloud of overlapping windowed grains* from it (variable
  position/rate/pitch/pan). Gives **infinite sustain**, evolving texture,
  **works on chords**, and **keeps your guitar's timbre**. No pitch tracking.
- **B — Synth-follow (optional layer):** pitch-track the input → detuned
  oscillators → a clean synth pad / sub drone that *follows your notes*. Polished,
  but pitch tracking is fragile (latency, chords break it).
- **C — Shimmer / ambient reverb:** a big modulated reverb with **octave-up
  pitch-shift in the feedback** — the classic shimmer wash. Cheap, hugely
  effective, glues A/B together.

**MVP = A + C** (granular freeze into a shimmer reverb). B is a later layer.

## Signal architecture
```
 input ─┬─► [granular / freeze] ──────┐
        ├─► [pitch → synth voices]* ───┤─► mix ─► [SVF + LFO movement] ─► [shimmer reverb] ─► out
        └─► dry ──────────────────────┘
                                       (* optional layer B)
```
Plus a **slow amp envelope** (swell-in / long release, or "drone hold") and
**LFOs** on filter cutoff, detune, grain position, and pan for constant motion.

## `daisy-dsp` primitives it needs
See [`daisy-dsp-roadmap.md`](daisy-dsp-roadmap.md) for the unified build order.

| Primitive | Use | Shared with spatializer? |
|-----------|-----|--------------------------|
| `DelayLine` (fractional) | grains, pitch-shift, capture | ✅ |
| `Prng` (seeded) | grain spray — *deterministic* for tests | (pad-specific but reusable) |
| `Window` (Hann) | grain windows | ✅ (also FFT) |
| `Granular` (scheduler + player) | the texture core | — |
| `Freeze` (capture + crossfade loop) | infinite sustain | — |
| `PitchShifter` | shimmer (+12), grain pitch | (delay-based; FFT variant shares `Fft`) |
| `FdnReverb` | the ambience | ✅ (big win) |
| `Svf` filter | tone / movement | ✅ |
| `Lfo`, `Env` | evolution, swells | ✅ |
| `Oscillator` (BLEP/wavetable), `PitchDetector` (YIN) | layer B synth | — |
| `Gain`/`Pan`/`Mix` | blend, width | ✅ |

## Compute / memory budget (H750)
- Granular (a dozen grains) + `Svf` + FDN shimmer reverb ≈ **well under real-time** on the 480 MHz M7 + FPU; the pitch tracker adds a little. Lighter than the spatializer.
- Capture buffer lives in **SDRAM** (seconds of audio, easily).

## How it runs
- **Standalone pedal:** guitar in → ambient out, a normal stompbox — no computer.
- **Or a USB plugin** via the fx-loop soundcard (generate atmospheres on a Mac track).

## Controls (Hothouse)
- **Knobs:** blend (dry↔pad) · pad level · reverb size/mix · tone (filter) · texture (grain density/spray or detune) · movement (LFO rate/depth).
- **Toggles:** voicing (unison / +oct / +5th drone) · shimmer (off / +oct / −oct) · mode (pad ↔ freeze ↔ shimmer-only).
- **Footswitches:** **Freeze/hold** (capture & sustain the current sound) · bypass. LEDs show freeze/engaged.

## Testing (rides the DSP test framework)
- **Tier A (host):** deterministic blocks as `DspProcessor`s with goldens —
  oscillators, `Svf`, reverb (RT60), YIN on synthetic tones, and **granular with a
  fixed PRNG seed** (determinism is what makes granular testable *and* makes two
  units sound identical).
- **Tier C (hardware):** the real-time budget with all layers + reverb — how many
  grains / how big a reverb fits live.

## Phases
1. **Freeze + reverb** — capture-and-sustain into a big reverb. Instant payoff, no pitch tracking.
2. **Granular texture** — overlapping grains, spray, pitch, movement LFOs.
3. **Shimmer** — octave-up feedback in the reverb.
4. **Synth-follow layer** (pitch → oscillators) + voicing/drone intervals.
5. **Polish** — presets, swell envelopes, stereo width, control feel.

## Risks / decisions
- **Pitch tracking** (layer B) is the fragile bit — keep it optional; the granular core needs none.
- **Determinism** — the grain "randomness" must be a *seeded* PRNG so tier-A goldens reproduce and units match.
- **Reverb quality** — make-or-break for "atmosphere"; invest in a good FDN + diffusion.

## Why build this one first
It's lighter, needs **no external data** (no HRTF pipeline), gives instant
payoff, and it forces the shared `daisy-dsp` foundation (reverb, filters, delay,
LFO, PRNG, windowing) into existence — which the **binaural spatializer** then
reuses. See the roadmap.
