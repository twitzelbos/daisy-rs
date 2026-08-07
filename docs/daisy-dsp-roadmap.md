# `daisy-dsp` roadmap — shared primitives & build order

Two planned projects — the [pad/drone generator](pad-drone-generator.md) and the
[binaural spatializer](binaural-spatializer.md) — are both `daisy-dsp` citizens
built to the **one `DspProcessor` contract** (see the [DSP test
framework](dsp-test-framework.md)). They share most of their DNA. This is the
order to build the library so each phase unlocks the most, and so the lighter,
no-external-data project (the pad/drone) ships first and forces the shared
foundation into existence.

## The primitive matrix

| Primitive | Pad/drone | Spatializer | Notes |
|-----------|:---------:|:-----------:|-------|
| `DelayLine` (fractional, interpolating) | ✅ | ✅ | the workhorse — grains, pitch-shift, early reflections, dry-delay |
| `OnePole` (LP/HP) | ✅ | ✅ | tone, air-absorption |
| `Svf` (state-variable filter) | ✅ | ○ | movement/tone |
| `Lfo` | ✅ | ○ | evolution, position drift |
| `Env` (AR/ADSR + slow swell) | ✅ | — | swells, drone hold |
| `Prng` (seeded, deterministic) | ✅ | ○ | grain spray — **seeded so tests reproduce** |
| `Gain`/`Pan`/`Mix` utils | ✅ | ✅ | blends, distance gain, summing |
| `Window` (Hann) | ✅ | ○ | grains; also FFT |
| **`FdnReverb`** (+ diffusion) | ✅ | ✅ | **biggest shared win** — atmosphere *and* externalization |
| `PitchShifter` | ✅ | ○ | shimmer (+12); FFT variant shares `Fft` |
| `Granular` (scheduler + player) | ✅ | — | the pad texture core |
| `Freeze` (capture + crossfade loop) | ✅ | — | infinite sustain |
| `Oscillator` (BLEP/wavetable) | ✅ | — | synth-follow layer |
| `PitchDetector` (YIN) | ✅ | — | synth-follow layer |
| `Fir` / `Convolver` (short) | — | ✅ | HRIR convolution |
| `HrirConvolver` (L/R FIR pair + interp) | — | ✅ | the spatializer voice |
| `EarlyReflections` (multitap) | — | ✅ | built on `DelayLine` |
| `Fft` (real) | ○ | ○ | partitioned convolution + phase-vocoder pitch-shift |
| `PartitionedConvolver` (overlap-save) | — | ✅ | BRIR path (hi-fi) |

✅ needed · ○ optional/nice-to-have · — not used

## Build order

**Phase 0 — Foundation** *(both; small, pure, host-testable via tier A)*
`DelayLine`, `OnePole`, `Svf`, `Lfo`, `Env`, `Prng`, `Window`, `Gain/Pan/Mix`.
Nothing target-specific; each ships with goldens on day one.

**Phase 1 — Reverb** *(both — the single biggest shared asset)*
`FdnReverb` (+ diffusion), built on `DelayLine`. Externalization for the
spatializer, atmosphere for the pad. Add a `PitchShifter` here for shimmer.

**Phase 2 — Pad/drone MVP** *(project #2, phases 1–3)*
`Freeze`, `Granular` → route through the Phase-1 reverb/shimmer. **Ships a
playable ambient generator** and exercises the whole `daisy-dsp` + test-framework
+ Hothouse pipeline end-to-end, with no external data.

**Phase 3 — Pad synth layer** *(project #2, phase 4)*
`Oscillator`, `PitchDetector`. Optional; the granular core stands alone.

**Phase 4 — Spatializer (path A)** *(project #1, phases 1–4)*
`Fir`/`HrirConvolver`, `EarlyReflections`, + the **host-side HRTF/SOFA data
pipeline** → SDRAM asset. Reuses the Phase-1 reverb for the room.

**Phase 5 — Heavy convolution** *(project #1 path B, and pad FFT pitch-shift)*
`Fft` → `PartitionedConvolver` for BRIRs; the same `Fft` upgrades the
`PitchShifter` to a phase vocoder. The compute-heavy, highest-fidelity tier.

## Recommendation
Build **Phases 0–1 then the pad/drone (Phase 2) first.** It's the lighter build,
needs no HRTF data, gives instant payoff, and stands up the shared foundation
(reverb, filters, delay, LFO, PRNG, windowing) that the spatializer then reuses —
so project #1 becomes "add the convolver + the HRTF data pipeline," not "build a
DSP library from scratch."

Every primitive above is a pure `no_std` unit with a tier-A golden, a tier-B
Renode run, and a tier-C real-time-budget number — so the library grows *already
tested* across all three tiers.
