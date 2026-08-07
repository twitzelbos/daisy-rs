# Binaural spatializer — "put my sources in the room around me"

Place mono sources at chosen points in 3D space so that, on **headphones**, they
sound like they're physically out in the room — e.g. a drum machine a few feet
**behind-left** and a guitar amp directly **behind** you. A virtual monitoring /
virtual-room rig.

## How it works
Per source, convolve the mono signal with a left-ear and right-ear **HRIR**
(Head-Related Impulse Response — the time-domain HRTF) for the target direction:

```
mono ─┬─► ⊛ HRIR_L ─► left  ear
      └─► ⊛ HRIR_R ─► right ear
```

The HRIR encodes the interaural time/level differences **and** the pinna
spectral cues that let you tell front from back and up from down. Sum the stereo
outputs of all sources → headphones.

## Two hard truths (design around these)
1. **"Behind me" lives in the pinna notches** captured by a *measured* HRIR — a
   pan/delay can't do it. Needs a real HRTF dataset **with rear measurements**,
   and even then generic HRTFs cause *front-back confusion* for some listeners.
2. **"A few feet away" needs a room, not just an HRIR.** Dry HRTF images at your
   ears / inside your head. **Externalization** comes from distance attenuation +
   air-absorption LP + **reverb / early reflections** — this is the biggest
   single factor in it sounding "out there behind me."

## Two build paths (start with A)
- **A — HRIR + parametric room (MVP):** short HRIR convolution for *direction* +
  a synthetic early-reflection/reverb engine for *distance & room*. Lighter, and
  positions are **live-movable from the Hothouse knobs** (HRIR interpolation).
- **B — BRIR convolution (hi-fi):** convolve with a full **Binaural Room Impulse
  Response** (HRTF + room baked in) per position — best externalization,
  simplest signal path, but long IRs → **partitioned FFT convolution** and fixed
  position set.

## Routing on the Daisy/Hothouse
- **Source 1 (guitar):** codec IN (mono, live).
- **Source 2 (drum sim):** USB playback from the Mac (or the 2nd codec input).
- Spatialize → mix → **codec OUT (stereo) → headphones**. Keep this the
  **low-latency monitor path** (you're playing live — a long convolver is
  disorienting; keep the direct/early part short → favors path A).

## `daisy-dsp` primitives it needs
See [`daisy-dsp-roadmap.md`](daisy-dsp-roadmap.md) for the unified build order.

| Primitive | Use | Shared with pad/drone? |
|-----------|-----|------------------------|
| `DelayLine` (fractional) | early reflections, dry-delay match | ✅ |
| `Fir` / `Convolver` (short) | HRIR convolution (path A) | — |
| `HrirConvolver` (L/R FIR pair + interpolation) | the spatializer voice | — |
| `EarlyReflections` (multitap delay) | room / distance | (built on `DelayLine`) |
| `FdnReverb` | late reverb / externalization | ✅ (big win) |
| `OnePole` LP | air absorption vs distance | ✅ |
| `Gain`/`Pan`/`Mix` | distance gain, summing | ✅ |
| `Fft` + `PartitionedConvolver` | BRIR convolution (path B) | ✅ (FFT) |

Plus a **non-DSP data pipeline** (host-side): read a **SOFA** HRTF set (MIT
KEMAR / SADIE II / ARI — pick a permissive license), extract the wanted
directions, resample to 48 kHz, export a binary asset → loaded into **SDRAM**.

## Compute / memory budget (H750, 480 MHz M7 + FPU)
- Path A: 2 sources × 2 ears × ~256-tap FIR @ 48 kHz ≈ **~50 M MAC/s ≈ ~10 % CPU** + a few % for reverb. Comfortable.
- HRIR storage: full KEMAR ≈ 0.7 MB → trivial in the 64 MB SDRAM; a few positions is nothing.
- Path B (BRIR): 0.5–1 s IRs (24k–48k taps) → partitioned FFT convolution; still fits, the heavy path.

## Controls (Hothouse)
- **Knobs:** src1 azimuth · src1 distance · src2 azimuth · src2 distance · room/reverb · master.
- **Toggles:** elevation up/level/down · front↔back nudge · preset bank.
- **Footswitches:** bypass (A/B) · preset switch.

## Testing (rides the DSP test framework)
- **Tier A (host):** the spatializer is a `DspProcessor`; assert convolution output vs a scipy/numpy golden; ITD/ILD sanity per direction.
- **Tier C (hardware):** the real-time budget — cycles/block for N sources × taps, i.e. how many sources / how long a room fits live.

## Phases
1. Mono → one hardcoded HRIR pair ("behind-left"); prove it images behind (host + Daisy).
2. Add the room (distance + reverb) → externalization.
3. Two sources + mix.
4. Live positioning from the knobs (HRIR interpolation).
5. **Stretch:** BRIR path; **head-tracking** via a small IMU — dynamic cues resolve front/back and hugely improve externalization (the biggest quality upgrade).

## Risks / decisions to lock early
- **Front-back confusion** — good rear HRIRs + reverb mitigate; head-tracking solves.
- **Externalization** — don't skimp on the room model.
- **Individualization** — generic ≠ your ears; offer a few HRTF sets to pick the best-imaging one.
- **Monitor latency** — the live path must stay low-latency (favors A).

## Synergy
This is the natural consumer of the **pad/drone generator** (`pad-drone-generator.md`):
generate a drone, then place it "a few feet behind you." Both are `daisy-dsp`
`DspProcessor`s and share the reverb/delay/filter foundation.
