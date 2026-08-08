# References

The project's central bibliography. **Guiding principle:** whenever we implement
a published algorithm, cite it inline at the implementation site *and* add/update
an entry here (algorithm → reference → file). Covers DSP algorithms, audio-sample
sources (with licence), and hardware/reference-manual citations.

## DSP algorithms

| Algorithm | Reference | Used in |
|-----------|-----------|---------|
| Biquad (RBJ) coefficients | R. Bristow-Johnson, *Audio EQ Cookbook* | `daisy-dsp/src/filter.rs` |
| One-pole LP/HP | standard `y=(1−a)x+a·z`, `a=e^(−2π·fc/fs)` | `daisy-dsp/src/filter.rs` |
| Fractional delay (linear interp) | standard | `daisy-dsp/src/delay.rs` |
| Xorshift32 PRNG | G. Marsaglia, "Xorshift RNGs," *J. Stat. Soft.* 8(14), 2003 | `daisy-dsp/src/noise.rs` |
| Hann window / COLA overlap-add | standard (constant-overlap-add) | `daisy-dsp/src/window.rs`, granular |
| FDN reverb + Householder feedback matrix | J.-M. Jot & A. Chaigne, "Digital delay networks for designing artificial reverberators," *AES 90th*, 1991 | `daisy-dsp/src/reverb.rs` |
| Schroeder all-pass (input diffusion) | M. R. Schroeder, "Natural Sounding Artificial Reverberation," *JAES* 10(3), 1962; J. Dattorro, "Effect Design Part 1," *JAES* 45(9), 1997 | `daisy-dsp/src/reverb.rs` |
| Reverb delay-line modulation (chorused tank, anti-metallic) | J. Dattorro, "Effect Design Part 1," 1997; D. Griesinger | `daisy-dsp/src/reverb.rs` |
| Coupled-form ("magic circle") quadrature oscillator | J. W. Gordon & J. O. Smith, "A sine generator..."; Smith, *DAFX*/CCRMA notes | `daisy-dsp/src/reverb.rs` (modulation LFO), `daisy-dsp/src/choir.rs` (phase-accum osc bank) |
| RT60 via Schroeder backward integration (EDC) | M. R. Schroeder, "New Method of Measuring Reverberation Time," *JASA* 37, 1965 | `daisy-dsp-testkit/src/metrics.rs` |
| Karplus-Strong plucked string | K. Karplus & A. Strong, "Digital Synthesis of Plucked-String and Drum Timbres," *Computer Music J.* 7(2), 1983 | `tools/dsp-golden` (test-material generator) |
| Granular overlap-add + gain normalisation, spray/jitter | C. Roads, *Microsound* (2001); B. Truax, granular scheduling | `daisy-dsp/src/granular.rs` |
| Source-filter / parallel-formant vocal synthesis | D. Klatt, "Software for a cascade/parallel formant synthesizer," *JASA* 67, 1980 | `daisy-dsp/src/choir.rs` |
| Sample playback (rompler): pitched fractional read + crossfade loop | standard sampler practice | `daisy-dsp/src/sampler.rs` |
| FDN decay gains from RT60 (`g = 10^(−3·D/(RT60·fs))`) | Jot; standard | `daisy-dsp/src/reverb.rs` |

### Planned / prototyped
| Algorithm | Reference | Status |
|-----------|-----------|--------|
| Spectral freeze via sinusoidal (peak) resynthesis | R. McAulay & T. Quatieri, "Speech analysis/synthesis based on a sinusoidal representation," *IEEE ASSP* 34, 1986; X. Serra & J. Smith, PARSHL | Python prototype; `SpectralFreeze` engine TODO |
| Parabolic peak interpolation (STFT bin → true frequency) | J. O. Smith & X. Serra, PARSHL, 1987 | prototype |
| Phase-vocoder freeze + identity phase-locking | J. Laroche & M. Dolson, "Improved Phase Vocoder Time-Scale Modification of Audio," *IEEE SAP* 7(3), 1999; "New Phase-Vocoder Techniques...," *JAES*, 1999 | researched; alternative to sinusoidal freeze |

## Hardware / reference-manual citations

| Topic | Reference | Used in |
|-------|-----------|---------|
| DWT (CYCCNT, gating, comparators) | *ARMv7-M Architecture Reference Manual* (DDI 0403E) §C1.8; Cortex-M7 TRM (DDI 0489) | `dsp-bench`, `renode/peripherals/STM32H7_DWT_Clocked.cs` |
| MPU (PRIVDEFENA, AP decode) | *ARMv7-M ARM* §B3.5 | `mpu-exerciser`, tlib patch |
| Fault/exception vectors | *ARMv7-M ARM*; RM0433 | `fault-exerciser` |
| RCC / clock tree, PWR/VOS, FMC/SDRAM, OTG (DWC_OTG §59) | ST **RM0433** (STM32H750) | RCC/DWT/PWR/SDRAM/OTG models + drivers |

## Audio-sample / material sources

| Source | Licence | Notes |
|--------|---------|-------|
| **AKWF — Adventure Kid Waveforms** ([github](https://github.com/KristofferKarlAxelEkstrand/AKWF-FREE)) | **CC0 / public domain** | Recorded single-cycle waveforms (`AKWF_hvoice` = human voice). Verified downloadable. *Single-cycle → wavetable-like, not sustained.* Used for `pad-audition` dev material. |
| Freesound.org | CC0 / CC-BY (per file) | Reachable, but downloads need an API key / login. Good for user-sourced CC0 choir pads. |
| VSCO 2 CE / Versilian Community Sample Library | **CC0** | Candidate (unverified): sustained orchestral + some choir — best lead for a real sustained CC0 choir. |
| Wikimedia Commons | public domain (per file) | Candidate (unverified); often `.ogg` (needs decode). |
| Aerospace Atmosphere soundpacks | user-owned | The emulation target's own 16-bit WAVs — best-quality, user-supplied. |
| ⚠️ Philharmonia / Sonatina / most "free" orchestral libs | usually **not** CC0 (attribution / non-commercial) | Do not embed without a per-file licence check. |
