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

## Web-sourced solutions & debugging findings

Solutions or diagnoses found online that informed a decision in this repo are
logged here with their full sources, so the reasoning is traceable and the links
don't rot in a commit message. (Standing principle — cite web sources thoroughly,
same as published algorithms.)

### USB OTG interrupt does not wake the CPU from `wfi` (→ poll, don't sleep)

**Symptom (this repo, HW-verified on STM32H750):** with the composite
`daisy-usb-audio` app servicing USB purely from the `OTG_FS` interrupt and the
main loop idling in `wfi`, the OTG interrupt fired only ~2× and enumeration
stalled at `Default`. Every RM-required condition for the interrupt to fire +
wake the M7 was satisfied (verified over SWD): `GINTMSK` sources, `GAHBCFG.GINT`,
NVIC IRQ 101, `AHB1ENR`/`AHB1LPENR.USB2OTG(LP)EN`, `AHB3LPENR.QSPILPEN` (XIP code),
`PCGCCTL` ungated, HSI48 kernel clock, `SCB.SCR.SLEEPDEEP=0`. Polling `usb_dev`
from the main loop fixes it (ISR then fires 100s of ×, reaches `Configured`).

**Finding:** "USB OTG core + `wfi` → CPU doesn't wake for USB events" is a
well-known cross-family STM32 behavior, and the accepted fix is **don't `wfi`
while running USB OTG** (i.e. poll / keep the CPU awake). `wfi` is a power
optimization, not an RM requirement — nothing in RM0433 mandates sleeping to run
USB. Moot for an audio device (the sample loop never idles).

| Source | What it says |
|--------|--------------|
| TinyUSB discussion [#2295](https://github.com/hathach/tinyusb/discussions/2295) — "STM32F4 WFI Instruction + HS core == Failing TinyUSB" | WFI combined with the USB HS core leaves the CPU not waking for USB events → enumeration/latency failures; documented fix is to **disable WFI** in the idle task (traced to the `libusb_stm32` demos). Same class of issue we hit on the H7. |
| mbed-os PR [#13780](https://github.com/ARMmbed/mbed-os/pull/13780) | H7 targets need special USB-in-sleep clock handling (ULPI sleep-clock) to keep `USBDevice` working across sleep — confirms H7 USB+sleep is a known trouble spot. Our ULPI sleep-clock bits are already 0, so that lever is already pulled and did not help our (internal-FS-PHY) case. |
| Cliffle, ["An STM32 WFI bug"](https://cliffle.com/blog/stm32-wfi-bug/) | A *different* `wfi` failure — debug-clock keeps the prefetch pipeline advancing during sleep on L4/G4, corrupting instructions on wake; fixed with an `ISB` after `wfi`. Not our symptom (ours is no-wake, not crash-on-wake), but corroborates that `wfi` on these cores has multiple silicon gotchas. |
| ST Community — [USBX CDC-ACM + Sleep: waking on USB activity](https://community.st.com/stm32-mcus-products-25/usbx-cdc-acm-sleep-mode-how-to-wake-stm32u5-on-usb-activity-152815) | Reinforces that USB-activity wake-from-sleep needs deliberate handling on STM32 and isn't automatic. |

**Used in:** `crates/daisy-usb-audio/src/main.rs` — main loop polls `usb_dev`
instead of `wfi` (the `OTG_FS` ISR is kept for low-latency servicing while awake).
