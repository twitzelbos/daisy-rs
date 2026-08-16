#![no_std]

//! Pure-Rust DSP for the Electrosmith Daisy (STM32H750) — `no_std`, no alloc,
//! `f32`. A batteries-included toolkit of primitives and effects that all follow
//! the same *construct-then-run* shape.
//!
//! # Using it
//!
//! Every processor is **built from a sample rate plus typed parameters**, then
//! run one of two ways, by a consistent convention:
//!
//! - **Per-sample** processors expose `tick`: `fn tick(&mut self, x: f32) -> f32`
//!   (a few return a small struct — e.g. [`filter::Svf`] yields all four
//!   responses). Oscillators and LFOs take no input: `tick(&mut self) -> f32`.
//! - **Block** processors expose `process`-style methods: mono
//!   `process(&[f32], &mut [f32])`, an explicit mono→stereo
//!   `process(input, out_l, out_r)`, or the fixed-size `process_hop` /
//!   `process_block` of the spectral and convolution engines.
//!
//! Nothing allocates. Storage that must outlive a call — delay lines, FFT
//! scratch, impulse responses — is **borrowed from the caller** (`&mut [f32]`),
//! so you choose whether it lives in DTCM, AXI SRAM, or SDRAM (see
//! `docs/memory-placement.md`). A few large spectral processors own their buffers
//! inline instead; each one says so.
//!
//! ```
//! use daisy_dsp::filter::Biquad;
//! use daisy_dsp::waveshaper::{Shape, Waveshaper};
//!
//! // Build once from the sample rate…
//! let mut lp = Biquad::lowpass(48_000.0, 1_200.0, 0.707);
//! let mut drive = Waveshaper::new(Shape::Tanh, 4.0, /* antialias */ true);
//!
//! // …then chain, per sample:
//! let dry = 0.5_f32;
//! let wet = lp.tick(drive.tick(dry));
//! # assert!(wet.is_finite());
//! ```
//!
//! # What's in here
//!
//! - **Filters** — [`filter::OnePole`], [`filter::Biquad`], [`filter::Svf`].
//! - **Dynamics** — [`dynamics::Compressor`], [`dynamics::Limiter`],
//!   [`dynamics::Gate`], [`dynamics::EnvelopeFollower`].
//! - **Generators** — [`oscillator::Oscillator`] (band-limited), [`lfo::Lfo`],
//!   [`noise::Prng`], [`window`].
//! - **Distortion / shaping** — [`waveshaper::Waveshaper`] (antialiased).
//! - **Modulation** — [`modulation::Chorus`], [`modulation::Flanger`],
//!   [`modulation::Phaser`].
//! - **Pitch** — [`pitch::PitchShifter`], [`pitch::Octaver`], [`yin::detect`],
//!   [`stft::SpectralPitchShifter`].
//! - **Spectral** — [`fft::RealFft`] (+ fixed-point SIMD [`fft_q15`]),
//!   [`stft::Stft`], [`stft::SpectralFreeze`].
//! - **Delay & envelopes** — [`delay::DelayLine`], [`env::Env`].
//! - **Reverb & convolution** — [`reverb::FdnReverb`], [`convolution::Convolver`],
//!   [`convolution::StereoConvolver`] (convolution reverb / HRIR).
//! - **Instruments & textures** — [`granular::Granular`], [`freeze::Freeze`],
//!   [`sampler::SamplePad`], [`choir::ChoirVoice`], [`pad::PadDrone`].
//!
//! Every item cites the published algorithm it implements and is covered by a
//! host-side test suite — the FFT doubles as the spectral test oracle.
//!
//! # Conventions in one place
//!
//! - Samples are `f32`, roughly `[-1.0, 1.0]`.
//! - Multi-channel blocks are **planar**: one `&[f32]` per channel.
//! - Constructors take the sample rate in Hz; time constants are seconds,
//!   frequencies Hz, gains linear unless a name says `_db`.
//! - Constructors `assert!` on impossible arguments (see each `# Panics`) — those
//!   are programmer errors, caught at build-up, never mid-stream.

pub mod choir;
pub mod convolution;
pub mod delay;
pub mod dynamics;
pub mod env;
pub mod fft;
pub mod fft_q15;
pub mod filter;
pub mod freeze;
pub mod granular;
pub mod lfo;
pub mod math;
pub mod modulation;
pub mod noise;
pub mod oscillator;
pub mod pad;
pub mod pitch;
pub mod reblock;
pub mod reverb;
pub mod sampler;
pub mod stft;
pub mod waveshaper;
pub mod window;
pub mod yin;

/// FFT twiddle-factor tables, generated at build time (see build.rs). Shared by
/// [`fft`] (f32 `TW_COS`/`TW_SIN`) and [`fft_q15`] (packed `TW_Q15`).
mod twiddles {
    include!(concat!(env!("OUT_DIR"), "/twiddles.rs"));
}

/// The largest block a [`DspProcessor`] must accept in one `process` call.
pub const MAX_BLOCK: usize = 64;

/// The uniform, allocation-free contract for a **stereo→stereo** block effect,
/// used by the three-tier (host / Renode / hardware) test framework to drive
/// full effects generically.
///
/// It intentionally covers only the effects with a stereo-in / stereo-out shape
/// (e.g. [`pad::PadDrone`]). It is **not** a universal processor interface:
/// per-sample primitives use `tick`, and mono→stereo effects (reverb,
/// convolution, spatial) use an inherent `process(input, out_l, out_r)` — those
/// I/O shapes don't fit here, and forcing them would be a worse abstraction than
/// their typed methods. Construction stays per-type (typed parameters, not a
/// dynamic dict); the trait covers only the real-time run/reset contract.
pub trait DspProcessor {
    /// Process one block. All four slices have the same length, `≤ MAX_BLOCK`.
    /// In-place is allowed (`out` may alias `in`); implementations must read a
    /// sample before overwriting it if so.
    fn process(&mut self, in_l: &[f32], in_r: &[f32], out_l: &mut [f32], out_r: &mut [f32]);

    /// Return to the just-constructed state (flush filter/delay memory).
    fn reset(&mut self);
}
