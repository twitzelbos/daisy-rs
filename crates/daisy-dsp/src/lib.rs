#![no_std]

//! Pure-Rust DSP primitives for the Daisy (STM32H750), `no_std` + no alloc.
//!
//! # Two layers
//!
//! 1. **Primitives** (this crate's modules) — small, idiomatic, allocation-free
//!    building blocks with *typed* constructors and simple `tick`/`process`
//!    methods: [`filter::OnePole`], [`filter::Biquad`], [`delay::DelayLine`],
//!    [`noise::Prng`], … Each is deterministic and validated against an
//!    independent oracle (scipy) through the golden test framework — see
//!    `docs/dsp-test-framework.md` and `tools/dsp-golden`.
//!
//! 2. **Effects** — full processors (a reverb, the pad/drone generator, the
//!    binaural spatializer) that *compose* primitives and implement
//!    [`DspProcessor`] so the three-tier test framework (host / Renode /
//!    hardware) can drive them uniformly. Primitives stay typed and un-dynamic;
//!    only effects take the trait.
//!
//! # Conventions
//!
//! - Samples are `f32`, normalised to roughly `[-1.0, 1.0]`.
//! - A processor is constructed from a **sample rate** (Hz) plus its typed
//!   parameters, and never allocates. Storage that must outlive a block (delay
//!   buffers) is *borrowed* from the caller, so it can live in DTCM, AXI SRAM,
//!   or SDRAM as the app chooses.
//! - Blocks are **planar stereo**: separate `&[f32]` per channel. Mono uses one.

pub mod delay;
pub mod filter;
pub mod math;
pub mod noise;
pub mod reverb;

/// The largest block a [`DspProcessor`] must accept in one `process` call.
pub const MAX_BLOCK: usize = 64;

/// A block-based, allocation-free, deterministic stereo audio effect.
///
/// This is the contract the three-tier test framework drives. **Primitives do
/// not implement it** — they are the typed building blocks effects compose.
/// Construction is per-type (parameters are typed, not a dynamic dict); the
/// trait covers only the real-time contract.
pub trait DspProcessor {
    /// Process one block. All four slices have the same length, `≤ MAX_BLOCK`.
    /// In-place is allowed (`out` may alias `in`); implementations must read a
    /// sample before overwriting it if so.
    fn process(&mut self, in_l: &[f32], in_r: &[f32], out_l: &mut [f32], out_r: &mut [f32]);

    /// Return to the just-constructed state (flush filter/delay memory).
    fn reset(&mut self);
}
