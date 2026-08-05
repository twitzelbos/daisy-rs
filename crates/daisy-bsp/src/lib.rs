#![no_std]

//! Board support for the Electrosmith Daisy Seed and its host boards.
//!
//! Peripheral-touching modules (`clocks`, `led`, ...) are compiled only for
//! bare-metal targets. Pure-logic modules (`boot_check`) are available on the
//! host as well so unit tests can run under `cargo test --target <host>`.

pub mod boot_check;

#[cfg(target_os = "none")]
pub use stm32h7xx_hal as hal;

#[cfg(target_os = "none")]
pub mod clocks;

#[cfg(target_os = "none")]
pub mod led;
