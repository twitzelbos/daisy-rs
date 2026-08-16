#![no_std]

//! Board support for the Electrosmith Daisy Seed and its host boards.
//!
//! Peripheral-touching modules (`clocks`, `led`, ...) are compiled only for
//! bare-metal targets. Pure-logic modules (`boot_check`) are available on the
//! host as well so unit tests can run under `cargo test --target <host>`.

pub mod boot_check;

// SDRAM register-encoding logic is pure and host-testable; the bare-metal
// bring-up inside it is target-gated.
pub mod sdram;

// Hothouse pedal (a Daisy Seed host board). The control logic — debounce,
// toggle decode — is pure and host-testable; the pin binding is target-gated.
pub mod hothouse;

pub mod pod;

#[cfg(target_os = "none")]
pub use stm32h7xx_hal as hal;

#[cfg(target_os = "none")]
pub mod clocks;

#[cfg(target_os = "none")]
pub mod led;

// Software reset + field DFU entry (write a Backup-SRAM flag + soft-reset).
#[cfg(target_os = "none")]
pub mod reset;
