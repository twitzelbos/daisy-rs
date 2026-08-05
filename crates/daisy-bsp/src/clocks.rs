//! Clock tree init for the Daisy Seed.
//!
//! The Seed has a 16 MHz HSE crystal. We spin PLL1 up to 480 MHz sysclk (the
//! H750's ceiling in VOS0) and let the HAL pick PLL1_Q freely. USB and SAI
//! clocking will be configured by their respective subsystems on PLL3.

use crate::hal::pac;
use crate::hal::prelude::*;
use crate::hal::rcc::Ccdr;

/// Bring the chip up to 480 MHz sysclk.
///
/// Consumes `PWR` and `RCC` (they're single-shot inits) but only borrows
/// `SYSCFG` since callers still need it for other subsystems.
pub fn init(pwr: pac::PWR, rcc: pac::RCC, syscfg: &pac::SYSCFG) -> Ccdr {
    // NB: we ask for VOS1 (400 MHz max) rather than VOS0 (480 MHz).
    // With `lto = "fat"` the compiler proves an assertion inside the HAL's
    // VOS0 freeze path is unreachable-at-callsite and eliminates the rest
    // of main. VOS1 hits 400 MHz cleanly and leaves the whole binary intact.
    // Reserved TODO: track the 480 MHz LTO issue upstream and re-enable.
    let pwrcfg = pwr.constrain().freeze();
    rcc.constrain()
        .use_hse(16.MHz())
        .sys_ck(400.MHz())
        .freeze(pwrcfg, syscfg)
}
