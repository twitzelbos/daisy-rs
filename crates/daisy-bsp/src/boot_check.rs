//! Vector-table plausibility checks used by the bootloader to decide whether
//! the QSPI application slot holds something worth jumping to.
//!
//! Pure integer arithmetic — no peripherals — so it can be exercised on the
//! host with `cargo test`.

/// STM32H750 QSPI-XIP window base and end.
pub const APP_BASE: u32 = 0x9000_0000;
pub const APP_END: u32 = 0x9080_0000;

/// Does `msp` look like a plausible initial main-stack pointer?
///
/// Cortex-M convention: the first word of the vector table is loaded into
/// MSP at reset, and it points one past the end of the region reserved for
/// the stack. So we accept any address in the H750's on-chip RAM banks or
/// one word past their upper bounds.
///
///   DTCM  128 KiB @ 0x2000_0000
///   AXI   512 KiB @ 0x2400_0000
///   SRAM  D2 domain (SRAM1..3, 288 KiB) @ 0x3000_0000
pub fn is_valid_sram(msp: u32) -> bool {
    matches!(msp,
        0x2000_0000..=0x2002_0000
        | 0x2400_0000..=0x2408_0000
        | 0x3000_0000..=0x3005_0000)
}

/// Does `reset` look like a plausible reset vector?
///
/// Two conditions: bit 0 must be set (Thumb state) and the target address —
/// with the Thumb bit cleared — must lie inside the QSPI XIP window.
pub fn is_valid_reset(reset: u32) -> bool {
    (reset & 1) == 1 && (APP_BASE..APP_END).contains(&(reset & !1))
}

/// Combined check: given the two words at the base of the app image, decide
/// whether the bootloader should hand control over.
pub fn is_plausible_image(msp: u32, reset: u32) -> bool {
    is_valid_sram(msp) && is_valid_reset(reset)
}

#[cfg(test)]
mod tests {
    use super::*;

    // MSP acceptance -----------------------------------------------------

    #[test]
    fn erased_flash_is_not_valid_msp() {
        // 0xFFFF_FFFF is what a freshly-erased NOR flash reads as.
        assert!(!is_valid_sram(0xFFFF_FFFF));
    }

    #[test]
    fn zero_is_not_valid_msp() {
        // Uninitialised or intentionally-cleared image.
        assert!(!is_valid_sram(0));
    }

    #[test]
    fn top_of_dtcm_is_valid_msp() {
        // 128 KiB of DTCM → MSP points one past the top (inclusive of the boundary).
        assert!(is_valid_sram(0x2002_0000));
    }

    #[test]
    fn middle_of_axi_is_valid_msp() {
        // Common linker choice: stack near the top of AXI SRAM.
        assert!(is_valid_sram(0x2404_0000));
    }

    #[test]
    fn just_past_axi_is_rejected() {
        // One word past the top of the 512 KiB region.
        assert!(!is_valid_sram(0x2408_0004));
    }

    #[test]
    fn peripheral_address_is_rejected() {
        // Peripheral base — obviously wrong for a stack pointer.
        assert!(!is_valid_sram(0x4000_0000));
    }

    #[test]
    fn qspi_window_is_not_ram() {
        // Reset vector may live here, but MSP never should.
        assert!(!is_valid_sram(0x9000_0000));
    }

    // Reset-vector acceptance --------------------------------------------

    #[test]
    fn erased_flash_is_not_valid_reset() {
        assert!(!is_valid_reset(0xFFFF_FFFF));
    }

    #[test]
    fn reset_without_thumb_bit_is_rejected() {
        // Even bytes → CPU would fault; ARMv7-M requires bit 0 set.
        assert!(!is_valid_reset(0x9000_0100));
    }

    #[test]
    fn reset_with_thumb_bit_in_qspi_is_accepted() {
        // Typical: reset handler a few hundred bytes into the app image.
        assert!(is_valid_reset(0x9000_0101));
    }

    #[test]
    fn reset_at_qspi_base_edge_is_accepted() {
        // First byte of the reset vector = base + 1 for Thumb.
        assert!(is_valid_reset(0x9000_0001));
    }

    #[test]
    fn reset_outside_qspi_window_is_rejected() {
        // Just past 8 MiB — outside our accepted window.
        assert!(!is_valid_reset(0x9080_0001));
    }

    #[test]
    fn reset_in_internal_flash_is_rejected() {
        // Bootloader's own range — an app image should not point here.
        assert!(!is_valid_reset(0x0800_0101));
    }

    // Combined -----------------------------------------------------------

    #[test]
    fn erased_qspi_fails_combined_check() {
        assert!(!is_plausible_image(0xFFFF_FFFF, 0xFFFF_FFFF));
    }

    #[test]
    fn well_formed_image_passes() {
        assert!(is_plausible_image(0x2404_0000, 0x9000_0101));
    }

    #[test]
    fn valid_msp_bad_reset_fails() {
        assert!(!is_plausible_image(0x2404_0000, 0xDEAD_BEEF));
    }

    #[test]
    fn valid_reset_bad_msp_fails() {
        assert!(!is_plausible_image(0xDEAD_BEEF, 0x9000_0101));
    }
}
