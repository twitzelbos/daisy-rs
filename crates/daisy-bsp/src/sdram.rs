//! FMC SDRAM support for the Daisy Seed.
//!
//! This board carries an Alliance Memory **AS4C16M32SB-6BCN**: 16M × 32-bit =
//! 64 MByte, 4 internal banks, 13 row bits, 9 column bits, CAS latency 3,
//! -6 speed grade (~166 MHz max). (libDaisy targets the pin/geometry-compatible
//! AS4C16M32MSA variant; the SB part on this unit has the same 16M × 32
//! organisation and -6 timing, so the FMC configuration is identical.) It
//! hangs off FMC SDRAM **Bank 1**, mapped at **0xC000_0000**, clocked at
//! **SDCLK = 100 MHz** (FMC kernel = PLL2R = 200 MHz, ÷2). Register values
//! match Electrosmith libDaisy (`src/dev/sdram.cpp`, `src/sys/system.cpp`).
//!
//! This module is split so the register-encoding logic is host-testable:
//!   * [`config`] — pure logic: device parameters → FMC register values.
//!     Compiles and runs under `cargo test` on the host.
//!   * `init` (bare-metal only) — the actual bring-up: PLL2 kernel clock,
//!     GPIO alt-functions, and the FMC SDRAM init command sequence.
//!
//! Refs: RM0433 §22 (FMC SDRAM controller), AS4C16M32MSA datasheet,
//! libDaisy `dev/sdram.cpp` + `sys/system.cpp`. ES0392 §2.6.6/§2.6.7 (SDRAM
//! timing errata) do not trigger with these timings (TWR < TRCD, TXSR).

#[cfg(target_os = "none")]
pub use bare::init;

/// Pure-logic SDRAM configuration: device parameters and the FMC register
/// encodings derived from them. No hardware access — unit-tested on the host.
pub mod config {
    /// SDRAM geometry / timing for one device on one FMC bank. Timing fields
    /// are in SDCLK cycles, exactly as the ST HAL's `FMC_SDRAM_TimingTypeDef`
    /// expresses them (the register field is `value - 1`).
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct SdramConfig {
        /// Column address bits (8..=11).
        pub column_bits: u8,
        /// Row address bits (11..=13).
        pub row_bits: u8,
        /// Data bus width in bits (8, 16 or 32).
        pub data_width: u8,
        /// Internal bank count (2 or 4).
        pub internal_banks: u8,
        /// CAS latency in cycles (1..=3).
        pub cas_latency: u8,
        /// SDCLK divider of the FMC kernel clock (2 or 3).
        pub sdclk_divider: u8,
        /// Read burst enable.
        pub read_burst: bool,
        /// Read-pipe delay in cycles (0..=2).
        pub read_pipe_delay: u8,

        // Timing (SDCLK cycles), as in the HAL timing struct.
        pub t_mrd: u8, // load-mode-register to active
        pub t_xsr: u8, // exit-self-refresh to active
        pub t_ras: u8, // self-refresh (min row-active) time
        pub t_rc: u8,  // row-cycle (refresh→refresh / active→active)
        pub t_wr: u8,  // write-recovery
        pub t_rp: u8,  // row-precharge
        pub t_rcd: u8, // row-to-column

        /// Auto-refresh burst count in the init sequence.
        pub auto_refresh_cycles: u8,
        /// SDRTR refresh-timer COUNT value (bits[13:1] of FMC_SDRTR).
        pub refresh_count: u16,
    }

    /// The Daisy Seed's SDRAM (AS4C16M32SB-6BCN) on FMC Bank 1, at SDCLK =
    /// 100 MHz (10 ns/cycle). Timings are the datasheet -6 minimums rounded up
    /// to whole cycles; most match libDaisy, but TWO differ where libDaisy is
    /// out of spec for this part:
    ///   * `t_ras = 5` (50 ns): datasheet tRAS(min) = 42 ns → 5 cycles.
    ///     libDaisy uses 4 (40 ns), which is 2 ns UNDER spec.
    ///   * `refresh_count = 0x2F9`: 8192 rows / 64 ms = 7.8125 µs/row = 781
    ///     cycles; COUNT = 781 − 20 = 0x2F9 (RM0433 §22 formula). libDaisy
    ///     programs 0x806 (~20 µs/row), which UNDER-refreshes ~2.6× and risks
    ///     retention loss. The datasheet-correct value refreshes often enough.
    ///
    /// The remaining values (tRCD/tRP/tRC/tWR) keep libDaisy's conservative
    /// choices, which already exceed the datasheet minimums.
    pub const SEED: SdramConfig = SdramConfig {
        column_bits: 9,
        row_bits: 13,
        data_width: 32,
        internal_banks: 4,
        cas_latency: 3,
        sdclk_divider: 2,
        read_burst: true,
        read_pipe_delay: 0,
        t_mrd: 2,  // tMRD 12 ns → 2 cycles
        t_xsr: 7,  // exit-self-refresh ≥ tRFC 60 ns → 6; 7 is safe
        t_ras: 5,  // tRAS(min) 42 ns → 5 cycles (libDaisy's 4 is under-spec)
        t_rc: 8,   // tRC(min) 60 ns → 6; libDaisy 8 (conservative)
        t_wr: 3,   // tWR 12 ns → 2; libDaisy 3 (conservative)
        t_rp: 16,  // tRP 18 ns → 2; libDaisy 16 (very conservative)
        t_rcd: 10, // tRCD 18 ns → 2; libDaisy 10 (very conservative)
        auto_refresh_cycles: 4,
        // 8192 rows / 64 ms at 100 MHz: COUNT = 781 − 20 = 0x2F9 (RM0433).
        refresh_count: 0x2F9,
    };

    /// SDRAM base address (FMC Bank 1).
    pub const BASE_ADDRESS: u32 = 0xC000_0000;
    /// Total size in bytes (16M × 32-bit = 64 MiB).
    pub const SIZE_BYTES: u32 = 0x0400_0000;

    impl SdramConfig {
        /// FMC_SDCR1 value (RM0433 §22.7.1). SDCLK/RBURST/RPIPE live only in
        /// SDCR1 (they apply to both banks); NC/NR/MWID/NB/CAS/WP are per-bank.
        pub const fn sdcr1(&self) -> u32 {
            let nc = (self.column_bits - 8) as u32; // 8→00 … 11→11
            let nr = (self.row_bits - 11) as u32; // 11→00 … 14→11
            let mwid = match self.data_width {
                8 => 0,
                16 => 1,
                _ => 2, // 32-bit
            };
            let nb = if self.internal_banks == 4 { 1 } else { 0 };
            let cas = self.cas_latency as u32; // 1→01, 2→10, 3→11
            let sdclk = self.sdclk_divider as u32; // ÷2→10, ÷3→11
            let rburst = if self.read_burst { 1 } else { 0 };
            let rpipe = self.read_pipe_delay as u32;

            (nc & 0x3)
                | ((nr & 0x3) << 2)
                | ((mwid & 0x3) << 4)
                | (nb << 6)
                | ((cas & 0x3) << 7)
                // WP (bit 9) = 0
                | ((sdclk & 0x3) << 10)
                | (rburst << 12)
                | ((rpipe & 0x3) << 13)
        }

        /// FMC_SDTR1 value (RM0433 §22.7.3). Each field is `cycles - 1`. TRC
        /// and TRP are defined only in SDTR1 (they apply to both banks).
        pub const fn sdtr1(&self) -> u32 {
            ((self.t_mrd as u32 - 1) & 0xF)
                | (((self.t_xsr as u32 - 1) & 0xF) << 4)
                | (((self.t_ras as u32 - 1) & 0xF) << 8)
                | (((self.t_rc as u32 - 1) & 0xF) << 12)
                | (((self.t_wr as u32 - 1) & 0xF) << 16)
                | (((self.t_rp as u32 - 1) & 0xF) << 20)
                | (((self.t_rcd as u32 - 1) & 0xF) << 24)
        }

        /// SDRAM mode-register value loaded via the Load-Mode-Register command
        /// (JEDEC mode register): burst length 1 (single), sequential, the
        /// device CAS latency, single-location write-burst. libDaisy uses
        /// burst length code 010b (M0='0', M1='1'), which reads back as 0x232
        /// for CAS 3.
        pub const fn mode_register(&self) -> u32 {
            const BURST_LENGTH_CODE: u32 = 0b010; // M[2:0]
            const WRITE_BURST_SINGLE: u32 = 1 << 9; // M9
            BURST_LENGTH_CODE | ((self.cas_latency as u32 & 0x7) << 4) | WRITE_BURST_SINGLE
        }

        /// FMC_SDCMR mode-register-definition field (bits[22:9] carry the mode
        /// register for a Load-Mode-Register command).
        pub const fn sdcmr_mrd(&self) -> u32 {
            self.mode_register() << 9
        }

        /// FMC_SDRTR value: COUNT occupies bits[13:1].
        pub const fn sdrtr(&self) -> u32 {
            (self.refresh_count as u32 & 0x1FFF) << 1
        }
    }

    #[cfg(test)]
    #[allow(clippy::identity_op)]
    mod tests {
        use super::*;

        // Geometry (SDCR1) and the mode register match libDaisy; the timing
        // (SDTR1) and refresh (SDRTR) are derived from the AS4C16M32SB-6
        // datasheet and differ from libDaisy where libDaisy is out of spec.
        #[test]
        fn seed_sdcr1_matches_libdaisy() {
            // 9 col / 13 row / 32-bit / 4 banks / CAS3 / SDCLK÷2 / RBURST.
            assert_eq!(SEED.sdcr1(), 0x0000_19E9);
        }

        #[test]
        fn seed_sdtr1_is_datasheet_compliant() {
            // Fields (cycles-1): TMRD1 TXSR6 TRAS4 TRC7 TWR2 TRP15 TRCD9.
            // Differs from libDaisy's 0x09F2_7361 only in TRAS (4 vs 3), the
            // fix for the 42 ns tRAS minimum.
            assert_eq!(SEED.sdtr1(), 0x09F2_7461);
        }

        #[test]
        fn seed_tras_meets_datasheet_minimum() {
            // tRAS(min) = 42 ns; at SDCLK 100 MHz (10 ns) that needs ≥ 5 cycles.
            assert!(SEED.t_ras >= 5, "TRAS under datasheet minimum (42 ns)");
        }

        #[test]
        fn seed_mode_register_matches_libdaisy() {
            assert_eq!(SEED.mode_register(), 0x232);
            assert_eq!(SEED.sdcmr_mrd(), 0x232 << 9);
        }

        #[test]
        fn seed_refresh_count_is_datasheet_compliant() {
            // COUNT = rows_refresh_interval_cycles - 20, per RM0433 §22.
            assert_eq!(SEED.refresh_count, 0x2F9);
            assert_eq!(SEED.sdrtr(), 0x0000_05F2);
        }

        #[test]
        fn seed_refresh_is_frequent_enough() {
            // The device needs all 8192 rows refreshed within 64 ms. At
            // SDCLK = 100 MHz the per-row budget is 64ms/8192 = 781.25 cycles;
            // the FMC issues a refresh every (COUNT + 20) cycles, which must
            // not exceed that budget. libDaisy's 0x806 (2054) fails this.
            const ROWS: u64 = 8192;
            const SDCLK_HZ: u64 = 100_000_000;
            const REFRESH_PERIOD_NS: u64 = 64_000_000;
            let per_row_cycles = REFRESH_PERIOD_NS * SDCLK_HZ / (ROWS * 1_000_000_000);
            let actual_interval = SEED.refresh_count as u64 + 20;
            assert!(
                actual_interval <= per_row_cycles,
                "refresh interval {actual_interval} cycles exceeds per-row budget {per_row_cycles}"
            );
        }

        #[test]
        fn seed_geometry() {
            assert_eq!(BASE_ADDRESS, 0xC000_0000);
            assert_eq!(SIZE_BYTES, 0x0400_0000); // 64 MiB
                                                 // 16M addresses × 4 bytes = 64 MiB.
            let addresses = 1u64 << (SEED.row_bits + SEED.column_bits + 2/*bank bits*/);
            assert_eq!(addresses * (SEED.data_width as u64 / 8), SIZE_BYTES as u64);
        }
    }
}

/// Bare-metal FMC SDRAM bring-up (STM32H750 only).
///
/// # HARDWARE-VALIDATION REQUIRED
/// This performs the real FMC SDRAM init: PLL2 kernel clock, ~50 GPIO
/// alt-functions, and the JEDEC power-up command sequence. The `command
/// sequence` IS exercised in sim: Renode's `STM32H7_FMC_SDRAM` model gates the
/// 64 MiB window on exactly this ordered sequence (Clock-Config-Enable →
/// Precharge-All → Auto-Refresh → Load-Mode-Register) and reports SDSR
/// not-busy so the status polls terminate — `sdram-exerciser` +
/// `renode/sdram_init.robot` run this real `init` and assert the window then
/// round-trips data. What sim CANNOT show — real DRAM cell behaviour, SDTR
/// timing, refresh, aliasing — still needs hardware (probe-rs); the
/// `daisy-sdram-test` CDC app is the on-hardware validator. The FMC register
/// *values* are unit-tested in [`super::config`]; the sequence, pin map and
/// PLL2 setup mirror Electrosmith libDaisy (`dev/sdram.cpp`, `sys/system.cpp`).
///
/// Raw-register (not PAC-typed) so it is fully self-contained and needs no
/// `ccdr`: the app runs after the bootloader has frozen the clocks, and PLL2
/// is independent of the frozen PLL1. All status polls are bounded so a
/// mis-wired controller cannot hang forever.
#[cfg(target_os = "none")]
mod bare {
    use super::config::SEED;

    // --- Peripheral base addresses (RM0433) ------------------------------
    const RCC: u32 = 0x5802_4400;
    const RCC_D1CCIPR: *mut u32 = (RCC + 0x4C) as *mut u32; // FMCSEL[1:0]
    const RCC_AHB3ENR: *mut u32 = (RCC + 0xD4) as *mut u32; // FMCEN=12
    const RCC_AHB4ENR: *mut u32 = (RCC + 0xE0) as *mut u32; // GPIOxEN

    const FMC: u32 = 0x5200_4000;
    const FMC_SDCR1: *mut u32 = (FMC + 0x140) as *mut u32;
    const FMC_SDTR1: *mut u32 = (FMC + 0x148) as *mut u32;
    const FMC_SDCMR: *mut u32 = (FMC + 0x150) as *mut u32;
    const FMC_SDRTR: *mut u32 = (FMC + 0x154) as *mut u32;
    const FMC_SDSR: *const u32 = (FMC + 0x158) as *const u32; // BUSY = bit 5

    const GPIO_BASE: u32 = 0x5802_0000;
    const GPIO_STRIDE: u32 = 0x400;

    // FMC pin masks per port, all AF12 (from libDaisy `sdram.cpp`). Ports:
    // C=2, D=3, E=4, F=5, G=6, H=7, I=8 (index into 0x5802_0000 + n*0x400).
    const PINS: &[(u32, u16)] = &[
        (2, 1 << 0), // PC0  SDNWE
        (
            3,
            (1 << 0) | (1 << 1) | (1 << 8) | (1 << 9) | (1 << 10) | (1 << 14) | (1 << 15),
        ),
        (
            4, // GPIOE
            (1 << 0)
                | (1 << 1)
                | (1 << 7)
                | (1 << 8)
                | (1 << 9)
                | (1 << 10)
                | (1 << 11)
                | (1 << 12)
                | (1 << 13)
                | (1 << 14)
                | (1 << 15),
        ),
        (
            5, // GPIOF
            (1 << 0)
                | (1 << 1)
                | (1 << 2)
                | (1 << 3)
                | (1 << 4)
                | (1 << 5)
                | (1 << 11)
                | (1 << 12)
                | (1 << 13)
                | (1 << 14)
                | (1 << 15),
        ),
        (
            6,
            (1 << 0) | (1 << 1) | (1 << 2) | (1 << 4) | (1 << 5) | (1 << 8) | (1 << 15),
        ),
        (
            7, // GPIOH
            (1 << 2)
                | (1 << 3)
                | (1 << 8)
                | (1 << 9)
                | (1 << 10)
                | (1 << 11)
                | (1 << 12)
                | (1 << 13)
                | (1 << 14)
                | (1 << 15),
        ),
        (
            8, // GPIOI
            (1 << 0)
                | (1 << 1)
                | (1 << 2)
                | (1 << 3)
                | (1 << 4)
                | (1 << 5)
                | (1 << 6)
                | (1 << 7)
                | (1 << 9)
                | (1 << 10),
        ),
    ];

    /// Set every pin in `mask` on GPIO port `port` to AF12, very-high speed,
    /// push-pull, no pull — the FMC signal configuration.
    unsafe fn configure_af12(port: u32, mask: u16) {
        let base = GPIO_BASE + port * GPIO_STRIDE;
        let moder = base as *mut u32;
        let ospeedr = (base + 0x08) as *mut u32;
        let pupdr = (base + 0x0C) as *mut u32;
        let afrl = (base + 0x20) as *mut u32;
        let afrh = (base + 0x24) as *mut u32;
        for pin in 0..16u32 {
            if mask & (1 << pin) == 0 {
                continue;
            }
            let two = pin * 2;
            core::ptr::write_volatile(
                moder,
                (core::ptr::read_volatile(moder) & !(0b11 << two)) | (0b10 << two),
            ); // alternate function
            core::ptr::write_volatile(ospeedr, core::ptr::read_volatile(ospeedr) | (0b11 << two)); // very high speed
            core::ptr::write_volatile(pupdr, core::ptr::read_volatile(pupdr) & !(0b11 << two)); // no pull
            if pin < 8 {
                let four = pin * 4;
                core::ptr::write_volatile(
                    afrl,
                    (core::ptr::read_volatile(afrl) & !(0xF << four)) | (12 << four),
                );
            } else {
                let four = (pin - 8) * 4;
                core::ptr::write_volatile(
                    afrh,
                    (core::ptr::read_volatile(afrh) & !(0xF << four)) | (12 << four),
                );
            }
        }
    }

    /// Spin until FMC_SDSR.BUSY (bit 5) clears, bounded so a mis-configured
    /// controller can't hang forever.
    unsafe fn wait_fmc_ready() {
        for _ in 0..1_000_000u32 {
            if core::ptr::read_volatile(FMC_SDSR) & (1 << 5) == 0 {
                return;
            }
        }
    }

    /// Route the FMC kernel clock to PLL2R (D1CCIPR.FMCSEL = 10b).
    ///
    /// PLL2R itself (200 MHz) is configured up-front by [`crate::clocks::init`]
    /// through the HAL's PLL builder, so this app-side code no longer touches
    /// the PLL registers — it only selects PLL2R as the FMC source (the kernel
    /// mux is not part of `freeze()`). PLL2 is already on and locked by the time
    /// any app runs.
    unsafe fn route_fmc_to_pll2r() {
        let mut ccipr = core::ptr::read_volatile(RCC_D1CCIPR);
        ccipr = (ccipr & !0b11) | 0b10;
        core::ptr::write_volatile(RCC_D1CCIPR, ccipr);
    }

    /// Bring up the external SDRAM. `delay_us` must block for at least the
    /// given microseconds at the current core clock (used for the >100 µs
    /// SDRAM power-up settle between clock-enable and precharge).
    ///
    /// # Safety
    /// Reconfigures RCC PLL2, the FMC controller and ~50 GPIO pins by direct
    /// register access; the caller must own those peripherals. See the
    /// module-level HARDWARE-VALIDATION note — do not treat as validated.
    pub unsafe fn init(delay_us: impl Fn(u32)) {
        // Enable GPIO port clocks (AHB4ENR: A=0 … I=8) for C,D,E,F,G,H,I,
        // then the FMC clock (AHB3ENR bit 12).
        let gpio_mask: u32 =
            (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8);
        core::ptr::write_volatile(
            RCC_AHB4ENR,
            core::ptr::read_volatile(RCC_AHB4ENR) | gpio_mask,
        );
        core::ptr::write_volatile(
            RCC_AHB3ENR,
            core::ptr::read_volatile(RCC_AHB3ENR) | (1 << 12),
        );
        cortex_m::asm::dsb();

        for &(port, mask) in PINS {
            configure_af12(port, mask);
        }

        route_fmc_to_pll2r();

        // Program the (host-validated) SDRAM control + timing registers.
        core::ptr::write_volatile(FMC_SDCR1, SEED.sdcr1());
        core::ptr::write_volatile(FMC_SDTR1, SEED.sdtr1());

        // JEDEC power-up sequence, all targeting bank 1 (SDCMR.CTB1 = bit 4).
        const CTB1: u32 = 1 << 4;
        // 1) Clock Configuration Enable (MODE=001).
        wait_fmc_ready();
        core::ptr::write_volatile(FMC_SDCMR, 0b001 | CTB1);
        // 2) >100 µs settle.
        delay_us(200);
        // 3) Precharge All (MODE=010).
        wait_fmc_ready();
        core::ptr::write_volatile(FMC_SDCMR, 0b010 | CTB1);
        // 4) Auto-Refresh (MODE=011), NRFS = cycles-1 in bits[8:5].
        wait_fmc_ready();
        let nrfs = (SEED.auto_refresh_cycles as u32 - 1) << 5;
        core::ptr::write_volatile(FMC_SDCMR, 0b011 | CTB1 | nrfs);
        // 5) Load Mode Register (MODE=100), MRD in bits[22:9].
        wait_fmc_ready();
        core::ptr::write_volatile(FMC_SDCMR, 0b100 | CTB1 | SEED.sdcmr_mrd());
        // 6) Program the refresh timer.
        wait_fmc_ready();
        core::ptr::write_volatile(FMC_SDRTR, SEED.sdrtr());
        wait_fmc_ready();
    }
}
