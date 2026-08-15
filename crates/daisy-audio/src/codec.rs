//! Codec-independent descriptors for the Daisy's audio codecs.
//!
//! This module is **pure logic and host-testable** (no target gate): it names
//! the codecs, resolves which one a board has from its version straps, and
//! carries the per-codec facts (word size, full-scale, SAI topology, channel
//! order, I²C program) that the target-only bring-up in `crate::bare` consumes.
//!
//! Ground truth: libDaisy `src/daisy_seed.cpp` `Configure()` /
//! `CheckBoardVersion()` (four board revisions, three classic + Seed 3) and the
//! WM8731 datasheet (`reference/WolfsonWM8731.pdf`).

/// The audio codec fitted to a given Daisy board.
///
/// The three *classic* Seeds (AK4556, WM8731, PCM3060) are distinguished at
/// runtime by two version straps ([`Codec::from_straps`]); the Seed 3's TAC5242
/// is a different board with 32-bit framing and is selected at compile time via
/// the `seed3` feature, never runtime-detected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Codec {
    /// Daisy Seed v1 (rev4) — AKM **AK4556**. Pin-strapped format; needs a
    /// reset pulse on PB11. SAI block A = TX, B = RX. 24-bit.
    Ak4556,
    /// Daisy Seed 1.1 (rev5) — Wolfson **WM8731**. Configured over I²C2
    /// (SCL = PH4, SDA = PB11). SAI block A = **RX**, B = TX. 24-bit.
    Wm8731,
    /// Daisy Seed 2 DFM — TI **PCM3060**. Pin-strapped; the de-emphasis-disable
    /// input on PB11 is held low. SAI block A = TX, B = RX. 24-bit.
    Pcm3060,
    /// Daisy Seed 3 — TI **TAC5242**. Hardware-strapped, no I²C/reset. SAI
    /// block A = TX master, B = RX slave. 32-bit MSB.
    Tac5242,
}

impl Codec {
    /// Resolve the *classic* Seed codec from the two board-version straps
    /// (active-low, internally pulled up): PD3 low ⇒ Seed 1.1 (WM8731), else
    /// PD4 low ⇒ Seed 2 DFM (PCM3060), else Seed v1 (AK4556). Mirrors libDaisy
    /// `CheckBoardVersion`. Never returns [`Codec::Tac5242`] — the Seed 3 is a
    /// separate board selected by the `seed3` feature.
    pub const fn from_straps(pd3_low: bool, pd4_low: bool) -> Codec {
        if pd3_low {
            Codec::Wm8731
        } else if pd4_low {
            Codec::Pcm3060
        } else {
            Codec::Ak4556
        }
    }

    /// SAI word length in bits (drives `I2SDataSize`).
    pub const fn bits(self) -> u8 {
        match self {
            Codec::Tac5242 => 32,
            _ => 24,
        }
    }

    /// Full-scale magnitude for `i32(word) ↔ f32` conversion — `2^(bits-1)`.
    pub const fn scale(self) -> f32 {
        match self {
            Codec::Tac5242 => 2_147_483_648.0, // 2^31
            _ => 8_388_608.0,                  // 2^23
        }
    }

    /// Whether SAI block **A** carries the capture (RX) line.
    ///
    /// Only the WM8731 (Seed 1.1) wires the codec ADC onto SD_A (PE6); every
    /// other Daisy board captures on SD_B (PE3) with block A transmitting. This
    /// flips the whole SAI master/slave + DMA-channel topology, so it is the one
    /// bit the bring-up branches on.
    pub const fn a_is_rx(self) -> bool {
        matches!(self, Codec::Wm8731)
    }

    /// Whether the codec presents/accepts frames as `[right, left]` and needs a
    /// channel swap at the board boundary. Only the TAC5242 does (verified
    /// empirically in daisy-embassy on a stock Daisy Pod).
    pub const fn swap_lr(self) -> bool {
        matches!(self, Codec::Tac5242)
    }

    /// Whether this codec is configured over I²C2 (WM8731 only). If false the
    /// codec is pin-strapped and PB11 is a plain GPIO (reset / de-emphasis).
    pub const fn uses_i2c(self) -> bool {
        matches!(self, Codec::Wm8731)
    }
}

/// WM8731 7-bit I²C device address (CSB tied low on the Daisy).
pub const WM8731_I2C_ADDR: u8 = 0x1A;

/// Pack a WM8731 control word — 7-bit register index in bits [15:9] plus 9 data
/// bits — into the two I²C bytes the part expects
/// (`byte0 = index<<1 | data[8]`, `byte1 = data[7:0]`).
pub const fn wm8731_control_word(reg: u8, val: u16) -> [u8; 2] {
    [(reg << 1) | ((val >> 8) as u8 & 1), (val & 0xFF) as u8]
}

/// WM8731 register program for I²S / 24-bit / 48 kHz / **slave** with an
/// external 12.288 MHz MCLK (256 × fs) from the SAI. `(register index, 9-bit
/// value)`, applied in order. Field-verified against the datasheet (Tables
/// 3–29): powers down the internal oscillator, CLKOUT and the unused mic.
pub const WM8731_INIT: [(u8, u16); 10] = [
    (0x0F, 0b0_0000_0000), // R15 reset
    (0x06, 0b0_0110_0010), // R6 power: up line-in/ADC/DAC/out; OSC+CLKOUT+MIC off
    (0x00, 0b0_0001_0111), // R0 L line-in: 0 dB, unmute
    (0x01, 0b0_0001_0111), // R1 R line-in: 0 dB, unmute
    (0x04, 0b0_0001_0010), // R4 analog: INSEL=line, BYPASS off, DACSEL on, MUTEMIC on
    (0x05, 0b0_0000_0000), // R5 digital: DAC un-muted, ADC HPF on, de-emph off
    (0x07, 0b0_0000_1010), // R7 interface: I2S (FORMAT=10), 24-bit (IWL=10), slave (MS=0)
    (0x08, 0b0_0000_0000), // R8 sampling: NORMAL, 256fs, SR=0000 → 48 kHz @ 12.288 MHz MCLK
    (0x09, 0b0_0000_0001), // R9 activate
    (0x06, 0b0_0110_0010), // R6 power: same as above (keep OSC/CLKOUT/MIC OFF — NOT 0)
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straps_match_libdaisy_truth_table() {
        // PD3 low ⇒ Seed 1.1; PD4 low ⇒ Seed 2 DFM; neither ⇒ Seed v1.
        assert_eq!(Codec::from_straps(true, false), Codec::Wm8731);
        assert_eq!(Codec::from_straps(true, true), Codec::Wm8731); // PD3 wins
        assert_eq!(Codec::from_straps(false, true), Codec::Pcm3060);
        assert_eq!(Codec::from_straps(false, false), Codec::Ak4556);
    }

    #[test]
    fn word_size_and_scale() {
        for c in [Codec::Ak4556, Codec::Wm8731, Codec::Pcm3060] {
            assert_eq!(c.bits(), 24);
            assert_eq!(c.scale(), 8_388_608.0);
        }
        assert_eq!(Codec::Tac5242.bits(), 32);
        assert_eq!(Codec::Tac5242.scale(), 2_147_483_648.0);
        // scale == 2^(bits-1)
        for c in [Codec::Ak4556, Codec::Tac5242] {
            assert_eq!(c.scale(), 2f32.powi(c.bits() as i32 - 1));
        }
    }

    #[test]
    fn topology_flags() {
        // Only the WM8731 captures on block A.
        assert!(Codec::Wm8731.a_is_rx());
        for c in [Codec::Ak4556, Codec::Pcm3060, Codec::Tac5242] {
            assert!(!c.a_is_rx());
        }
        // Only the TAC5242 swaps L/R.
        assert!(Codec::Tac5242.swap_lr());
        for c in [Codec::Ak4556, Codec::Wm8731, Codec::Pcm3060] {
            assert!(!c.swap_lr());
        }
        // Only the WM8731 talks I²C.
        assert!(Codec::Wm8731.uses_i2c());
        for c in [Codec::Ak4556, Codec::Pcm3060, Codec::Tac5242] {
            assert!(!c.uses_i2c());
        }
    }

    #[test]
    fn wm8731_control_word_packing() {
        // R7 = 0x0A: byte0 = 0x07<<1 | 0 = 0x0E, byte1 = 0x0A.
        assert_eq!(wm8731_control_word(0x07, 0b0_0000_1010), [0x0E, 0x0A]);
        // A 9th data bit lands in byte0 bit0: reg 0, val 0x100 → [0x01, 0x00].
        assert_eq!(wm8731_control_word(0x00, 0x100), [0x01, 0x00]);
        // Reset R15 = 0: [0x1E, 0x00].
        assert_eq!(wm8731_control_word(0x0F, 0), [0x1E, 0x00]);
        // The activate write R9 = 1 → [0x12, 0x01].
        assert_eq!(wm8731_control_word(0x09, 1), [0x12, 0x01]);
    }
}
