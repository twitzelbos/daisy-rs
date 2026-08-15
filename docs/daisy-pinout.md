# Daisy Seed — essential peripheral pinout

The one-stop map of which STM32H750 pin each on-board peripheral uses, so we
stop rediscovering it by scope and SWD. **Ground truth:** `reference/libDaisy`
(`src/daisy_seed.cpp`, `src/per/qspi.cpp`, `src/dev/sdram.cpp`) cross-checked
against our own hardware bring-up. When in doubt, grep libDaisy — do not guess a
peripheral's pin (see [check-reference-code] in the project memory).

> **Pin traps that have each cost us multiple days — read these first.**
>
> | Peripheral | The trap | Correct | Wrong-but-plausible |
> |---|---|---|---|
> | **SDRAM `SDNWE`** | Write-enable is on **PH5** on this board. libDaisy configures it on **both PC0 and PH5** because revisions differ. Configure only PC0 and a *cold* SDRAM init leaves PH5 floating → every address aliases to one cell. | **PH5** (+ PC0 for other revs) | PC0 only |
> | **USB (on-board micro)** | The micro-USB connector is **OTG_FS on PA11/PA12** (AF10). PB14/PB15 are OTG1_HS on the D29/D30 *breakout* header, not the connector. | **PA11/PA12** | PB14/PB15 |
> | **FMC controller enable** | `FMC_BCR1.FMCEN` (bit 31) is a *separate* enable from the `RCC AHB3ENR.FMCEN` clock gate. Both are required. | set both | set only the clock gate |
> | **Audio SD lines** | The ADC/DAC data lines are **swapped between board revs**: Seed 1.1 captures on SD_A (PE6); every other board captures on SD_B (PE3). | per-codec (see below) | assuming one direction |

All pin names are STM32H750 port pins. "Daisy pin" = the number silkscreened on
the Seed's castellated header, where relevant.

---

## SDRAM — FMC bank 1 (Alliance AS4C16M32SB, 64 MB @ `0xC000_0000`)

All FMC signals are **AF12**, `VERY_HIGH` speed, no pull, push-pull. 57 pins
total; leaving **any one** floating on a cold init produces address aliasing
that is invisible at the FMC register level. See `crates/daisy-bsp/src/sdram.rs`.

| Signal | Pin(s) |
|---|---|
| Address A0–A5 | PF0, PF1, PF2, PF3, PF4, PF5 |
| Address A6–A9 | PF12, PF13, PF14, PF15 |
| Address A10–A12 | PG0, PG1, PG2 |
| Bank BA0, BA1 | PG4, PG5 |
| Data D0–D1 | PD14, PD15 |
| Data D2–D3 | PD0, PD1 |
| Data D4–D12 | PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15 |
| Data D13–D15 | PD8, PD9, PD10 |
| Data D16–D23 | PH8, PH9, PH10, PH11, PH12, PH13, PH14, PH15 |
| Data D24–D27 | PI0, PI1, PI2, PI3 |
| Data D28–D31 | PI6, PI7, PI9, PI10 |
| Byte-lane NBL0–NBL3 | PE0, PE1, PI4, PI5 |
| `SDCLK` | PG8 |
| `SDNRAS` (row strobe) | PF11 |
| `SDNCAS` (col strobe) | PG15 |
| **`SDNWE` (write enable)** | **PH5** on this board (libDaisy also drives PC0) ⚠️ |
| `SDCKE0` (clock enable) | PH2 |
| `SDNE0` (chip select) | PH3 |

Controller notes (not pins, but part of the same trap):
- Set **`FMC_BCR1.FMCEN`** (bit 31) — the controller enable — after `SDCR1`/
  `SDTR1`, before the JEDEC command sequence. Separate from `RCC AHB3ENR.FMCEN`.
- Clear `FMC_BCR1.MBKEN` (bit 0) to disable NOR bank 1, matching ST `SystemInit`.
- FMC kernel clock = **PLL2R = 200 MHz** → SDCLK ÷2 = 100 MHz (`D1CCIPR.FMCSEL=10`).

Full story: `memory/project_sdram_config.md`.

---

## QSPI flash — IS25LP064A (8 MB, XIP @ `0x9000_0000`)

Bank 1, quad mode. IO/CLK are **AF9**, NCS is **AF10** (see
`crates/daisy-boot/src/qspi.rs`).

| Signal | Pin | AF |
|---|---|---|
| IO0 (SI) | PF8 | 10 |
| IO1 (SO) | PF9 | 10 |
| IO2 | PF7 | 9 |
| IO3 | PF6 | 9 |
| CLK | PF10 | 9 |
| NCS | PG6 | 10 |

---

## USB — on-board micro-USB connector

**OTG2_HS in FS mode via the internal FS PHY**, AF10. This is the connector you
plug into; the HAL peripheral is `USB2`/`OTG_FS`.

| Signal | Pin | AF |
|---|---|---|
| USB D− | PA11 | 10 |
| USB D+ | PA12 | 10 |

USB kernel clock = **HSI48** (48 MHz RC), `USBSEL = HSI48`. **Not** PB14/PB15 —
those are OTG1_HS on the D29/D30 breakout header. See `memory/feedback_daisy_usb_pins.md`.

---

## Audio codec — SAI1 (per board revision)

The SAI1 clock/frame/MCLK pins are the same across all Seeds; **the data-line
direction and the codec-control pin change per revision.** The three classic
Seeds are auto-detected at runtime from version straps; the Seed 3 is a separate
`seed3` build. See `crates/daisy-audio/src/{codec,lib}.rs`.

Common SAI1 pins (all GPIOE, AF6):

| Signal | Pin |
|---|---|
| MCLK_A | PE2 |
| SCK_A (bit clock) | PE5 |
| FS_A (frame sync) | PE4 |
| SD_A (data line A) | PE6 |
| SD_B (data line B) | PE3 |

Per-codec configuration:

| Board (strap) | Codec | Init | SD_A / SD_B | Word | PB11 is… |
|---|---|---|---|---|---|
| Seed v1 (rev4, default) | **AK4556** | reset pulse on PB11, no I²C | A=TX, B=**RX** | 24-bit | reset line |
| Seed 1.1 (PD3→gnd) | **WM8731** | I²C2 registers | A=**RX**, B=TX | 24-bit | I²C2 SDA |
| Seed 2 DFM (PD4→gnd) | **PCM3060** | de-emphasis pin low | A=TX, B=**RX** | 24-bit | de-emph pin |
| Seed 3 (separate board) | **TAC5242** | hardware-strapped | A=TX, B=**RX** slave | 32-bit | — |

- **Board-version straps** (classic Seeds, internally pulled up, active-low):
  **PD3** low ⇒ Seed 1.1, **PD4** low ⇒ Seed 2 DFM, neither ⇒ Seed v1
  (libDaisy `CheckBoardVersion`).
- **PB11 is overloaded** — reset (AK4556) / I²C2 SDA (WM8731) / de-emphasis
  (PCM3060). What it must be depends on the detected codec, so it's configured
  after detection, not before.
- **I²C2** (WM8731 only): SCL = **PH4**, SDA = **PB11**, 400 kHz, AF4.
- Only the WM8731 captures on **SD_A**; every other board captures on **SD_B**
  — this flips the whole SAI master/slave + DMA-channel topology.

---

## Misc

| Function | Pin |
|---|---|
| User LED | PC7 |
| Boot pin (system bootloader) | BOOT0 |

---

## How to extend this doc

When you bring up a new peripheral, add its pins here **from libDaisy**, not
from memory. If a signal has revision-dependent or multiply-driven pins (like
`SDNWE`), list **all** of them and mark the trap — that ambiguity is exactly
what costs days on hardware.
