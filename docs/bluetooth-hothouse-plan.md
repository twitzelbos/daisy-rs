# Bluetooth for the Daisy Seed in a Hothouse — plan

**Status:** plan only — nothing built yet.
**Goal:** add a Bluetooth link to the Seed 3 installed in a Cleveland Music Co.
Hothouse pedal.

- **Must have:** a fluid serial terminal over Bluetooth (mirror the USB-CDC
  terminal / `daisy-hothouse` control panel over the air).
- **Nice to have:** audio streaming (A2DP source → Bluetooth headphones/speaker).

This constrains everything to the **free pins the Hothouse leaves on the Seed**,
which must be tapped at the Seed's own pin stubs — the Hothouse sockets the Seed
on single-row female headers and routes only the pins it uses, so an unused pin
is a dead socket position, not a pad.

---

## 1. Free-pin inventory (Hothouse)

The Hothouse consumes 16 of the Seed's 31 header pins (6 knobs, 3 toggles ×2, 2
footswitches, 2 LEDs — see `daisy_bsp::hothouse`). **15 pins are free.** Relevant
alternate functions (verified against `stm32h7xx-hal` 0.16 `serial.rs` /
`sai/i2s.rs`, not guessed):

| Seed | STM32 | Free? | Useful AFs on this pin |
|------|-------|-------|------------------------|
| D0   | PB12  | ✅ | I2S2_WS |
| D1   | PC11  | ✅ | **USART3_RX** (AF7), UART4_RX (AF8) |
| D2   | PC10  | ✅ | **USART3_TX** (AF7), UART4_TX (AF8) |
| D3   | PC9   | ✅ | (SDMMC/TIM) |
| D4   | PC8   | ✅ | (SDMMC/TIM) |
| D11  | PB8   | ✅ | **UART4_RX** (AF8), I2C1 |
| D12  | PB9   | ✅ | **UART4_TX** (AF8), I2C1 |
| D13  | PB6   | ✅ | **USART1_TX** (AF7) |
| D14  | PB7   | ✅ | **USART1_RX** (AF7) |
| D15  | PC0   | ✅ | **SAI2_FS_B** (AF8) |
| D24  | PA1   | ✅ | **SAI2_MCLK_B** (AF10) |
| D27  | PG9   | ✅ | **SAI2_FS_B** (AF10), USART6_RX |
| D28  | PA2   | ✅ | **SAI2_SCK_B** (AF8), USART2_TX |
| D29  | PB14  | ✅ | USART1 (AF4) |
| D30  | PB15  | ✅ | USART1 (AF4) |

Used by the Hothouse (for reference): D5/D6/D7/D8/D9/D10 (toggles), D16–D21
(knobs), D22/D23 (LEDs), D25/D26 (footswitches).

---

## 2. Module recommendation

The serial terminal is easy (any UART Bluetooth module). Audio is a **much**
bigger step: it needs A2DP (Bluetooth Classic) *and* a second I2S bus off the
Seed (§4). Three candidates:

| Module | Size | Terminal | Audio | Interface | Effort |
|--------|------|----------|-------|-----------|--------|
| **ESP32-MINI-1** (original ESP32) | ~13×17 mm | BT Classic SPP — fast, fluid | ✅ A2DP source via I2S | UART + I2S | write ESP32 firmware |
| **Microchip BM83** | ~16×27 mm | SPP (command-driven) | ✅ A2DP + onboard codec/DSP | UART + I2S | UART command set |
| **Microchip RN4678** | ~11.5×21.5 mm | SPP + BLE, ASCII API | ❌ data only | UART | lowest |

**Recommendation: ESP32-MINI-1.** Tiny, rock-solid high-throughput SPP terminal
today, A2DP audio path later (`esp-idf` `a2dp_source` fed from Seed I2S), biggest
example base. If you'd rather not write ESP32 firmware and want audio as a
UART-driven black box, use the **BM83**. If audio is dropped, the **RN4678** is
the smallest/simplest.

**Accuracy trap:** only the *original* ESP32 (Xtensa) does A2DP. ESP32-**C3/C6/
S3/H2** are BLE-only — no Bluetooth Classic, no A2DP. Pick an original-ESP32
module (ESP32-MINI-1, ESP32-PICO, ESP32-WROOM-32E).

---

## 3. Phase 1 — serial terminal over Bluetooth (the must-have)

Point the module's UART at **USART1** (the cleanest free UART), and bridge it to
the CDC terminal so the Bluetooth link mirrors the USB one.

### Wiring

| Seed pin | STM32 (AF) | → Module |
|----------|------------|----------|
| D13 | PB6, USART1_TX (AF7) | UART RX |
| D14 | PB7, USART1_RX (AF7) | UART TX |
| 3V3\* | — | VCC (see §5) |
| GND | — | GND |

\* Power from VIN/5 V through the module's own regulator, not the Seed's analog
3V3 — see §5.

**No hardware flow control.** USART1's RTS/CTS are PA11/PA12, which are the
Seed's on-board USB. Run the SPP link without RTS/CTS (fine for a terminal), or
at a moderate baud. If flow control ever proves necessary, move to **UART4**
(D11/D12) or **USART3** (D1/D2) and check their RTS/CTS pins against the free
list.

### Seed firmware

1. `daisy_bsp` helper: bring up USART1 on PB6/PB7 (`serial()` from the HAL, with
   the recovered `CoreClocks`), DMA or interrupt RX.
2. In `daisy-hothouse` (or a new `daisy-hothouse-bt`): run the same `panel::Panel`
   renderer, but `drain_to()` the UART *as well as* the CDC — the SerialBackend's
   cell diffing keeps a live redraw to a few bytes, which SPP handles easily.
3. Feed UART RX bytes into `panel.on_input()` so a Bluetooth terminal's DSR/CPR
   size reply resizes the UI exactly like the USB path.

### ESP32 firmware (if ESP32)

`BluetoothSerial` (Arduino) or `esp-idf` SPP: bytes ↔ UART. ~10 lines.

---

## 4. Phase 2 — audio over Bluetooth (the nice-to-have)

### Finding: a second I2S does NOT fit on free pins alone

Checked all four SAI blocks in `stm32h7xx-hal` `sai/i2s.rs`. Among the 15 free
pins, only **four** are SAI-capable, and they cover only clock/FS/MCLK — **never
the data line**:

| SAI2 block B role | Free pin options |
|-------------------|------------------|
| MCLK_B | **PA1 / D24** (AF10) |
| SCK_B  | **PA2 / D28** (AF8) |
| FS_B   | **PC0 / D15** (AF8) or **PG9 / D27** (AF10) |
| **SD_B (data)** | PA0 (footswitch 1), PG10 (toggle 2), PE11, PF11 |

Every SAI **SD** pin, across SAI1–SAI4, is either a **used Hothouse control**
(PA0, PC1, PD11, PG10) or **not broken out** on the Seed header (PB2, PD1, PD6,
PD9, PE3, PE6, PE11, PF6, PF11). SAI1 itself is fully committed to the on-board
TAC5242 (all on port E). The HAL has no SPI-as-I2S driver, and the SPI/I2S pin
sets can't assemble CK+WS+SD on free pins either.

**Conclusion:** you can get I2S clocks + FS + MCLK for free, but the **audio data
line costs exactly one Hothouse control.** Given the constraint *"any control but
a footswitch,"* the sacrifice is **toggle 2** — its up-contact **PG10 (D7)** is a
`SAI2_SD_B` pin (AF10), on **block B, whose clocks are all free**.

> Why not a knob? The only knob that is a SAI data pin is **knob 5 (PC1)**, but
> PC1 is `SAI2_SD_`**`A`** — and SAI2 block A's clocks (SCK_A/FS_A/MCLK_A =
> PD13/PD12/PE0/PIx) are **not broken out** on the Seed. A knob therefore can't
> carry the data line with free clocks; **toggle 2 is the only workable
> non-footswitch sacrifice.**

**Graceful degradation:** taking PG10 (up) leaves toggle 2's *down* contact
(PG11 / D8) intact, so it survives as a **2-position switch** (down vs. not-down);
only the up-vs-middle distinction is lost. `daisy_bsp::hothouse` would expose it
as a 2-state control in this build. **This degradation is avoidable** — see the
*Amendment* below, which keeps toggle 2 full by relocating it off PG10.

### Wiring (Phase 2 — sacrifices toggle 2)

Simplest topology: the **module is the I2S master** (generates BCLK + LRCLK), the
Seed's SAI2-B is a **slave transmitter** (SD is its only output; MCLK unneeded).

| Seed pin | STM32 (AF) | Role | → Module I2S |
|----------|------------|------|--------------|
| D28 | PA2, SAI2_SCK_B (AF8) | BCLK (in) | BCLK out |
| D15 | PC0, SAI2_FS_B (AF8) | LRCLK/WS (in) | LRCLK out |
| D7  | PG10, SAI2_SD_B (AF10) | data (out) | DIN ← *was toggle 2 up* |
| D24 | PA1, SAI2_MCLK_B (AF10) | MCLK (opt.) | MCLK — only if the module needs it |

If instead the **Seed is I2S master**, the same three pins carry BCLK/WS as
*outputs* plus SD; MCLK (D24) optionally drives the module's codec.

### Amendment — keep toggle 2 by relocating it (PG10 ↔ free-GPIO swap)

The toggle-2 sacrifice above is **avoidable**, because the two things fighting
over **PG10** are not equally picky:

- **`SAI2_SD_B` is fixed to PG10.** The audio data line has no alternative among
  usable pins (recap: `SD_B` exists only on `PA0` = footswitch 1, `PG10` =
  toggle 2, and `PE11`/`PF11`, which aren't broken out on the Seed header). It
  **cannot** move to a free pin.
- **A toggle is just a GPIO input.** Toggle 2 reads two switch throws to ground
  (up = `PG10`, down = `PG11`) with internal pull-ups; "middle" = both high.
  *Any* of the 15 free pins can serve as that input — none of the alternate-
  function machinery matters.

So swap ownership of PG10: give the **inflexible** signal (audio) the pin it must
have, and **move the flexible one** (the toggle) to a free GPIO.

**The swap**

- Move **toggle-2's up throw off `PG10`** onto a free GPIO. Its down throw
  (`PG11 / D8`) does not move.
- Route **`PG10` to the module's I2S `DIN`** (`SAI2_SD_B`, AF10) exactly as in
  the Phase-2 wiring table above.
- Toggle 2 stays a full **3-position** control; nothing is degraded.

**Which free pin.** Pick one with no other Phase-1/2 role — i.e. avoid the UART
pair (`D13/D14`) and the SAI2 clocks (`D15` FS, `D24` MCLK, `D28` SCK). The
cleanest are **`D3 / PC9`** or **`D4 / PC8`** (plain GPIO; their only alt is
unused SDMMC/TIM). `D0/PB12`, `D29/PB14`, `D30/PB15` also work.

| Toggle-2 throw | Was | Becomes |
|----------------|-----|---------|
| up   | PG10 / D7 | **PC9 / D3** (plain GPIO input + pull-up) |
| down | PG11 / D8 | PG11 / D8 (unchanged) |
| → frees | — | **PG10 / D7** for `SAI2_SD_B` (I2S DIN) |

**This is a hardware change, not firmware-only.** On the *existing* Hothouse it
means lifting toggle-2's up leg off the PG10 trace and jumpering it to the D3
stub. But our carrier boards are custom, so a **next carrier revision gets this
for free**: route toggle-2-up to D3 and PG10 to the module's DIN by design, and
audio-over-BLE *and* all three toggle positions coexist with zero soldering.

**Firmware.** `daisy_bsp::hothouse` reads toggle 2's "up" state from the relocated
pin (`PC9`) instead of `PG10`; the "down" read (`PG11`) is unchanged, so the
`ToggleswitchPosition` Up/Middle/Down decode is identical. It's a one-line pin
change in the toggle-2 constructor, gated on a board-revision feature so the
un-modded build still reads `PG10`.

**Caveats.**

- Confirm the chosen pin is a plain input on your build (`PC9`/`PC8` are the free
  SDMMC1 D1/D0 — unused here). Enable the internal pull-up to match the other
  toggles.
- If Phase 1's BLE UART ever moves off USART1 to `UART4` (`D11/D12`) or `USART3`
  (`D1/D2`), keep the relocated toggle pin clear of that pair too.

### Firmware / clocking considerations

- **SAI2 kernel clock:** SAI1 already runs off PLL3 (set by the bootloader; the
  XIP app re-points `SAI1SEL` at PLL3P). SAI2 has its own kernel mux (`SAI23SEL`
  in `D2CCIP1R`) — point it at PLL3P too, or verify PLL3 has the headroom. This is
  a bootloader/`daisy-bsp` clocks change, done with the same
  `kernel_clk_mux(Sai2ClkSel::Pll3P)` pattern as SAI1.
- **DMA:** SAI2 gets its own DMA stream/request line — mirror the existing
  SAI1↔DMA path in `daisy-audio`.
- **Sample-rate match:** the A2DP module and SAI2 must agree on rate (typically
  44.1 or 48 kHz). Resample on the Seed if the codec path differs.

### Lower-cost audio fallback (no extra pins, lower quality)

Stream audio as **PCM/compressed bytes over the SPP link** (Phase 1's UART) and
let the ESP32 do A2DP. No second I2S, no lost control — but UART bandwidth caps
it: 48 kHz × 16-bit stereo ≈ 1.5 Mbit/s raw is tight even at 2 Mbaud, so this is
realistically **mono or compressed**, not full-quality stereo. Fine for talkback
/ monitoring, not for hi-fi.

---

## 5. Power

- Bluetooth TX current spikes (~250 mA on the ESP32). Do **not** feed the module
  from the Seed's analog 3V3 LDO — it will disturb the knob ADC references.
- Feed from **VIN / 5 V** (Seed pin) through the module's own regulator (most
  ESP32 and Microchip dev modules include one), or a dedicated small LDO.
- Common ground with the Seed.

---

## 6. Consolidated wiring (terminal + optional audio, ESP32)

```
Seed (in Hothouse)                         ESP32-MINI-1 module
------------------                         -------------------
D13 PB6  USART1_TX  ───────────────────▶   UART RX
D14 PB7  USART1_RX  ◀───────────────────   UART TX
VIN 5V   ──────────▶ [reg] ─────────────▶  VCC
GND      ───────────────────────────────   GND

# Phase 2 only (costs toggle 2):
D28 PA2  SAI2_SCK_B ◀───────────────────   I2S BCLK   (module = master)
D15 PC0  SAI2_FS_B  ◀───────────────────   I2S LRCLK
D7  PG10 SAI2_SD_B  ───────────────────▶   I2S DIN
D24 PA1  SAI2_MCLK_B ──────────────────▶   MCLK       (optional)

# Phase 2 variant that KEEPS toggle 2 (see §4 Amendment): additionally
#   relocate toggle-2 up leg  PG10/D7 → PC9/D3 (plain GPIO), which frees
#   PG10 for SAI2_SD_B above. Best baked into a carrier-board revision.
```

---

## 7. Risks & open questions

- **A2DP latency (~100–200 ms) + lossy** — good for casual listening, useless for
  real-time pedal monitoring. Set expectations accordingly.
- **Losing toggle 2** (degraded to a 2-position switch) for audio is the agreed
  cost **only if you don't do the PG10↔free-GPIO swap** (§4 Amendment), which
  keeps toggle 2 full at the price of one jumper — and is *free* if baked into a
  carrier-board revision. The UART-PCM fallback (§4) keeps all controls with no
  rewiring at all.
- **PLL3 headroom for SAI2** — verify the fractional PLL can clock both SAI1 and
  SAI2 at the target rate, or add a PLL path.
- **Physical tap** — the audio pins (PG10/PA1/PA2/PC0) and the two UART pins must
  be hand-wired to the Seed pin stubs; plan the mechanical mount.
- **ESP32 firmware** is a second codebase to maintain (out of the all-Rust tree,
  unless using `esp-hal`/`esp-idf` Rust).

---

## 8. Suggested phasing

1. **Phase 1a** — USART1 helper in `daisy-bsp` + a CDC↔UART bridge so the existing
   control-panel TUI streams over the module's UART. Validate with a bare UART-USB
   dongle first (no Bluetooth), then the module.
2. **Phase 1b** — pair the module, confirm a fluid SPP terminal.
3. **Phase 2** (optional) — if audio: SAI2 clock route + DMA in
   `daisy-bsp`/`daisy-audio`, ESP32 A2DP source, wire the I2S. Decide toggle 2:
   either accept the 2-position degradation, or apply the **PG10↔free-GPIO swap**
   (§4 Amendment) to keep it full — ideally folded into the next carrier-board
   revision so PG10→DIN and toggle-2-up→D3 are there by design.
