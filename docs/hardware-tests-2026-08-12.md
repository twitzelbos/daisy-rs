# Hardware test results — 2026-08-12

First on-silicon measurement + validation session for daisy-rs. All figures below
were taken on **real STM32H750 hardware**, not Renode.

## Setup

- **Board:** Daisy Seed 1.1 (STM32H750IB, Cortex-M7) seated in a Hothouse pedal.
- **Debug/flash:** ST-Link V3 over SWD, driven by `probe-rs` 0.32 (`--chip STM32H750IB`).
- **Method:** each measurement is a small **standalone-from-flash** exerciser (linked
  at `0x0800_0000` with its own `memory.x`, running on the HSI 64 MHz reset clock,
  code + data in DTCM, no cache) that writes results to fixed DTCM addresses. The
  host reads them back with `probe-rs read`. `probe-rs` halts the core for each
  access and resumes on detach, so where an exerciser self-paces we insert a short
  host-side delay between polls to give the core uninterrupted run time.
- The benches/exercisers overwrite the bootloader in internal flash; it is
  reflashed (`daisy-boot`) at the end of the session, and again here.

> These exercisers are throwaway measurement firmware, not shipped crates. The
> point of recording the numbers is to replace estimates with silicon truth and to
> close the HW-validation gaps that Renode structurally cannot cover.

---

## 1. DSP primitive cycle costs — `dsp-bench`

DWT `CYCCNT`, cycles per **64-sample block**, overhead-subtracted. HSI 64 MHz,
DTCM, **no cache** (so these are conservative vs. an XIP app running cached from
QSPI with the I/D caches on).

| Primitive   | cycles / 64-block | ~cycles / sample |
| ----------- | ----------------: | ---------------: |
| OnePole     |             1 757 |             27.5 |
| Biquad      |             1 780 |             27.8 |
| Env follower|             3 321 |             51.9 |
| Freeze      |             4 218 |             65.9 |
| Delay       |             7 505 |            117.3 |
| FDNReverb   |           140 189 |          2 190.5 |
| PadDrone    |           153 599 |          2 400.0 |

FFT (radix-2, cycles per transform):

| N    | cycles / transform |
| ---- | -----------------: |
| 256  |             44 969 |
| 512  |             98 120 |
| 1024 |            202 554 |
| 2048 |            438 385 |

These are the numbers to check DSP designs against the tier-C per-block budget.
(No-cache DTCM run — treat as the pessimistic bound.)

---

## 2. FFT shootout — `fft-shootout`

Same conditions; cycles per transform, ITERS = 32. Compares our `daisy-dsp` FFT
against `microfft`, real vs. complex, plus the Q15 SIMD complex path.

| Entrant                     |   256 |    512 |   1024 |   2048 |
| --------------------------- | ----: | -----: | -----: | -----: |
| ours `f32` **real**         | 49 499| 105 766| 221 607| 473 750|
| microfft **real**           | **30 206** | **69 413** | **165 384** | **350 873** |
| ours `f32` complex          | 68 791| 148 108| 324 974| 698 135|
| Q15 complex (CMSIS-style SIMD) | 68 046| 147 775| 319 916| 686 467|

Correctness sentinel `Q15_OK = 0x00C0FFEE` — **PASS**.

**Findings**

- **microfft real beats our `f32` real FFT ~1.35×** across every size — a real,
  reproducible gap worth chasing in `daisy-dsp`.
- **Q15 SIMD complex is only ~2% faster than `f32` complex.** On the M7's FPU the
  Q15 pack/scale overhead nearly cancels the SIMD win — the compute economy case
  for Q15 FFT on this part is weak.
- **There is no Q15-*real* entrant:** the `fft_q15` crate only implements
  `cfft_q15` (complex); there is no `rfft_q15`. So a Q15-real vs. f32-real
  comparison isn't available without writing one.

---

## 3. MPU + fault vectors — PASS

- **`mpu-exerciser`** — 16 MPU regions active (`MPU_TYPE.DREGION` = 16). A
  deliberate access to a no-access region faulted with `MMFSR = 0x82`
  (DACCVIOL + MMARVALID), `MMFAR = 0x2400_0000`, and the MemManage handler
  recovered cleanly (`M_DONE = 0x00C0FFEE`, `FAULT_FLAG = 0`).
- **`fault-exerciser`** — the exception vectors fire and the MemManage handler
  recovers the test sentinel (`0x00C0FFEE`).

The Renode-validated MPU region layout and fault-handling design hold on silicon.

---

## 4. ADC bring-up — PASS

- **`adc-exerciser`** reached stage `0x15`: the full `stm32h7xx-hal` `Adc::adc1`
  bring-up handshake (LDO ready, `ADCAL` self-clear, `ADRDY`) plus one conversion —
  the exact path the Hothouse knob reader uses. Live sample off PA3 (ADC1) =
  `0x451A` (17690, ~27% of full scale).

Confirms the analog front-end HAL path works on real silicon, not just in sim.

---

## 5. USB OTG device bring-up — PASS (to `UsbBus::new`)

- **`usb-init-exerciser`** reached stage `0x14`: the XIP app's **freeze-free**
  OTG2_HS (FS) bring-up **returns from `UsbBus::new`** — no hang on the OTG
  core-reset poll. The poll counter stays 0 because only SWD is attached (no USB
  host on the board's micro-USB), so the device correctly can't enumerate — but the
  hang-free `UsbBus::new` is precisely the thing Renode cannot prove (no OTG host
  model in sim).

Full USB-audio enumeration/streaming still needs the board's micro-USB connected to
a host — see "Not covered" below.

---

## 6. D-cache / DMA coherency — not reproducible over SWD (mechanism established)

The `cache-coherency-exerciser` maps a buffer at `0x3000_0000` (D2 SRAM) as
**cacheable + write-back** and tries to reproduce the classic DMA stale-read
hazard: CPU caches the line, a "foreign master" overwrites backing memory, CPU
re-reads. We tried to play the foreign master with `probe-rs`.

What we actually established on silicon:

1. **The probe *can* read and write D2 SRAM.** With `0x3000_0000` **not** cached
   (bootloader MPU), `probe-rs write` of `0xDEADBEEF` / `0xFEEDFACE` persisted and
   read back correctly. (An earlier session's "writes to D2 SRAM don't persist"
   claim was **wrong** and is retracted — it was not backed by the reference
   manual, and this directly refutes it.) For the record, `RCC_AHB2ENR`
   D2SRAM1/2/3EN gate the D2 SRAM clocks (reset value 0; libDaisy's `SystemInit`
   sets them — `reference/libDaisy/src/sys/system_stm32h7xx.c:205`), but the writes
   above persisted even with `AHB2ENR = 0`, so clock-gating is *not* the explanation
   either.
2. **When the region is cacheable and the CPU holds the line, the probe cannot act
   as a foreign master.** Injecting `0xC0FFEE00` while the CPU had the line cached
   was visible to the CPU through **neither** path: scenario A (no invalidate) read
   the stale cached `0`, and scenario B (SCB `DCIMVAC` invalidate → re-read from
   memory) *also* read `0` — the injected value never became CPU-visible. This is
   consistent with the **Cortex-M7 debug AHBS slave port being coherent with L1**:
   the debugger sees the same memory the CPU sees, so it is *not* an independent
   bus master behind the cache.
3. **Conclusion:** the DMA stale-cache read hazard is **structurally not
   reproducible via SWD** — it requires a real DMA engine on the AXI/AHB matrix
   (not cache-coherent), unlike the debugger's AHBS port. This is why the **Renode
   functional coherency checker** (`CacheCoherencyChecker.cs`, PR #22) is the CI
   tool for this, and why true on-HW validation needs a **DMA-driven** exerciser
   (e.g. mem-to-mem DMA, or the SAI/DMA audio path) writing a buffer the CPU has
   cached — a probe cannot stand in for it.

Status: **open** — needs a DMA-driven HW exerciser; the probe method is a dead end
by design, now with the reason understood rather than guessed.

---

## Not covered this session (needs board USB → host, or a DMA master)

- **USB-audio composite enumeration + streaming** (`daisy-usb-audio`: CDC + UAC +
  MIDI). The app is built and links at `0x9000_0000` (QSPI XIP); it loads via
  `daisy flash` (DFU through the bootloader), which needs the board's micro-USB on
  the host. Not attempted — only SWD was connected.
- **WM8731 / codec real audio**, isochronous USB-audio streaming, and D-cache/DMA
  coherency of the real `.sram_d2` audio buffers — all require the USB host and/or a
  DMA master.

## Summary

| Area                       | Result |
| -------------------------- | ------ |
| DSP primitive cycle costs  | measured (table §1) |
| FFT shootout               | measured; microfft-real ~1.35× faster, Q15≈f32 (§2) |
| MPU + fault vectors        | **PASS** |
| ADC bring-up               | **PASS** |
| USB OTG `UsbBus::new`       | **PASS** (no host attached) |
| D-cache/DMA coherency      | open — not reproducible via SWD; needs a DMA master (§6) |
