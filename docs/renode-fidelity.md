# Renode fidelity — what the simulation proves, and what only hardware can

Renode is a **functional** emulator (its CPU core, tlib, is a QEMU-TCG-style
dynamic binary translator). It models what instructions and peripherals *do*,
not micro-architectural timing, caches, or analog behaviour. This document draws
the boundary explicitly so a green sim run is never mistaken for evidence it
cannot provide.

**Rule of thumb:** trust Renode for *logic, registers, clock-tree configuration,
and vector/DMA/MPU wiring*. Never trust it for *timing, cache/DMA coherency, or
anything analog* — those are hardware-only, and have their own HW tests.

For the silicon-errata dimension — which published ES0392 limitations touch our
peripherals, which we work around, and which are un-modellable (caught instead
by a boot-time *presence* assertion) — see
[errata-coverage.md](errata-coverage.md).

## Sim is authoritative for (trust a green run)

| Concern | Model | Notes |
|---|---|---|
| Clock-tree configuration | `STM32H7_RCC_Clocked` | All 3 PLLs, DIVMx/DIVN/P/Q/R, FRACN, `sys_ck`; drives DWT. *Caught the PLL2 `PLLCFGR`-offset bug.* |
| DWT registers (CYCCNT gating, caps, wrap) | `STM32H7_DWT_Clocked` | ARMv7-M §C1.8-faithful; CYCCNT **rate** tracks real `sys_ck`. |
| MPU region enforcement | tlib patch (PRIVDEFENA) | tlib actually faults on MPU violations. |
| QSPI/OCTOSPI XIP behaviour | `STM32H7_QuadSPI` + `IS25LP064A` | Continuous-read, NCS pin-mux, dummy cycles. *Caught the XIP-boot bug.* |
| DMA circular / half-transfer / double-buffer | `STM32H7_DMA_Circular` | Logic + HTIF/TCIF/DBM. |
| SDRAM sizing, gating, timing registers | `STM32H7_FMC_SDRAM` | 64 MiB part; OOB faults. |
| PWR / VOS register logic | `STM32H7_PWR` | |
| Boot / vector table / exception dispatch | tlib + `fault-exerciser` | Reset SP/PC, SysTick, NVIC IRQ, MemManage entry+recovery. |
| USB OTG **register** bring-up (device mode) | `STM32H7_OTG` | `UsbBus::new` completes; enumeration exercised at the register level. |

## Sim CANNOT prove — hardware-only (never infer from a green run)

### 1. Cache / DMA coherency — **the highest-risk gap**
Renode's Cortex-M has **no cache model**. Cacheability attributes and cache-
maintenance ops (SCB `DCCIMVAC`/`DCISW`/…) have **no effect**. A DMA/USB/audio
buffer with the wrong cache attributes — or a missing clean/invalidate around a
DMA — **passes in sim and corrupts on silicon**. This is the exact class of bug
that produced the "QSPI/DMA buffer corruption" on real HW. tlib enforces the MPU
but not the cache. → **HW test required** for every DMA/CPU-shared buffer.

**Partially closed by a checker.** `renode/peripherals/CacheCoherencyChecker.cs`
(+ `cache-coherency-exerciser` + `cache_coherency.robot`) is a *functional
checker* — not a real cache — that overlays a region, classifies each access by
master (CPU vs foreign/DMA), tracks coherency-relevant line state, honours the
SCB maintenance ops via watchpoints, and **flags** the silent-failure cases in
CI: a stale CPU read after a foreign write (missing `DCIMVAC`), a stale DMA read
of a dirty line (missing `DCCMVAC`), and — for executable regions — a stale
instruction fetch of code written but not cleaned to PoU (`DCCMVAU`) or modified
without I-cache invalidate (`ICIALLU`). It cannot catch *every* miscoherency (it
assumes the watched region is cacheable + write-back and does not re-derive the
MPU attribute map), so real cache correctness of any *specific* buffer still
needs HW — but the bug class that bit us now fails loudly in sim.

**Closed on the HW side by `dma-cache-exerciser`.** The one thing SWD can never
be is the foreign master (the debug AHBS port is cache-coherent — see
`docs/hardware-tests-2026-08-12.md` §6). That firmware kicks a **DMA1
mem-to-mem** transfer to *be* the foreign master, so both hazards are reproducible
on silicon with only a probe reading DTCM markers. Its Renode counterpart
(`dma_cache.robot`) can validate the DMA programming but not the coherency
divergence — in sim both the buggy and correct variant read fresh (no cache
model, and a firmware-kicked DMA copy runs in CPU context so the checker can't
classify it foreign). That boundary is pinned by the robot and documented in §6a.

### 2. Timing / cycle accuracy / real-time budget
Functional translator: CYCCNT's *rate* is right, but cycles-per-code-region is
not silicon (no pipeline, dual-issue, FPU latency, wait states, cache). Also:
- **SAI frame rate is a fake constant** (chosen for `RunFor` speed), not 48 kHz —
  audio throughput/underruns are not represented.
- **PLL lock is instantaneous**; clock-switch timing is not cycle-exact.
→ Compute budgets come from **DWT on hardware** (`dsp-bench`), not sim.

### 3. Analog / codec / real audio path
- **Codec is not modelled** (TAC5242 / WM8731): no I2C-strapped config, no audio.
- **ADC** returns register values, not real conversions/voltages.
- **SAI** is a loopback queue, not a real audio stream.
→ The end-to-end audio path (codec↔SAI↔DMA↔DSP) is **HW-only**.

### 4. USB host-side behaviour
Device registers enumerate in sim, but there is **no real USB host stack**
validating our descriptors/class behaviour, and **isochronous streaming timing
is HW-only**. Whether a real OS accepts the composite CDC+UAC+MIDI device → HW.

### 5. Electrical
Absolute HSE frequency (model assumes 16 MHz), VOS/flash-latency limits on the
achievable `sys_ck`, and whether the PLL physically locks — all HW-only.

### 6. Peripheral breadth
We modelled only what we use. **Timers, I2C, DAC, RNG, advanced-timer features**
ride Renode's generic models or bare tags, not RM0433-verified.

## Inherent vs closable

- **Inherent to functional emulation** (accept; cover on HW): §1 cache, §2 timing,
  §3 analog. No Renode work removes these — they *define* what the hardware tests
  must own.
- **Closable in Renode if we choose:** §4 (a USB/IP bridge to a real host stack),
  §6 (RM-faithful models for peripherals we actually use). The highest-value one —
  a **functional cache-coherency *checker*** — is now **built** (see §1): it
  catches §1's D-cache/DMA *and* I-cache bug classes **without** a cycle-accurate
  core. What remains inherent is verifying a *specific* buffer's real cacheability
  on HW; the checker proves the maintenance *discipline*, not the MPU attributes.

## Practical consequence

CI (Renode) gates **logic/register/clock/DMA-wiring** regressions. A separate
**hardware test tier** (probe-rs + `dsp-bench` and friends) owns **timing,
coherency, and analog**. A feature is only "verified" when the sim tier proves
its logic *and* the HW tier proves its timing/coherency — neither alone suffices.
