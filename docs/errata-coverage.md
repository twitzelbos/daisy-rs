# ES0392 errata coverage — STM32H750

A deliberate cross-reference of the STM32H750 device errata (**ES0392 Rev 15**)
against this firmware and its Renode models. The question this answers: *for
every published silicon limitation that touches a peripheral we actually use, do
we have the workaround — and can a regression be caught?*

Silicon revisions in play (ES0392 Table 2): **rev Y/W** = `REV_ID 0x1003`,
**rev X** = `0x2001`, **rev V** = `0x2003`. The dev unit is rev V; production
Daisy Seeds in the field span older revisions, so a rev-V-only fix is not
sufficient on its own.

Erratum categories: **A** = no reasonable workaround / always-on hazard, **P** =
partial, **N** = no functional impact. "Cat" below is the worst applicable
revision's status.

---

## Handled — workaround present and verified

| § | Erratum | Cat | Where we handle it |
|---|---|---|---|
| **2.2.10** | Reading from AXI SRAM may lead to data read corruption (rev Y/W/X; fixed on V) | A | `daisy-boot/src/main.rs` sets `AXI_TARG7_FN_MOD` (`0x5100_8108`) READ_ISS_OVERRIDE, **gated to `REV_ID < 0x2000`**, mirroring libDaisy `system_stm32h7xx.c`. We use AXI SRAM (`0x2400_0000`) for every TUI heap, so this one is load-bearing. Regression-guarded by `renode/errata_workarounds.robot`. |
| **2.7.4** | QUADSPI internal timing criticality | A | `daisy-boot`'s `qspi_errata_2_7_4_workaround()` — the CR/CCR free-running-clock dance — invoked from `exit_memory_mapped` before every ABORT. |
| **2.2.21** | 480 MHz max CPU frequency not available on rev Y/W | P | `clocks::init` gates 480 MHz / VOS0 on `(idcode>>16) >= 0x2003` (rev V). This *is* the erratum's workaround ("use rev V or X"). Renode-checked by `clocks_boot.robot` (rev V → 480, rev Y → 400). |
| **2.6.6 / 2.6.7** | SDRAM TRCD/TXSR/TWR timing (documentation errata) | doc | `daisy-bsp/src/sdram.rs` chooses timings so `TWR < TRCD` and `< TXSR`, side-stepping the delay-substitution described. |

## Verified not applicable (checked, ruled out)

| § | Erratum | Why it doesn't bite us |
|---|---|---|
| **2.1.1** | Data corruption with **write-through** D-cache | Only write-through **stores** trigger it. Our sole write-through MPU region is **QSPI XIP (read-only code)** — the CPU never stores there. Every *writable* region is non-cacheable (SRAM_D2, Backup SRAM) or write-**back** (SDRAM), which is Arm's recommended mitigation. Audited against `daisy-app-template/src/main.rs::configure_mpu_and_caches`. |
| **2.7.6** | QUADSPI failure using HCLK kernel clock with `PRESCALER=0` | We run **prescaler = 7** (÷8), so the 50 %-duty condition is satisfied. |
| **2.9.x** | ADC injected / dual-interleaved conversion hazards | Hothouse uses single **regular** conversions on ADC1; no injected/dual modes. |
| **2.5.x** | DMAMUX synchronization / request-generator | Our DMA is peripheral-triggered / mem-to-mem, never sync or request-gen mode. |
| **2.2.17 / 2.2.24** | Backup-SRAM level-regression stall / tamper-erase-when-clock-gated | We keep `BKPRAMEN` on and never do a runtime RDP level regression or use tamper. |
| **2.2.3** | CRS synchronization with USB SOF does not work | We clock from the HSE crystal, not CRS. |
| **2.19.x** | I2C Stop-mode / ADDRCF | The WM8731 path is a plain 400 kHz master write; no Stop-mode wakeup. |

## Latent traps — noted, low probability, no action yet

| § | Erratum | Exposure |
|---|---|---|
| **2.1.5** | Store after cache-invalidate needs an intervening DMB | `cortex_m`'s `SCB::{clean,invalidate}_dcache_by_address` already emit `dsb`+`isb`, so idiomatic use is safe. **Design note for the DMA cache-coherency exerciser** (hardware-tests §6): any hand-rolled DMA-master + cache-maintenance sequence must keep the barrier between the maintenance op and the following store. |
| **2.7.5** | Memory-mapped read of the *last* FSIZE byte returns 0x00 and a repeat read stalls AXI | We set `FSIZE = 22` (exactly 8 MiB). Only reachable if an app image's rodata reaches `0x907F_FFFF`; images are far smaller. A silent AXI stall if it ever happens — keep app images clear of the top byte. |
| **2.2.13** | USB OTG_FS PHY drive limited (≤ 5 mA on DP/DM) | Board/electrical; no firmware action. Noted for hardware bring-up. |
| **2.2.21** (rev X) | — | Our 480 MHz gate (`>= 0x2003`) conservatively excludes **rev X** (`0x2001`), which the erratum says *also* reaches 480 MHz. Deliberate: rev X still carries the 2.2.10 AXI and 2.2.17 hazards that rev V fixes, so we leave rev X at 400. Not a bug — a conservative choice. |

---

## The Renode angle

None of these errata are *modellable* — they are analog, timing, or
multi-master-silicon behaviour (see [renode-fidelity.md](renode-fidelity.md)
§2 timing, §5 electrical). But **2.2.10 is the same failure class as the
SDNWE=PH5 pin bug and the FMC `BCR1.FMCEN` gate**: a required register poke that
passes silently in sim if omitted, because Renode doesn't model the underlying
hazard. The mitigation for that class is not a behavioural model but a
**presence assertion**: `renode/errata_workarounds.robot` boots the real
bootloader and checks that `AXI_TARG7_FN_MOD` was written on pre-rev-V silicon
and left untouched on rev V — turning a silent omission into a red test, exactly
as `qspi_ncs_pinmux.robot` does for the XIP NCS pin-mux.
