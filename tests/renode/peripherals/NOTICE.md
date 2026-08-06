# STM32H7 peripheral stubs — attribution and scope

The Python peripheral stubs in this directory (`stm32h7_pwr.py`, `stm32h7_rcc.py`,
`stm32h7_flash.py`, `stm32h7_syscfg.py`) unblock Renode simulation for the
STM32H750-based Daisy Seed. Renode 1.16 ships stubs or generic-STM32 models
for these peripherals that omit the ready-bit semantics stm32h7xx-hal polls
during `clocks::init`, so the firmware would otherwise hang.

The idea and the choice of *which bits to force* were inspired by
[magnusarinell/renode-visual-console](https://github.com/magnusarinell/renode-visual-console),
which demonstrates that a small set of register-level stubs is enough to boot
unmodified libDaisy applications on a simulated STM32H750. That upstream
repository has no LICENSE file, so this directory does **not** vendor its
source.

**Authority order.** Every offset and bit position in these stubs is derived
from **RM0433 rev8**, the ST reference manual for STM32H742/H743/H750/H753.
The PAC (`stm32h7-0.15.1`) and HAL (`stm32h7xx-hal-0.16`) are cited only as
secondary references — they're SVD-derived and human-authored respectively,
and both can (and do) contain bugs relative to the silicon. If any conflict
arises between RM0433 and PAC/HAL, RM0433 wins.

## H7 sub-family applicability

| Sub-family | Reference manual | Applicability of these stubs |
| --- | --- | --- |
| STM32H742/H743/H750/H753 (single-core) | RM0433 | **Direct target — verified.** |
| STM32H745/H747/H755/H757 (dual M7 + M4) | RM0399 | Same PWR/RCC/FLASH/SYSCFG core layout as RM0433; stubs work as-is for the M7. |
| STM32H723/H725/H730/H733/H735 | RM0468 | Same offsets for the subset each stub touches (verified per-stub in each file). Safe to use. |
| STM32H7A3/H7B0/H7B3 | RM0455 | **Not applicable — RCC layout differs; do not use.** Fork with a `_family` selector at the top if needed. |

If a Renode maintainer later ships proper `STM32H7_PWR` / `STM32H7_FLASH`
peripheral models, delete this directory and switch back to the stock
platform.
