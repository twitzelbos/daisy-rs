# -*- coding: utf-8 -*-
# STM32H7 RCC stub for Renode.
#
# Applicability
# -------------
# Written for STM32H742/H743/H745/H747/H750/H753/H755/H757 (RM0433 / RM0399,
# identical on the RCC block). The STM32H723/H725/H730/H735 sub-family
# (RM0468) has a very similar RCC layout — CR/CFGR/BDCR/CSR are at the
# same offsets and use the same enable→ready pairing — but adds PLL1/2/3
# fractional-latch flags in different places. Those aren't touched by the
# stm32h7xx-hal 0.16 clock-init path, so this stub is safe for either
# sub-family. If we ever move to H7A3/H7B0/H7B3 (RM0455), the register
# layout changes significantly and this stub should be forked with a
# `_family` selector at the top.
#
# Register map — RM0433 rev8 §8.7 / stm32h7-0.15.1 PAC stm32h743/rcc.rs:
#
#   0x00  CR       — clock control                 (RM0433 §8.7.2)
#   0x04  HSICFGR                                  (§8.7.3)
#   0x08  CRRCR    — clock recovery RC             (§8.7.4)
#   0x0C  CSICFGR                                  (§8.7.5)
#   0x10  CFGR     — clock configuration           (§8.7.6)
#   0x18  D1CFGR                                   (§8.7.7)
#   0x1C  D2CFGR                                   (§8.7.8)
#   0x20  D3CFGR                                   (§8.7.9)
#   0x28  PLLCKSELR / 0x2C PLLCFGR                 (§8.7.11 / §8.7.12)
#   0x30..0x44   PLL[123]DIVR / PLL[123]FRACR      (§8.7.13-8.7.18)
#   0x4C..0x58   D[123]CCIP[1..]R                  (§8.7.19-8.7.22)
#   0x60..0x68   CIER / CIFR / CICR                (§8.7.23-8.7.25)
#   0x70  BDCR    — Backup Domain control          (§8.7.30)
#   0x74  CSR     — Clock Control and Status       (§8.7.31)
#   0x7C  AHB3RSTR / 0x80 AHB1RSTR / …             (peripheral reset regs)
#
# HAL polls (stm32h7xx-hal-0.16 src/rcc/mod.rs :: freeze()):
#   * `while rcc.csr.read().lsirdy().is_not_ready() {}` — CSR bit 1
#   * `while rcc.cr.read().pll1rdy().is_not_ready() {}` — CR bit 25 etc.
#   * SW→SWS switch: read CFGR bit 5:3 for currently-active source
#
# This stub gives sticky read/write for every offset AND mirrors every
# enable→ready pair the HAL polls on:
#
#   CR    bit 0  HSION    → bit 2  HSIRDY
#         bit 7  CSION    → bit 8  CSIRDY
#         bit 12 HSI48ON  → bit 13 HSI48RDY
#         bit 16 HSEON    → bit 17 HSERDY
#         bit 24 PLL1ON   → bit 25 PLL1RDY
#         bit 26 PLL2ON   → bit 27 PLL2RDY
#         bit 28 PLL3ON   → bit 29 PLL3RDY
#   CFGR  bits 2:0 SW     → bits 5:3 SWS
#   BDCR  bit 0  LSEON    → bit 1  LSERDY
#   CSR   bit 0  LSION    → bit 1  LSIRDY
#
# All bit positions verified against stm32h7-0.15.1 PAC field
# definitions (which are SVD-derived from ST's silicon description).

try:
    _rcc
except NameError:
    _rcc = {}
    # RCC_CR reset value per RM0433 §8.7.2: HSION|HSIRDY|HSIDIVF (bits 0, 2, 5).
    _rcc[0x000] = 0x00000025

_CR   = 0x000
_CFGR = 0x010
_BDCR = 0x070
_CSR  = 0x074

_CR_ON_TO_RDY = {
    0:  2,   # HSI     → HSIRDY
    7:  8,   # CSI     → CSIRDY
    12: 13,  # HSI48   → HSI48RDY
    16: 17,  # HSE     → HSERDY
    24: 25,  # PLL1    → PLL1RDY
    26: 27,  # PLL2    → PLL2RDY
    28: 29,  # PLL3    → PLL3RDY
}

if request.IsRead:
    off = request.Offset
    v = _rcc.get(off, 0)

    if off == _CR:
        for on, rdy in _CR_ON_TO_RDY.items():
            if v & (1 << on):
                v |= (1 << rdy)
            else:
                v &= ~(1 << rdy)
        v |= (1 << 2)  # HSI is always ready in sim.

    elif off == _CFGR:
        # SWS[5:3] reflects SW[2:0] — clock switch takes effect immediately.
        sw = v & 0x7
        v = (v & ~(0x7 << 3)) | (sw << 3)

    elif off == _BDCR:
        # LSEON (bit 0) → LSERDY (bit 1). Reset value has all bits clear.
        if v & 0x1:
            v |= (1 << 1)

    elif off == _CSR:
        # LSION (bit 0) → LSIRDY (bit 1). Polled by rcc.freeze() during
        # clock init even when no user code actually enables the LSI, so
        # this poll is on the boot critical path.
        if v & 0x1:
            v |= (1 << 1)

    request.Value = v

elif request.IsWrite:
    _rcc[request.Offset] = request.Value
