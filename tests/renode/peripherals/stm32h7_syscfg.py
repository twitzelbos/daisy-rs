# -*- coding: utf-8 -*-
# STM32H7 SYSCFG stub for Renode.
#
# Applicability
# -------------
# Written for STM32H742/H743/H745/H747/H750/H753/H755/H757 (RM0433 / RM0399).
# The STM32H723/H725/H730/H735 sub-family (RM0468) shares the same core
# register layout for the offsets this stub cares about (PMCR, CCCSR,
# PWRCR); H7A3/H7B0/H7B3 (RM0455) diverge enough that this stub should
# be forked if we ever target that family.
#
# Register map — RM0433 rev8 §11.3 / stm32h7-0.15.1 PAC stm32h743/syscfg.rs:
#
#   0x04  PMCR    — peripheral mode config              (RM0433 §11.3.1)
#   0x08  EXTICR1..0x14 EXTICR4                         (§11.3.2)
#   0x20  CCCSR   — compensation cell control/status    (§11.3.4)
#   0x24  CCVR    — compensation cell value  (RO)       (§11.3.5)
#   0x28  CCCR    — compensation cell code               (§11.3.6)
#   0x2C  PWRCR   — SYSCFG power control (VOS0 boost)   (§11.3.7)
#
# HAL polls (stm32h7xx-hal-0.16):
#   * enable_overdrive() writes PWRCR.ODEN (bit 0) — no ready poll in HAL
#     but Renode returning zero here breaks readback-verify loops.
#   * rcc/mod.rs freeze() writes CCCSR.EN and polls CCCSR.READY (bit 8)
#
# This stub gives sticky read/write for every offset AND forces
# CCCSR.READY to reflect CCCSR.EN so the compensation-cell wait loop
# exits immediately.

try:
    _syscfg
except NameError:
    _syscfg = {}

_CCCSR = 0x20
_PWRCR = 0x2C

if request.IsRead:
    off = request.Offset
    v = _syscfg.get(off, 0)
    if off == _CCCSR:
        # bit 0 EN → bit 8 READY (RM0433 §11.3.4)
        if v & 0x1:
            v |= (1 << 8)
    request.Value = v

elif request.IsWrite:
    _syscfg[request.Offset] = request.Value
