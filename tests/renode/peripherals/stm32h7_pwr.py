# -*- coding: utf-8 -*-
# STM32H7 PWR stub for Renode.
#
# Applicability
# -------------
# Written for the STM32H742/H743/H745/H747/H750/H753/H755/H757 sub-family,
# whose PWR register map is documented in RM0433 (single-core) and RM0399
# (dual-core) — both are identical on the PWR block.
#
# The STM32H723/H725/H730/H733/H735 sub-family (RM0468) has additional
# registers (SR1 changed to STOPF/SBF via WKUPCR extensions) but the
# subset this stub implements (CR1.DBP, CSR1.ACTVOSRDY, D3CR.VOSRDY)
# has the same offsets and bit positions there. So this stub is safe
# for either sub-family; a future H7B3/H7B0 or dual-M4-core parameter
# split can add per-family branches with `_family = "..."` at the top.
#
# Register map — RM0433 rev8 §7.5 / stm32h7-0.15.1 PAC stm32h743/pwr.rs:
#
#   0x00  CR1   — power control 1                (RM0433 §7.5.1)
#   0x04  CSR1  — power control status 1         (RM0433 §7.5.2)
#   0x08  CR2   — power control 2                (RM0433 §7.5.3)
#   0x0C  CR3   — power control 3                (RM0433 §7.5.4)
#   0x10  CPUCR — CPU power control              (RM0433 §7.5.5)
#   0x18  D3CR  — D3 domain control              (RM0433 §7.5.9)
#   0x20  WKUPCR / 0x24 WKUPFR / 0x28 WKUPEPR    (WKUP pins)
#
# HAL calls (stm32h7xx-hal-0.16 src/pwr.rs :: freeze()) — bit positions
# taken from RM0433 rev8 (authoritative); PAC cross-check in comments:
#   * writes CR3.LDOEN | CR3.SCUEN            (RM0433 §7.5.4 bits 1-2)
#   * writes D3CR.VOS[15:14]                  (RM0433 §7.5.9)
#   * polls D3CR.VOSRDY  (bit 13)             (RM0433 §7.5.9)
#   * polls CSR1.ACTVOSRDY (bit 13)           (RM0433 §7.5.2)
#   * writes CR1.DBP     (bit 8) and reads back to verify
#
# daisy-boot itself:
#   * writes CR3.USB33DEN (bit 24)            (RM0433 §7.5.4)
#   * polls CR3.USB33RDY  (bit 26)            (RM0433 §7.5.4)
#
# This stub gives sticky read/write for every offset AND forces the ready
# bits above to read as set as soon as the corresponding enable is written.

try:
    _pwr
except NameError:
    _pwr = {}

_CR1  = 0x00
_CSR1 = 0x04
_CR3  = 0x0C
_D3CR = 0x18

if request.IsRead:
    off = request.Offset
    v = _pwr.get(off, 0)
    if off == _CSR1:
        # bit 13 ACTVOSRDY — polled by pwr.freeze() after CR3 config.
        v |= (1 << 13)
        # bits 15:14 ACTVOS mirror D3CR.VOS so verification-loop reads
        # of "currently active VOS" match what firmware requested.
        vos = (_pwr.get(_D3CR, 0) >> 14) & 0x3
        v = (v & ~(0x3 << 14)) | (vos << 14)
    elif off == _CR3:
        # bit 24 USB33DEN → bit 26 USB33RDY (RM0433 §7.5.4).
        # daisy-boot::main polls USB33RDY after setting USB33DEN so the
        # external USB 3.3 V supply is confirmed before enabling OTG_HS.
        if v & (1 << 24):
            v |= (1 << 26)
    elif off == _D3CR:
        # bit 13 VOSRDY — polled by pwr.freeze() after VOS write.
        v |= (1 << 13)
    request.Value = v

elif request.IsWrite:
    _pwr[request.Offset] = request.Value
