# -*- coding: utf-8 -*-
# STM32H7 embedded-flash controller stub for Renode.
#
# Applicability
# -------------
# Written for STM32H742/H743/H745/H747/H750/H753/H755/H757 (RM0433 / RM0399).
# The STM32H723/H725/H730/H735 sub-family (RM0468) is single-bank so SR2
# is absent — reading offset 0x50 there would fall through to the sticky
# default (0), which happens to be the same "not busy, no error" answer
# this stub forces. So this stub is safe for either family; a bank-count
# parameter isn't needed.
#
# Register map — RM0433 rev8 §4.9 / stm32h7-0.15.1 PAC stm32h743/flash.rs:
#
#   0x00  ACR      — access control (LATENCY, WRHIGHFREQ)   (§4.9.2)
#   0x04  KEYR1    — bank 1 key
#   0x08  OPTKEYR  — option key
#   0x0C  CR1      — bank 1 control
#   0x10  SR1      — bank 1 status (BSY, QW, etc.)          (§4.9.4)
#   0x14  CCR1     — bank 1 clear-flags
#   0x18  OPTCR
#   0x1C  OPTSR_CUR / 0x20 OPTSR_PRG / 0x24 OPTCCR
#   0x100 ACR      — (H7 has two banks but ACR is per-bank
#                    on H743 and shared on H750; we treat it
#                    as an alias of 0x000 for readback.)
#   0x104..0x120 bank 2 KEYR2/CR2/SR2/CCR2/…
#
# HAL calls (stm32h7xx-hal-0.16):
#   * `flash.acr.modify(|_, w| w.latency().bits(N).wrhighfreq().bits(M))`
#     followed by readback of the same field.
#   * Some paths poll SR1.BSY (bit 0) / SR2.BSY. For OUR bootloader,
#     nothing programs flash, but Renode's real STM32H7_FlashController
#     doesn't report LATENCY sticking, so the readback verify hangs.
#
# This stub gives sticky read/write for every offset (so LATENCY sticks)
# and forces both SR1 and SR2 to report "idle, no error" (returns 0)
# for verify loops.

try:
    _flash
except NameError:
    _flash = {}

_SR1 = 0x010
_SR2 = 0x110  # RM0433 §4.9.13: SR2 = bank-2 offset (0x100) + local 0x10

if request.IsRead:
    off = request.Offset
    if off == _SR1 or off == _SR2:
        request.Value = 0  # BSY=0, QW=0, no error flags
    else:
        request.Value = _flash.get(off, 0)

elif request.IsWrite:
    _flash[request.Offset] = request.Value
