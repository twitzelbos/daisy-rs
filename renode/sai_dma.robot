*** Settings ***
Documentation    End-to-end test of the SAI + circular-DMA models with NO
...              firmware: program the DMAMUX + two DMA1 streams + SAI1 by
...              register pokes exactly as the HAL audio path would, run for a
...              few frames, and assert the SAI drives the DMA (half + complete
...              IRQ flags fire), real data moves through the SAI data register
...              via the TX→RX loopback, and streaming re-asserts after the
...              flags are cleared (circular wrap). Proves the whole chain:
...              SAI DmaRequest.Blink → DMAMUX (DMAREQ_ID match) → DMA stream
...              copy → HTIF/TCIF → NVIC.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
# DMAMUX1 channel config registers (CxCR, stride 4). Channel c drives DMA1 stream c.
${DMAMUX_C0}     0x40020800
${DMAMUX_C1}     0x40020804
# DMA1 stream register bases (SxCR/NDTR/PAR/M0AR at +0x10/+0x14/+0x18/+0x1C, stride 0x18).
${S0_CR}         0x40020010
${S0_NDTR}       0x40020014
${S0_PAR}        0x40020018
${S0_M0AR}       0x4002001C
${S1_CR}         0x40020028
${S1_NDTR}       0x4002002C
${S1_PAR}        0x40020030
${S1_M0AR}       0x40020034
${DMA_LISR}      0x40020000
${DMA_LIFCR}     0x40020008
# SAI1 data registers + control.
${SAI_ADR}       0x40015820
${SAI_BDR}       0x40015840
${SAI_ACR1}      0x40015804
${SAI_BCR1}      0x40015824
# Buffers in AXI SRAM.
${TX_BUF}        0x24000000
${RX_BUF}        0x24001000

*** Keywords ***
Set Up SAI DMA Loopback
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision SAI And Circular DMA

    # Seed 8 distinct TX samples.
    Execute Command    sysbus WriteDoubleWord 0x24000000 0x11111111
    Execute Command    sysbus WriteDoubleWord 0x24000004 0x22222222
    Execute Command    sysbus WriteDoubleWord 0x24000008 0x33333333
    Execute Command    sysbus WriteDoubleWord 0x2400000C 0x44444444
    Execute Command    sysbus WriteDoubleWord 0x24000010 0x55555555
    Execute Command    sysbus WriteDoubleWord 0x24000014 0x66666666
    Execute Command    sysbus WriteDoubleWord 0x24000018 0x77777777
    Execute Command    sysbus WriteDoubleWord 0x2400001C 0x88888888

    # DMAMUX: channel 0 (DMA1 stream0 = TX) ← SAI1_B req 88; channel 1
    # (stream1 = RX) ← SAI1_A req 87.
    Execute Command    sysbus WriteDoubleWord ${DMAMUX_C0} 0x00000058
    Execute Command    sysbus WriteDoubleWord ${DMAMUX_C1} 0x00000057

    # Stream0 = TX (MemoryToPeripheral): PAR=B.DR, M0AR=TX_BUF, NDT=8,
    # CR = DIR=01 | PSIZE=Word | MSIZE=Word | MINC | CIRC | EN = 0x5541.
    Execute Command    sysbus WriteDoubleWord ${S0_PAR} ${SAI_BDR}
    Execute Command    sysbus WriteDoubleWord ${S0_M0AR} ${TX_BUF}
    Execute Command    sysbus WriteDoubleWord ${S0_NDTR} 0x00000008
    Execute Command    sysbus WriteDoubleWord ${S0_CR} 0x00005541

    # Stream1 = RX (PeripheralToMemory): PAR=A.DR, M0AR=RX_BUF, NDT=8,
    # CR = DIR=00 | PSIZE=Word | MSIZE=Word | MINC | CIRC | HTIE | TCIE | EN = 0x5519.
    Execute Command    sysbus WriteDoubleWord ${S1_PAR} ${SAI_ADR}
    Execute Command    sysbus WriteDoubleWord ${S1_M0AR} ${RX_BUF}
    Execute Command    sysbus WriteDoubleWord ${S1_NDTR} 0x00000008
    Execute Command    sysbus WriteDoubleWord ${S1_CR} 0x00005519

    # Enable SAI blocks (SAIEN|DMAEN) — starts the frame pacer.
    Execute Command    sysbus WriteDoubleWord ${SAI_BCR1} 0x00030000
    Execute Command    sysbus WriteDoubleWord ${SAI_ACR1} 0x00030000

Assert Bit Set
    [Arguments]    ${reg}    ${mask}    ${expected}
    ${raw}=    Execute Command    sysbus ReadDoubleWord ${reg}
    ${v}=    Evaluate    int("${raw.strip()}", 16) & ${mask}
    Should Be Equal As Integers    ${v}    ${expected}

*** Test Cases ***
SAI Drives DMA With Half And Complete Interrupts And Loopback Data
    Set Up SAI DMA Loopback
    Execute Command    emulation RunFor "00:00:00.005"

    # RX stream1 flags in LISR: HTIF1 = bit 10 (0x400), TCIF1 = bit 11 (0x800).
    Assert Bit Set    ${DMA_LISR}    0x400    1024
    Assert Bit Set    ${DMA_LISR}    0x800    2048

    # The loopback carried each played TX sample into the captured RX buffer.
    ${r0}=    Execute Command    sysbus ReadDoubleWord ${RX_BUF}
    Should Contain    ${r0}    0x11111111
    ${r4}=    Execute Command    sysbus ReadDoubleWord 0x24001004
    Should Contain    ${r4}    0x22222222
    ${r7}=    Execute Command    sysbus ReadDoubleWord 0x2400101C
    Should Contain    ${r7}    0x88888888

Circular Wrap Re-Asserts After Clearing Flags
    Set Up SAI DMA Loopback
    Execute Command    emulation RunFor "00:00:00.005"
    # Clear HTIF1 + TCIF1 (LIFCR bits 10/11).
    Execute Command    sysbus WriteDoubleWord ${DMA_LIFCR} 0x00000C00
    Assert Bit Set    ${DMA_LISR}    0xC00    0
    # Keep running: circular streaming must set them again.
    Execute Command    emulation RunFor "00:00:00.005"
    Assert Bit Set    ${DMA_LISR}    0x800    2048
