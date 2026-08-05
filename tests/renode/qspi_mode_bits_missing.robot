*** Settings ***
Documentation    Reproduce the hardware failure we hit before the mode-bits
...              fix: 0xEB Fast Read Quad I/O configured in memory-mapped
...              mode with the alternate-bytes phase disabled (ABMODE=0).
...              Per IS25LP064A datasheet §8.7 this is invalid — real
...              silicon returns corrupted data / hangs instruction fetches.
...
...              We verify this in Renode by:
...                1. Loading a known 32-bit pattern at 0x9000_0000 (via
...                   sysbus, which goes directly to MappedMemory).
...                2. Configuring the QSPI controller in memory-mapped
...                   mode with a broken 0xEB config (ABMODE=0). Our
...                   STM32H7_QuadSPI_Fixed's protocol-validation blit
...                   runs on every CCR write with FMODE=11: it drives
...                   the SPI protocol against the IS25LP064A model for
...                   the first N bytes, then writes the protocol output
...                   back to MappedMemory. If the config is broken,
...                   phases misalign in the flash chip's state machine
...                   and the protocol-returned bytes diverge from the
...                   original.
...                3. Reading the same 32-bit word back and asserting
...                   it no longer matches the original pattern —
...                   proof the SPI protocol detected the config bug.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${QSPI_BASE}     0x52005000
# QSPI register offsets (from RM0433 §23.6).
${QSPI_CR}       0x52005000
${QSPI_DLR}      0x52005010
${QSPI_CCR}      0x52005014
${QSPI_AR}       0x52005018

*** Test Cases ***
Broken Enter Memory Mapped Without Mode Bits Corrupts XIP Data
    Execute Command    mach create "qspi-broken"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs

    # Prime the flash MappedMemory with a distinctive pattern the CPU
    # would fetch from at 0x9000_0000. If the SPI protocol is driven
    # correctly, our protocol-validation blit will re-write the same
    # bytes here (net no change). If ABMODE=0 makes the chip mis-parse
    # phases, the blit corrupts the pattern.
    Execute Command    sysbus WriteDoubleWord 0x90000000 0xDEADBEEF
    Execute Command    sysbus WriteDoubleWord 0x90000004 0xCAFEBABE
    Execute Command    sysbus WriteDoubleWord 0x90000008 0x12345678
    Execute Command    sysbus WriteDoubleWord 0x9000000C 0x9ABCDEF0

    # Enable the QSPI peripheral so writes to CCR are actually processed.
    # CR bit 0 = EN.
    Execute Command    sysbus WriteDoubleWord ${QSPI_CR} 0x00000001

    # Set the flash size in DCR (flashSize field). Not strictly required
    # for our test but keeps the controller state realistic.
    Execute Command    sysbus WriteDoubleWord 0x52005004 0x00160000

    # Write CCR to configure 0xEB Fast Read Quad I/O in memory-mapped
    # mode WITHOUT the alternate-bytes phase. Layout:
    #   INSTRUCTION[7:0]  = 0xEB
    #   IMODE[9:8]        = 01  (1 line)
    #   ADMODE[11:10]     = 11  (4 lines)
    #   ADSIZE[13:12]     = 10  (24-bit)
    #   ABMODE[15:14]     = 00  ← BROKEN — should be 11 (quad)
    #   ABSIZE[17:16]     = 00  (irrelevant when ABMODE=0)
    #   DCYC[22:18]       = 00110 = 6
    #   DMODE[25:24]      = 11  (4 lines, data)
    #   FMODE[27:26]      = 11  (memory-mapped)
    #   SIOO[28]          = 0
    # OR'd: 0xEB | 0x100 | 0xC00 | 0x2000 | 0x180000 | 0x3000000 | 0xC000000
    #     = 0x0F182DEB
    #
    # Writing this triggers ProtocolValidateMemoryMappedMode in the
    # patched controller. With ABMODE=0, the chip's ModeBits phase
    # consumes the byte our controller intended as a dummy, shifting
    # every subsequent data byte by one position. The first N bytes at
    # 0x9000_0000 get overwritten with the shifted content.
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x0F182DEB

    # Give Renode a moment to complete the blit synchronously.
    Execute Command    emulation RunFor "00:00:00.001"

    # Read back the first word. With correct config the blit is a no-op
    # and we still see 0xDEADBEEF. With ABMODE=0 the SPI-protocol output
    # is off-by-one, so byte 0 (the "dummy" consumed as mode bits)
    # returns 0xFF-then-original-bytes, giving a byte sequence like
    # 0xFF/0xEF/0xBE/0xAD — NOT 0xDEADBEEF.
    ${word0}=    Execute Command    sysbus ReadDoubleWord 0x90000000
    Log    Word at 0x90000000 after broken CCR write: ${word0.strip()}
    Should Not Contain    ${word0}    0xDEADBEEF
