*** Settings ***
Documentation    Reproduce the CURRENT hardware failure: the controller is
...              configured for the CORRECT 0xEB memory-mapped shape
...              (ABMODE=11 + 8-bit alternate byte 0xA0 + DCYC=6 + SIOO=1),
...              but the flash is left at its power-on default Read
...              Parameters (P4:P3=00 → 6 dummy cycles for 0xEB, datasheet
...              Table 6.10) instead of being programmed to 8 cycles via
...              0xC0 Set Read Parameters.
...
...              The controller clocks 8 pre-data cycles (alternate byte = 2
...              + DCYC 6); the flash, expecting only 6, starts driving data
...              one quad byte early. Every memory-mapped fetch is therefore
...              shifted by one byte. This is the exact mismatch that made
...              the bootloader→app XIP jump fail on real silicon while the
...              old Renode model (which mis-derived the flash's dummy count)
...              passed. libDaisy avoids it by issuing 0xC0 → 0xF0; the fix
...              in daisy-boot's qspi.rs now does the same, and the firmware
...              boot tests (boot_blink, bootloader_jump) cover that GOOD
...              path end-to-end. This test pins the NEGATIVE case so the
...              model keeps catching a regression that drops the 0xC0 step.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${QSPI_CR}       0x52005000
${QSPI_DCR}      0x52005004
${QSPI_DLR}      0x52005010
${QSPI_ABR}      0x5200501C
${QSPI_CCR}      0x52005014
${QSPI_DR}       0x52005020

*** Test Cases ***
Flash Left At Default Dummy Cycles Shifts XIP Data
    Execute Command    mach create "qspi-dummy-mismatch"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs

    Execute Command    sysbus WriteDoubleWord 0x90000000 0xDEADBEEF
    Execute Command    sysbus WriteDoubleWord 0x90000004 0xCAFEBABE
    Execute Command    sysbus WriteDoubleWord 0x90000008 0x12345678
    Execute Command    sysbus WriteDoubleWord 0x9000000C 0x9ABCDEF0

    Execute Command    sysbus WriteDoubleWord ${QSPI_CR} 0x00000001
    Execute Command    sysbus WriteDoubleWord ${QSPI_DCR} 0x00160000

    # QE must be set before any 0xEB quad read (datasheet §8.7). The flash's
    # dummy-cycle Read Parameter is deliberately left at its default here.
    Set Flash Quad Enable

    # Alternate byte = 0xA0 (AX mode bits), and the CORRECT memory-mapped
    # CCR: INSTRUCTION=0xEB, IMODE=01, ADMODE=11, ADSIZE=10, ABMODE=11,
    # ABSIZE=00, DCYC=6, DMODE=11, FMODE=11, SIOO=1 = 0x1F18EDEB.
    # NOTE: no 0xC0 Set Read Parameters is issued, so the flash stays at its
    # 6-cycle default while the controller drives 8 → one-byte shift.
    Execute Command    sysbus WriteDoubleWord ${QSPI_ABR} 0x000000A0
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x1F18EDEB
    Execute Command    emulation RunFor "00:00:00.001"

    # word0 is shifted by one byte, so it no longer reads 0xDEADBEEF.
    ${word0}=    Execute Command    sysbus ReadDoubleWord 0x90000000
    Log    Word at 0x90000000 with flash at default dummy config: ${word0.strip()}
    Should Not Contain    ${word0}    0xDEADBEEF

Set Read Parameters 0xF0 Makes XIP Byte Exact
    [Documentation]    The POSITIVE case (the fix): issuing 0xC0 Set Read
    ...                Parameters = 0xF0 raises the flash to 8 dummy cycles so
    ...                its 3 post-mode quad dummy bytes match the controller's
    ...                DCYC=6 (3 bytes). Every XIP word then reads back EXACTLY,
    ...                across multiple continuation fetches. This pins the good
    ...                path so a regression that programs the wrong Read
    ...                Parameters (or drops the 0xC0 step) is caught here, not on
    ...                silicon. Exercises the SRP data path — which was itself
    ...                masked by a FIFO-flush bug until the controller was fixed.
    Execute Command    mach create "qspi-dummy-ok"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs

    Execute Command    sysbus WriteDoubleWord 0x90000000 0xDEADBEEF
    Execute Command    sysbus WriteDoubleWord 0x90000004 0xCAFEBABE
    Execute Command    sysbus WriteDoubleWord 0x90000008 0x12345678
    Execute Command    sysbus WriteDoubleWord 0x9000000C 0x9ABCDEF0

    Execute Command    sysbus WriteDoubleWord ${QSPI_CR} 0x00000001
    Execute Command    sysbus WriteDoubleWord ${QSPI_DCR} 0x00160000
    Set Flash Quad Enable

    # 0xC0 Set Read Parameters = 0xF0 (P4:P3=10 → 8 dummy cycles), single-line.
    Execute Command    sysbus WriteDoubleWord ${QSPI_DLR} 0x00000000
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x010001C0
    Execute Command    sysbus WriteDoubleWord ${QSPI_DR} 0x000000F0

    Execute Command    sysbus WriteDoubleWord ${QSPI_ABR} 0x000000A0
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x1F18EDEB
    Execute Command    emulation RunFor "00:00:00.001"

    ${w0}=    Execute Command    sysbus ReadDoubleWord 0x90000000
    Log    word0 (aligned): ${w0.strip()}
    Should Contain    ${w0}    0xDEADBEEF
    ${w1}=    Execute Command    sysbus ReadDoubleWord 0x90000004
    Should Contain    ${w1}    0xCAFEBABE
    ${w2}=    Execute Command    sysbus ReadDoubleWord 0x90000008
    Should Contain    ${w2}    0x12345678
    ${w3}=    Execute Command    sysbus ReadDoubleWord 0x9000000C
    Should Contain    ${w3}    0x9ABCDEF0
