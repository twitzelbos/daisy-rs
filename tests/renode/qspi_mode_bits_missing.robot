*** Settings ***
Documentation    Reproduce the ORIGINAL hardware failure: 0xEB Fast Read
...              Quad I/O configured in memory-mapped mode with SIOO=1
...              (send instruction only once) but the alternate-bytes /
...              mode-bits phase disabled (ABMODE=0).
...
...              Per IS25LP064A datasheet §8.7, AX Continuous Read Mode is
...              entered only when the mode bits M[7:4] == 1010b (0xAx) are
...              driven during the mode-bits window. With ABMODE=0 the STM32
...              QUADSPI never drives those bits, so the flash does NOT enter
...              AX mode. SIOO=1 then tells the controller to skip the
...              instruction byte on the 2nd and later reads — but a flash
...              that isn't in AX mode still expects the instruction, so it
...              misparses the first address byte as a command. Result: the
...              FIRST XIP fetch reads correctly (instruction was sent), but
...              every CONTINUATION fetch returns garbage. That is exactly
...              the silent-on-hardware failure this config caused.
...
...              The datasheet-accurate IS25LP064A_Fixed model reproduces
...              this per-fetch: with the mode byte absent the flash stays
...              out of AX mode, and the SIOO-suppressed instruction on the
...              second read desyncs the state machine.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${QSPI_CR}       0x52005000
${QSPI_DCR}      0x52005004
${QSPI_CCR}      0x52005014

*** Test Cases ***
Missing Mode Bits With SIOO Corrupts Continuation Fetches
    Execute Command    mach create "qspi-broken"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs

    # Prime four distinctive words the CPU would fetch from XIP.
    Execute Command    sysbus WriteDoubleWord 0x90000000 0xDEADBEEF
    Execute Command    sysbus WriteDoubleWord 0x90000004 0xCAFEBABE
    Execute Command    sysbus WriteDoubleWord 0x90000008 0x12345678
    Execute Command    sysbus WriteDoubleWord 0x9000000C 0x9ABCDEF0

    # Enable the peripheral (CR.EN) and set an 8 MiB flash size in DCR.
    Execute Command    sysbus WriteDoubleWord ${QSPI_CR} 0x00000001
    Execute Command    sysbus WriteDoubleWord ${QSPI_DCR} 0x00160000

    # QE must be set before any 0xEB quad read (datasheet §8.7).
    Set Flash Quad Enable

    # CCR: 0xEB memory-mapped, ADMODE=11, ADSIZE=10, DCYC=6, DMODE=11,
    # FMODE=11 — but ABMODE=00 (BROKEN) and SIOO=1 (bit 28). Base value
    # 0x0F182DEB with SIOO set = 0x1F182DEB.
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x1F182DEB
    Execute Command    emulation RunFor "00:00:00.001"

    # First fetch: the instruction IS sent, the flash aligns (its default
    # 6-cycle dummy config matches the 6 pre-data cycles here since there's
    # no alternate-byte phase), so word0 reads back correctly.
    ${word0}=    Execute Command    sysbus ReadDoubleWord 0x90000000
    Log    Word at 0x90000000 (first fetch): ${word0.strip()}
    Should Contain    ${word0}    0xDEADBEEF

    # Continuation fetch: SIOO=1 skips the instruction, but the flash never
    # entered AX mode (no 0xA0 mode bits), so it misparses the address as a
    # command and returns garbage — the real hardware failure.
    ${word1}=    Execute Command    sysbus ReadDoubleWord 0x90000004
    Log    Word at 0x90000004 (continuation fetch): ${word1.strip()}
    Should Not Contain    ${word1}    0xCAFEBABE
