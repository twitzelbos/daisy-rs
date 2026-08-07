*** Settings ***
Documentation    Reproduce the hardware bug where the QSPI flash gets STUCK in
...              AX Continuous Read Mode and stays that way across a warm MCU
...              reset — so the bootloader inherits a wedged flash that ignores
...              every instruction-led command (including its own software
...              reset) until a continuous-EXIT frame is sent.
...
...              On silicon the flash is a separate power domain: an STM32 reset
...              does NOT reset the flash chip. The IS25LP064A_Fixed model now
...              preserves latched state (AX-continuous, read-parameters, QE,
...              deep-power-down) across Renode `machine Reset`, and only clears
...              it on PowerCycle() or the in-band software reset (0x66/0x99).
...
...              Observable, exactly as debugged over SWD: a single-line JEDEC
...              ID read (0x9F) returns 0x0017609D on a healthy flash, but
...              garbage while the flash is wedged in continuous mode.
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
${QSPI_CCR}      0x52005014
${QSPI_AR}       0x52005018
${QSPI_ABR}      0x5200501C
${QSPI_DR}       0x52005020
# JEDEC ID that a healthy IS25LP064A returns: 0x9D,0x60,0x17 packed LE.
${JEDEC_ID}      0x0017609D

*** Keywords ***
Provision QSPI Machine
    Execute Command    mach create "qspi-persist"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    # A recognisable word at flash offset 0 so a correct XIP read is checkable.
    Execute Command    sysbus WriteDoubleWord 0x90000000 0xDEADBEEF
    Execute Command    sysbus WriteDoubleWord ${QSPI_CR} 0x00000001
    Execute Command    sysbus WriteDoubleWord ${QSPI_DCR} 0x00160000
    Set Flash Quad Enable
    Align Flash Dummy Cycles

Align Flash Dummy Cycles
    [Documentation]    Set the flash Read Parameters to 0xF0 (8 dummy cycles),
    ...                as libDaisy does, so the flash's dummy count matches the
    ...                controller's DCYC=6 + 8-bit alt byte = 8 pre-data cycles.
    ...                Without this a continuous read is byte-shifted (that is a
    ...                different bug, covered by qspi_dummy_cycle_mismatch).
    Execute Command    sysbus WriteDoubleWord ${QSPI_DLR} 0x00000000
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x010001C0
    Execute Command    sysbus WriteDoubleWord ${QSPI_DR} 0x000000F0

Enter Continuous Read Mode
    [Documentation]    Configure libDaisy's exact memory-mapped 0xEB read
    ...                (ABR=0xA0 mode bits, SIOO=1) and do one XIP fetch, which
    ...                drives the flash into AX Continuous Read Mode.
    Execute Command    sysbus WriteDoubleWord ${QSPI_ABR} 0x000000A0
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x1F18EDEB
    Execute Command    emulation RunFor "00:00:00.001"
    ${w}=    Execute Command    sysbus ReadDoubleWord 0x90000000
    [Return]    ${w}

Read JEDEC Id
    [Documentation]    Indirect single-line 0x9F read of 3 bytes → QSPI_DR.
    Execute Command    sysbus WriteDoubleWord ${QSPI_DLR} 0x00000002
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x0500019F
    ${id}=    Execute Command    sysbus ReadDoubleWord ${QSPI_DR}
    [Return]    ${id}

Prime Backing Word
    [Documentation]    Force a flash word to all-zero via a raw backing write
    ...                (DirectWrite while the controller is not in memory-mapped
    ...                mode), so a subsequent erase to 0xFFFFFFFF is observable.
    [Arguments]    ${addr}
    Execute Command    sysbus WriteDoubleWord ${addr} 0x00000000

Attempt Sector Erase
    [Documentation]    WREN + 4 KiB Sector Erase (0x20) at flash offset ${off},
    ...                single-line — the same command sequence the DFU service
    ...                path issues (qspi::erase_sector_4k: CCR then AR, no data
    ...                phase). While the flash is wedged in continuous mode these
    ...                single-line opcodes are misframed and the erase is
    ...                dropped; from a clean single-line state it erases the
    ...                sector to 0xFF.
    [Arguments]    ${off}
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x00000106
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x00002520
    Execute Command    sysbus WriteDoubleWord ${QSPI_AR} ${off}

Read Backing Word
    [Documentation]    Read the raw backing store (DirectRead) by first taking
    ...                the controller out of memory-mapped mode, so the value
    ...                reflects what actually landed in flash, not a protocol read.
    [Arguments]    ${addr}
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x00000000
    ${v}=    Execute Command    sysbus ReadDoubleWord ${addr}
    [Return]    ${v}

Send Continuous Exit Frame
    [Documentation]    Indirect quad read with NO instruction (IMODE=00) and
    ...                mode bits = 0 (ABR=0) — the frame that breaks AX
    ...                Continuous Read Mode even while the flash ignores
    ...                instruction-led commands. Then a software reset.
    Execute Command    sysbus WriteDoubleWord ${QSPI_ABR} 0x00000000
    Execute Command    sysbus WriteDoubleWord ${QSPI_DLR} 0x00000000
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x0718EC00
    Execute Command    sysbus WriteDoubleWord ${QSPI_AR} 0x00000000
    # Software reset: RSTEN 0x66 then RST 0x99 (single-line, command-only).
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x00000166
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x00000199

*** Test Cases ***
Continuous Mode Wedges Instruction Commands
    [Documentation]    Once in AX continuous mode, a JEDEC read is swallowed as
    ...                a read continuation → not the real ID.
    Provision QSPI Machine
    Enter Continuous Read Mode
    ${id}=    Read JEDEC Id
    Log    JEDEC while wedged: ${id.strip()}
    Should Not Contain    ${id}    ${JEDEC_ID}

Continuous Mode Survives Warm MCU Reset
    [Documentation]    THE KEY TEST: after a warm `machine Reset` the flash is
    ...                STILL wedged (separate power domain), so JEDEC is still
    ...                garbage — until a continuous-exit frame recovers it.
    Provision QSPI Machine
    Enter Continuous Read Mode
    # Warm MCU reset — resets the controller, must NOT reset the flash chip.
    Execute Command    machine Reset
    Execute Command    sysbus WriteDoubleWord ${QSPI_CR} 0x00000001
    Execute Command    sysbus WriteDoubleWord ${QSPI_DCR} 0x00160000
    ${wedged}=    Read JEDEC Id
    Log    JEDEC after warm reset (expect still wedged): ${wedged.strip()}
    Should Not Contain    ${wedged}    ${JEDEC_ID}
    # Recover with the continuous-exit frame + software reset.
    Send Continuous Exit Frame
    ${ok}=    Read JEDEC Id
    Log    JEDEC after continuous-exit recovery: ${ok.strip()}
    Should Contain    ${ok}    ${JEDEC_ID}

Power Cycle Clears Continuous Mode
    [Documentation]    A physical power cycle DOES reset the flash — JEDEC comes
    ...                back clean without needing the exit frame.
    Provision QSPI Machine
    Enter Continuous Read Mode
    Execute Command    machine Reset
    Execute Command    qspi.qspiFlash PowerCycle
    Execute Command    sysbus WriteDoubleWord ${QSPI_CR} 0x00000001
    Execute Command    sysbus WriteDoubleWord ${QSPI_DCR} 0x00160000
    ${id}=    Read JEDEC Id
    Log    JEDEC after power cycle: ${id.strip()}
    Should Contain    ${id}    ${JEDEC_ID}

Erase While Wedged Is Dropped
    [Documentation]    Bug C: a DFU erase issued while the flash is still in
    ...                continuous mode is misframed — the single-line WREN/erase
    ...                opcodes are swallowed as read continuations, so the erase
    ...                never happens. This is why our bootloader had to recover
    ...                the flash to single-line before erase/program.
    Provision QSPI Machine
    Prime Backing Word    0x90000100
    Enter Continuous Read Mode
    Attempt Sector Erase    0x00000100
    ${back}=    Read Backing Word    0x90000100
    Log    Flash @0x100 after erase-while-wedged: ${back.strip()}
    Should Contain        ${back}    0x00000000
    Should Not Contain    ${back}    0xFFFFFFFF

Erase After Recovery Lands
    [Documentation]    Positive control: from a clean single-line state (never
    ...                wedged), the identical WREN + Sector Erase lands and clears
    ...                the sector to 0xFF. Proves the drop above is specifically
    ...                the continuous-mode misframing, not a broken erase path.
    Provision QSPI Machine
    Prime Backing Word    0x90000100
    Attempt Sector Erase    0x00000100
    ${back}=    Read Backing Word    0x90000100
    Log    Flash @0x100 after clean erase: ${back.strip()}
    Should Contain    ${back}    0xFFFFFFFF
