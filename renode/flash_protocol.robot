*** Settings ***
Documentation    Datasheet-grounded checks of the IS25LP064A model
...              driven through the STM32H7 QUADSPI in indirect mode — the
...              same command path the bootloader uses. Validates identity
...              reads, status-register / QE semantics, and (negatively)
...              that WRSR cannot write the read-only WIP/WEL bits.
...
...              References: IS25LP064A datasheet Table 8.5 (Product ID),
...              Table 6.1/6.2 (Status Register), §6.1 (QE bit), §8.7.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${QSPI_CR}       0x52005000
${QSPI_DLR}      0x52005010
${QSPI_CCR}      0x52005014
${QSPI_DR}       0x52005020
# Indirect-mode CCR presets (IMODE=1 line, DMODE=1 line, FMODE per op).
${CCR_WREN}      0x00000106
${CCR_WRSR}      0x01000101
${CCR_RDSR}      0x05000105
${CCR_JEDEC}     0x0500019F

*** Keywords ***
Create Flash Machine
    Execute Command    mach create "flash-proto"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Execute Command    sysbus WriteDoubleWord ${QSPI_CR} 0x00000001

Indirect Command Only
    [Arguments]    ${ccr}
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} ${ccr}

Indirect Write Byte
    [Arguments]    ${ccr}    ${value}
    Execute Command    sysbus WriteDoubleWord ${QSPI_DLR} 0x00000000
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} ${ccr}
    Execute Command    sysbus WriteDoubleWord ${QSPI_DR} ${value}

Read Status Register
    Execute Command    sysbus WriteDoubleWord ${QSPI_DLR} 0x00000000
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} ${CCR_RDSR}
    ${dr}=    Execute Command    sysbus ReadDoubleWord ${QSPI_DR}
    [Return]    ${dr.strip()}

*** Test Cases ***
JEDEC ID Reads ISSI 60 17
    [Documentation]    0x9F returns Manufacturer 0x9D, Type 0x60, Capacity
    ...                0x17 (Table 8.5). DR packs first byte in the low
    ...                position → 0x0017609D.
    Create Flash Machine
    Execute Command    sysbus WriteDoubleWord ${QSPI_DLR} 0x00000002
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} ${CCR_JEDEC}
    ${dr}=    Execute Command    sysbus ReadDoubleWord ${QSPI_DR}
    Log    JEDEC ID DR = ${dr.strip()}
    Should Contain    ${dr}    0x0017609D

Write Enable Sets WEL
    [Documentation]    WREN (0x06) sets the WEL bit (status bit 1).
    Create Flash Machine
    Indirect Command Only    ${CCR_WREN}
    ${sr}=    Read Status Register
    Log    Status after WREN = ${sr}
    Should Contain    ${sr}    0x00000002

Write Status Register Sets QE
    [Documentation]    WRSR (0x01) with 0x40 sets QE (bit 6). WEL auto-clears
    ...                on write completion, so the readback is 0x40.
    Create Flash Machine
    Indirect Command Only    ${CCR_WREN}
    Indirect Write Byte    ${CCR_WRSR}    0x00000040
    ${sr}=    Read Status Register
    Log    Status after WRSR 0x40 = ${sr}
    Should Contain    ${sr}    0x00000040

Write Status Register Cannot Set WIP Or WEL
    [Documentation]    Datasheet Table 6.2 Note 1: WRSR must not write WIP
    ...                (bit 0) or WEL (bit 1). Writing 0xFF must leave WIP=0
    ...                and WEL=0 (WEL auto-clears), giving 0xFC — never 0xFD
    ...                (which would mean WIP got written).
    Create Flash Machine
    Indirect Command Only    ${CCR_WREN}
    Indirect Write Byte    ${CCR_WRSR}    0x000000FF
    ${sr}=    Read Status Register
    Log    Status after WRSR 0xFF = ${sr}
    Should Contain       ${sr}    0x000000FC
    Should Not Contain   ${sr}    0x000000FD
