*** Settings ***
Documentation    Reproduce Bug D: the app resets GPIOG while executing from
...              QSPI XIP. PG6 is the QSPI chip-select (NCS) at alternate
...              function AF10; the HAL's GPIOG.split() does prec.enable()
...              .reset() — a peripheral-reset pulse that reverts PG6 to input.
...              With NCS no longer driven, the flash is deselected and the next
...              instruction fetch reads 0xFF and the CPU faults. The fix is
...              daisy-hothouse's split_without_reset for GPIOG.
...
...              STM32H7_QuadSPI_Fixed now models this with true pin-mux
...              fidelity: with NcsGpioModerAddress set, every XIP fetch checks
...              the live GPIOG MODER + AFRL for PG6 and returns 0xFF when it is
...              not in the QSPI alternate function.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}       ${CURDIR}/daisy_seed.repl
${GPIOG_MODER}    0x58021800
${GPIOG_AFRL}     0x58021820
${QSPI_CR}        0x52005000
${QSPI_DCR}       0x52005004
${QSPI_DLR}       0x52005010
${QSPI_CCR}       0x52005014
${QSPI_ABR}       0x5200501C
${QSPI_DR}        0x52005020

*** Keywords ***
Route NCS Pin To QSPI
    [Documentation]    Mux PG6 to AF10 (QUADSPI): MODER pin6 = 0b10, AFRL pin6 =
    ...                0xA — exactly what the bootloader's configure_pins sets.
    Execute Command    sysbus WriteDoubleWord ${GPIOG_MODER} 0x00002000
    Execute Command    sysbus WriteDoubleWord ${GPIOG_AFRL} 0x0A000000

Reset GPIOG
    [Documentation]    Model GPIOG.split()'s reset pulse: MODER reverts to its
    ...                all-input reset default, dropping PG6 out of AF10.
    Execute Command    sysbus WriteDoubleWord ${GPIOG_MODER} 0x00000000

Provision XIP With NCS Modelling
    Execute Command    mach create "qspi-ncs"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    # Enable true NCS pin-mux modelling against GPIOG MODER.
    Execute Command    qspi NcsGpioModerAddress 0x58021800
    Route NCS Pin To QSPI
    Execute Command    sysbus WriteDoubleWord 0x90000000 0xDEADBEEF
    Execute Command    sysbus WriteDoubleWord ${QSPI_CR} 0x00000001
    Execute Command    sysbus WriteDoubleWord ${QSPI_DCR} 0x00160000
    Set Flash Quad Enable
    # libDaisy's exact config: Set Read Parameters = 0xF0 (8 dummy cycles) so
    # the flash's 6-after-mode quad dummy cycles (3 bytes) match the
    # controller's DCYC=6 (3 bytes). Byte-exact continuous read.
    Execute Command    sysbus WriteDoubleWord ${QSPI_DLR} 0x00000000
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x010001C0
    Execute Command    sysbus WriteDoubleWord ${QSPI_DR} 0x000000F0
    Execute Command    sysbus WriteDoubleWord ${QSPI_ABR} 0x000000A0
    Execute Command    sysbus WriteDoubleWord ${QSPI_CCR} 0x1F18EDEB
    Execute Command    emulation RunFor "00:00:00.001"

Read XIP Word0
    ${w}=    Execute Command    sysbus ReadDoubleWord 0x90000000
    [Return]    ${w}

*** Test Cases ***
XIP Works While PG6 Is In QSPI Alternate Function
    Provision XIP With NCS Modelling
    ${w}=    Read XIP Word0
    Log    XIP word0 with NCS routed: ${w.strip()}
    Should Contain    ${w}    0xDEADBEEF

Resetting GPIOG Deasserts NCS And Breaks XIP
    Provision XIP With NCS Modelling
    ${ok}=    Read XIP Word0
    Should Contain    ${ok}    0xDEADBEEF
    # App resets GPIOG mid-XIP (the split() bug) — PG6 leaves AF10.
    Reset GPIOG
    ${broken}=    Read XIP Word0
    Log    XIP word0 after GPIOG reset: ${broken.strip()}
    Should Not Contain    ${broken}    0xDEADBEEF
    Should Contain        ${broken}    0xFFFFFFFF
    # Re-muxing PG6 to AF10 restores XIP — i.e. never resetting it (the
    # split_without_reset fix) keeps the chip-select alive.
    Route NCS Pin To QSPI
    ${restored}=    Read XIP Word0
    Log    XIP word0 after re-routing NCS: ${restored.strip()}
    Should Contain    ${restored}    0xDEADBEEF
