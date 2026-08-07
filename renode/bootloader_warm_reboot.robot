*** Settings ***
Documentation    Bug B, end-to-end in real firmware: after a WARM MCU reset the
...              QSPI flash is still in AX Continuous Read Mode (separate power
...              domain — the flash isn't reset). The bootloader must break it
...              out of continuous mode before reconfiguring, or every XIP fetch
...              on the second boot is misframed, the app vector table reads
...              garbage, is_plausible_image fails, and the bootloader falls
...              through to DFU instead of jumping.
...
...              This pins the enter_memory_mapped recovery (exit_continuous_read
...              + software reset). Remove it and the SECOND boot below stops
...              reaching the app — a regression this test catches.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${BOOTLOADER}    ${CURDIR}/../target/renode/thumbv7em-none-eabihf/release/daisy-boot
${APP}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/daisy-app-template
${SCB_VTOR}      0xE000ED08

*** Keywords ***
Boot From Reset
    [Documentation]    (Re)start the CPU from the bootloader vector table and
    ...                give it time to configure QSPI, validate the app, and jump.
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:03"

*** Test Cases ***
Bootloader Recovers A Wedged Flash On Warm Reboot
    Execute Command    mach create "daisy-warm-reboot"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    # Enforce NCS pin-mux across both boots (see bootloader_jump).
    Execute Command    qspi NcsGpioModerAddress 0x58021800
    Execute Command    sysbus LoadELF @${BOOTLOADER}
    Execute Command    sysbus LoadELF @${APP}

    # Cold boot: bootloader sets up 0xEB continuous mode (leaving the flash in
    # AX Continuous Read Mode) and jumps to the app.
    Boot From Reset
    ${vtor1}=    Execute Command    sysbus ReadDoubleWord ${SCB_VTOR}
    Log    VTOR after cold boot: ${vtor1.strip()}
    Should Contain    ${vtor1}    0x90000000

    # Warm MCU reset: the flash stays wedged in continuous mode (its state
    # survives an MCU reset). The bootloader must break it out again.
    Execute Command    machine Reset
    Boot From Reset
    ${vtor2}=    Execute Command    sysbus ReadDoubleWord ${SCB_VTOR}
    Log    VTOR after warm reboot: ${vtor2.strip()}
    Should Contain    ${vtor2}    0x90000000
