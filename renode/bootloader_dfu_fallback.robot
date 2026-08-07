*** Settings ***
Documentation    The boot DECISION path: when the QSPI slot holds no valid app
...              (erased flash / bad vector table), the bootloader must NOT
...              jump — is_plausible_image rejects it and the bootloader stays
...              in service mode (DFU on hardware; an alive-blink under the
...              renode_test build). bootloader_jump only covers the valid-app
...              path; this pins the negative decision so a regression that
...              jumps to a bogus vector (and hard-faults the CPU) is caught.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${BOOTLOADER}    ${CURDIR}/../target/renode/thumbv7em-none-eabihf/release/daisy-boot
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${SCB_VTOR}      0xE000ED08
${GPIOC_ODR}     0x58020814

*** Keywords ***
Sample LED Bit
    ${odr}=      Execute Command    sysbus ReadDoubleWord ${GPIOC_ODR}
    ${odr_int}=  Convert To Integer    ${odr.strip()}    16
    ${bit}=      Evaluate    (${odr_int} >> 7) & 1
    RETURN    ${bit}

*** Test Cases ***
Invalid QSPI App Does Not Jump
    Execute Command    mach create "daisy-dfu-fallback"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Execute Command    qspi NcsGpioModerAddress 0x58021800
    # Bootloader only — NO app loaded. Make the app vector explicitly invalid,
    # the erased-flash state (MSP + reset = 0xFFFFFFFF) a never-flashed slot has.
    Execute Command    sysbus LoadELF @${BOOTLOADER}
    Execute Command    sysbus WriteDoubleWord 0x90000000 0xFFFFFFFF
    Execute Command    sysbus WriteDoubleWord 0x90000004 0xFFFFFFFF
    Execute Command    cpu VectorTableOffset 0x08000000

    Execute Command    emulation RunFor "00:00:03"

    # Bootloader must NOT have jumped: VTOR stays at the bootloader's table.
    ${vtor}=    Execute Command    sysbus ReadDoubleWord ${SCB_VTOR}
    ${vtor_int}=    Convert To Integer    ${vtor.strip()}    16
    Log To Console    \nVTOR with invalid app: ${vtor.strip()}
    Should Be Equal As Integers    ${vtor_int}    0x08000000
    ...    Bootloader jumped despite an invalid app vector

    # ...and it must still be ALIVE (service-mode alive-blink: 250 ms on/off),
    # not hung or faulted: PC7 toggles both HIGH and LOW. Sample at 100 ms — a
    # non-multiple of the 500 ms blink period, so we don't alias to one phase.
    @{bits}=    Create List
    FOR    ${i}    IN RANGE    25
        Execute Command    emulation RunFor "00:00:00.1"
        ${b}=    Sample LED Bit
        Append To List    ${bits}    ${b}
    END
    Log To Console    GPIO PC7 samples (alive-blink): ${bits}
    Should Contain    ${bits}    ${1}    bootloader LED never went HIGH — not alive
    Should Contain    ${bits}    ${0}    bootloader LED never went LOW — not alive
