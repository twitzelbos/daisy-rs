*** Settings ***
Documentation    Boots the daisy-rs bootloader in Renode with the app
...              template preloaded into QSPI. After a short alive-blink,
...              the bootloader calls jump_to_app() — the same path
...              hardware uses. This test asserts:
...
...                (a) VTOR was retargeted to the app's vector table at
...                    0x9000_0000 — proof jump_to_app executed;
...                (b) the CPU's PC ends up inside the app's address
...                    range (0x9000_0000..0x9080_0000);
...                (c) the LED toggles both ON and OFF after the jump —
...                    proof the app is executing.
...
...              A pass here means the bootloader-to-app transition is
...              known-good in emulation, which pins the current
...              hardware bug to something silicon-specific (QSPI XIP
...              timing, cache, errata) rather than our jump code.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${BOOTLOADER}    ${CURDIR}/../target/renode/thumbv7em-none-eabihf/release/daisy-boot
${APP}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/daisy-app-template
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
Bootloader Jumps To App And App Reaches Its Main Loop
    Execute Command    mach create "daisy-boot-jump"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    # Enforce QSPI NCS pin-mux fidelity on every fetch: if the bootloader or app
    # ever drops PG6 out of AF10 while running from XIP, the flash deselects,
    # the fetch reads 0xFF and the CPU faults — so this boot would stop reaching
    # the app. (configure_pins sets PG6=AF10 before enter_memory_mapped.)
    Execute Command    qspi NcsGpioModerAddress 0x58021800
    Execute Command    sysbus LoadELF @${BOOTLOADER}
    Execute Command    sysbus LoadELF @${APP}
    Execute Command    cpu VectorTableOffset 0x08000000

    # Give the bootloader time to do clocks::init, alive-blink (~400 ms),
    # jump, and let the app run pre_init and main.
    Execute Command    emulation RunFor "00:00:03"

    ${vtor}=     Execute Command    sysbus ReadDoubleWord ${SCB_VTOR}
    ${vtor_int}=  Convert To Integer    ${vtor.strip()}    16
    Log To Console    \nVTOR after 3 s virtual: ${vtor.strip()}
    Should Be Equal As Integers    ${vtor_int}    0x90000000
    ...    VTOR was not retargeted to QSPI — jump_to_app never ran

    ${pc}=       Execute Command    cpu PC
    ${pc_int}=   Convert To Integer    ${pc.strip()}    16
    Log To Console    PC after 3 s virtual: ${pc.strip()}
    ${in_range}=    Evaluate    0x90000000 <= ${pc_int} < 0x90800000
    Should Be True    ${in_range}
    ...    PC ${pc.strip()} is not inside the QSPI XIP window

    # Verify the app's LED is toggling: sample GPIOC ODR bit 7 over a
    # 6 s window; require we see both HIGH and LOW.
    @{bits}=    Create List
    FOR    ${i}    IN RANGE    12
        Execute Command    emulation RunFor "00:00:00.5"
        ${b}=    Sample LED Bit
        Append To List    ${bits}    ${b}
    END
    Log To Console    GPIO PC7 samples: ${bits}
    Should Contain    ${bits}    ${1}    LED never went HIGH after jump
    Should Contain    ${bits}    ${0}    LED never went LOW after jump
