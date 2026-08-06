*** Settings ***
Documentation    Run daisy-app-template alone in Renode with VTOR pointing
...              at its vector table. If the LED toggles, the app's Reset
...              handler + __pre_init + main are executing correctly.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${APP}           ${CURDIR}/../../target/thumbv7em-none-eabihf/release/daisy-app-template
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${GPIOC_ODR}     0x58020814
${LED_MASK}      0x00000080

*** Keywords ***
Sample LED Bit
    ${odr}=      Execute Command    sysbus ReadDoubleWord ${GPIOC_ODR}
    ${odr_int}=  Convert To Integer    ${odr.strip()}    16
    ${bit}=      Evaluate    (${odr_int} >> 7) & 1
    RETURN    ${bit}

*** Test Cases ***
App Template Blinks Standalone
    Execute Command    mach create "daisy-app"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    # Register the patched QUADSPI controller (with its executable-IO "xip"
    # region at 0x9000_0000) and the IS25LP064A flash before loading the app —
    # the app's vector table + code live in the XIP window served by them.
    Apply H7 Peripheral Stubs
    Execute Command    sysbus LoadELF @${APP}
    # The app's vector table sits at 0x90000000 (QSPI XIP). Force VTOR
    # there — Renode's auto-guess would otherwise pick the wrong region.
    Execute Command    cpu VectorTableOffset 0x90000000

    # App has irregular pattern (5-slow-pulse pre_init + long-on/rapid
    # /gap main). Sample PC7 (GPIOC bit 7) at 12 x 500 ms virtual = 6 s
    # total. Assert we saw both ON and OFF states — proof the app runs.
    @{bits}=    Create List
    FOR    ${i}    IN RANGE    12
        Execute Command    emulation RunFor "00:00:00.5"
        ${b}=    Sample LED Bit
        Append To List    ${bits}    ${b}
    END
    Log To Console    \nGPIO PC7 samples: ${bits}
    Should Contain    ${bits}    ${1}    LED never went HIGH — app didn't run
    Should Contain    ${bits}    ${0}    LED never went LOW — app didn't run
