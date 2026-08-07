*** Settings ***
Documentation    Diagnostic: run the app for 1 s virtual then dump CPU state.
...              If we're stuck in a specific loop, PC tells us where.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}

*** Variables ***
${APP}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/daisy-app-template
${PLATFORM}      ${CURDIR}/daisy_seed.repl

*** Test Cases ***
Dump CPU State After One Second
    Execute Command    mach create "daisy-app"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Execute Command    sysbus LoadELF @${APP}
    Execute Command    cpu VectorTableOffset 0x90000000

    Execute Command    emulation RunFor "00:00:01"

    ${pc}=    Execute Command    cpu PC
    ${sp}=    Execute Command    cpu SP
    ${moder}=    Execute Command    sysbus ReadDoubleWord 0x58020800
    ${odr}=      Execute Command    sysbus ReadDoubleWord 0x58020814
    Log To Console    \nPC:    ${pc}
    Log To Console    SP:    ${sp}
    Log To Console    GPIOC MODER:    ${moder}
    Log To Console    GPIOC ODR:    ${odr}
