*** Settings ***
Documentation    Boot the daisy-spatializer app (built with `renode_test`, which
...              skips the SAI/codec bring-up sim can't model) from QSPI XIP at
...              0x9000_0000, exactly as the bootloader would. If PC7 toggles,
...              the app's Reset handler + #[pre_init] (MPU/caches/DWT) + main —
...              including building the HRIR StereoConvolver into its static
...              scratch — are executing from FLASH, proving the XIP conversion
...              and the convolver's large static allocation are sound (guards the
...              __ebss / startup-lockup class for this app).
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${APP}           ${CURDIR}/../target/renode/thumbv7em-none-eabihf/release/daisy-spatializer
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${GPIOC_ODR}     0x58020814

*** Keywords ***
Sample LED Bit
    ${odr}=      Execute Command    sysbus ReadDoubleWord ${GPIOC_ODR}
    ${odr_int}=  Convert To Integer    ${odr.strip()}    16
    ${bit}=      Evaluate    (${odr_int} >> 7) & 1
    RETURN    ${bit}

*** Test Cases ***
Spatializer App Boots From XIP
    Execute Command    mach create "daisy-spatializer"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Execute Command    sysbus LoadELF @${APP}
    Execute Command    cpu VectorTableOffset 0x90000000

    @{bits}=    Create List
    FOR    ${i}    IN RANGE    8
        Execute Command    emulation RunFor "00:00:00.25"
        ${b}=    Sample LED Bit
        Append To List    ${bits}    ${b}
    END
    Log To Console    \nGPIO PC7 samples: ${bits}
    Should Contain    ${bits}    ${1}    LED never went HIGH — XIP app didn't run
    Should Contain    ${bits}    ${0}    LED never went LOW — heartbeat not toggling
