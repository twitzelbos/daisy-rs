*** Settings ***
Documentation    Diagnostic: run bootloader for a bounded virtual time and
...              log CPU state so we can see where clock init gets stuck.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${BOOTLOADER}    ${CURDIR}/../target/renode/thumbv7em-none-eabihf/release/daisy-boot
${APP}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/daisy-app-template
${PLATFORM}      ${CURDIR}/daisy_seed.repl

*** Test Cases ***
Dump Bootloader State After Two Seconds
    Execute Command    mach create "daisy-boot"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Execute Command    sysbus LoadELF @${BOOTLOADER}
    Execute Command    sysbus LoadELF @${APP}
    Execute Command    cpu VectorTableOffset 0x08000000

    Execute Command    emulation RunFor "00:00:02"

    ${pc}=       Execute Command    cpu PC
    ${sp}=       Execute Command    cpu SP
    ${moder}=    Execute Command    sysbus ReadDoubleWord 0x58020800
    ${odr}=      Execute Command    sysbus ReadDoubleWord 0x58020814
    Log To Console    \n---- state after 2 s virtual ----
    Log To Console    PC=${pc.strip()}
    Log To Console    SP=${sp.strip()}
    Log To Console    GPIOC MODER=${moder.strip()}
    Log To Console    GPIOC ODR=${odr.strip()}

    Execute Command    emulation RunFor "00:00:02"
    ${pc2}=      Execute Command    cpu PC
    ${odr2}=     Execute Command    sysbus ReadDoubleWord 0x58020814
    Log To Console    ---- state after 4 s virtual ----
    Log To Console    PC=${pc2.strip()}
    Log To Console    GPIOC ODR=${odr2.strip()}

    Execute Command    emulation RunFor "00:00:04"
    ${pc3}=      Execute Command    cpu PC
    ${odr3}=     Execute Command    sysbus ReadDoubleWord 0x58020814
    Log To Console    ---- state after 8 s virtual ----
    Log To Console    PC=${pc3.strip()}
    Log To Console    GPIOC ODR=${odr3.strip()}
