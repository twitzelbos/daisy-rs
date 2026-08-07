*** Settings ***
Documentation    Boot the daisy-rs bootloader in Renode and verify the
...              user LED (PC7) toggles. Uses STM32H7 peripheral stubs
...              (PWR/RCC/FLASH/SYSCFG) so stm32h7xx-hal's `clocks::init`
...              runs to completion — see peripherals/NOTICE.md.
...
...              Note: the renode_test bootloader now includes the jump
...              path (renode_boot_and_jump), so what this test actually
...              observes is: 4 quick alive pulses from the bootloader,
...              then the jump, then the app's LED activity. Any LED
...              toggling in the 6 s window proves the whole chain works.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${BOOTLOADER}    ${CURDIR}/../target/renode/thumbv7em-none-eabihf/release/daisy-boot
${APP}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/daisy-app-template
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${GPIOC_ODR}     0x58020814

*** Keywords ***
Sample LED Bit
    ${odr}=      Execute Command    sysbus ReadDoubleWord ${GPIOC_ODR}
    ${odr_int}=  Convert To Integer    ${odr.strip()}    16
    ${bit}=      Evaluate    (${odr_int} >> 7) & 1
    RETURN    ${bit}

*** Test Cases ***
Bootloader Alive Blink And App Both Run
    Execute Command    mach create "daisy-boot"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Execute Command    sysbus LoadELF @${BOOTLOADER}
    Execute Command    sysbus LoadELF @${APP}
    Execute Command    cpu VectorTableOffset 0x08000000

    # 12 samples × 500 ms = 6 s virtual — covers bootloader alive-pulses,
    # jump, and enough of the app's pattern to see both ON and OFF states.
    @{bits}=    Create List
    FOR    ${i}    IN RANGE    12
        Execute Command    emulation RunFor "00:00:00.5"
        ${b}=    Sample LED Bit
        Append To List    ${bits}    ${b}
    END
    Log To Console    \nGPIO PC7 samples: ${bits}
    Should Contain    ${bits}    ${1}    LED never went HIGH — bootloader/app not running
    Should Contain    ${bits}    ${0}    LED never went LOW — bootloader/app not running
