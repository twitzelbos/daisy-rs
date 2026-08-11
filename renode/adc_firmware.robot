*** Settings ***
Documentation    End-to-end ADC bring-up in REAL firmware: the adc-exerciser
...              runs daisy_bsp::clocks::init then the HAL's Adc::adc1(...)
...              .enable() and one blocking conversion — the exact path
...              daisy-hothouse's knob reader uses. It records a progress marker
...              at each step and the converted value in a MARKERS[] static.
...
...              This proves the STM32H7_ADC model supports the real HAL
...              hand-shake (poll ISR.LDORDY, CR.ADCAL self-clear, ISR.ADRDY),
...              not just a hand-written register sequence — and that the ADC
...              kernel clock (PLL2P = 40 MHz) is within the VOS1 limit so
...              Adc::adc1's ≤ 80 MHz assert does not panic. The conversion must
...              return the value the test injects for the PA3 channel (15).
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${ELF}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/adc-exerciser
# Slots into the firmware's MARKERS[] static (base resolved via GetSymbolAddress).
${M_STAGE}       0
${M_VALUE}       1

*** Test Cases ***
Real ADC Bringup Completes And Reads Injected Channel
    Execute Command    mach create "adc-fw"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    # clocks::init runs the HAL freeze → needs the PWR/RCC/SYSCFG/FLASH stubs and
    # the clock-computing RCC; the H7 ADC model replaces the base F0 one.
    Apply H7 Peripheral Stubs
    Provision Clocked RCC And DWT
    Provision ADC
    # Inject the pot value the firmware will read on PA3 (ADC1 channel 15).
    Execute Command    adc1 SetChannelValue 15 0x0ABC

    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    cpu VectorTableOffset 0x08000000
    ${mb}=    Execute Command    sysbus GetSymbolAddress "MARKERS"
    ${base}=    Convert To Integer    ${mb.strip()}    16
    Execute Command    emulation RunFor "00:00:00.5"

    # Stage 0x15 = clocks::init + full Adc::adc1 bring-up + enable + one
    # conversion all completed (each earlier stage would leave a lower marker if
    # a poll hung).
    ${sa}=    Evaluate    ${base} + ${M_STAGE} * 4
    ${stage}=    Execute Command    sysbus ReadDoubleWord ${sa}
    ${stage_int}=    Convert To Integer    ${stage.strip()}    16
    Log    ADC firmware stage: ${stage_int}
    Should Be Equal As Integers    ${stage_int}    0x15    ADC firmware did not reach stage 0x15

    # ...and the conversion returned the value injected for channel 15.
    ${va}=    Evaluate    ${base} + ${M_VALUE} * 4
    ${value}=    Execute Command    sysbus ReadDoubleWord ${va}
    ${value_int}=    Convert To Integer    ${value.strip()}    16
    Log    ADC read value: ${value_int}
    Should Be Equal As Integers    ${value_int}    0x0ABC    ADC did not read the injected channel value
