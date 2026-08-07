*** Settings ***
Documentation    The STM32H7 ADC bring-up handshake — the sequence
...              stm32h7xx-hal's Adc::adc1(...).enable() drives, and which
...              hung in sim because the base platform mapped ADC1/2 to an
...              F0-generation model missing the H7 status bits. Drives the
...              exact register sequence (ADVREGEN→LDORDY, ADCAL self-clear,
...              ADEN→ADRDY, ADSTART→EOC→DR) against STM32H7_ADC and asserts each
...              handshake completes, then that a conversion returns the injected
...              channel value (standing in for a Hothouse pot).
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
# ADC1 register block @ 0x40022000 (RM0433 §25.4).
${ADC_ISR}       0x40022000
${ADC_CR}        0x40022008
${ADC_CFGR}      0x4002200C
${ADC_SQR1}      0x40022030
${ADC_DR}        0x40022040

*** Keywords ***
Provision ADC Machine
    Execute Command    mach create "adc"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision ADC

Read ADC Reg
    [Arguments]    ${addr}
    ${v}=    Execute Command    sysbus ReadDoubleWord ${addr}
    ${i}=    Convert To Integer    ${v.strip()}    16
    [Return]    ${i}

Bit Should Be Set
    [Arguments]    ${val}    ${bit}    ${msg}
    ${b}=    Evaluate    (${val} >> ${bit}) & 1
    Should Be Equal As Integers    ${b}    1    ${msg}

Bit Should Be Clear
    [Arguments]    ${val}    ${bit}    ${msg}
    ${b}=    Evaluate    (${val} >> ${bit}) & 1
    Should Be Equal As Integers    ${b}    0    ${msg}

*** Test Cases ***
ADC Bringup Handshake Completes And Conversion Reads Back
    Provision ADC Machine

    # Reset state: CR = 0 (RM0433 §25.4.25 — DEEPPWD NOT set, LDO off).
    ${cr}=    Read ADC Reg    ${ADC_CR}
    Should Be Equal As Integers    ${cr}    0    CR reset value is not 0

    # 1) power_up: enable the voltage regulator (ADVREGEN, bit 28) → LDORDY.
    Execute Command    sysbus WriteDoubleWord ${ADC_CR} 0x10000000
    ${isr}=    Read ADC Reg    ${ADC_ISR}
    Bit Should Be Set    ${isr}    12    LDORDY never set after ADVREGEN — power_up would hang

    # 2) calibrate: set ADCAL (bit 31); it must self-clear (calibration done).
    Execute Command    sysbus WriteDoubleWord ${ADC_CR} 0x90000000
    ${cr}=    Read ADC Reg    ${ADC_CR}
    Bit Should Be Clear    ${cr}    31    ADCAL did not self-clear — calibrate would hang

    # 3) enable: set ADEN (bit 0) → ADRDY.
    Execute Command    sysbus WriteDoubleWord ${ADC_CR} 0x10000001
    ${isr}=    Read ADC Reg    ${ADC_ISR}
    Bit Should Be Set    ${isr}    0    ADRDY never set after ADEN — enable would hang

    # 4) convert channel 3 (SQR1.SQ1 = 3 → 3<<6 = 0xC0) with an injected value.
    Execute Command    sysbus WriteDoubleWord ${ADC_SQR1} 0x000000C0
    Execute Command    adc1 SetChannelValue 3 0x1234
    Execute Command    sysbus WriteDoubleWord ${ADC_CR} 0x10000005
    ${isr}=    Read ADC Reg    ${ADC_ISR}
    Bit Should Be Set    ${isr}    2    EOC never set after ADSTART — read would hang
    ${dr}=    Read ADC Reg    ${ADC_DR}
    Should Be Equal As Integers    ${dr}    0x1234    conversion did not return the injected channel value

    # Reading DR cleared EOC (RM0433 §25.4.26).
    ${isr}=    Read ADC Reg    ${ADC_ISR}
    Bit Should Be Clear    ${isr}    2    EOC was not cleared by reading DR

    # Single-conversion mode (CFGR.CONT=0) self-clears ADSTART (bit 2).
    ${cr}=    Read ADC Reg    ${ADC_CR}
    Bit Should Be Clear    ${cr}    2    ADSTART did not self-clear in single mode
