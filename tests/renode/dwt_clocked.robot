*** Settings ***
Documentation    Tests for STM32H7_DWT_Clocked — the DWT whose CYCCNT tick
...              rate is driven by the RCC clock tree instead of a fixed
...              platform constant. Covers the CYCCNTENA control flag, the
...              CYCCNT counter (count rate, freeze, write), and the RCC
...              integration that rescales the rate with sys_ck.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}        ${CURDIR}/daisy_seed.repl
${DWT_CTRL}        0xE0001000
${DWT_CYCCNT}      0xE0001004
${RCC_CR}          0x58024400
${RCC_CFGR}        0x58024410
${RCC_PLLCKSELR}   0x58024428
${RCC_PLLCFGR}     0x5802442C
${RCC_PLL1DIVR}    0x58024430
${ONE_MS}          00:00:00.001

*** Keywords ***
New DWT Machine
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision Clocked RCC And DWT

Enable CYCCNT
    Execute Command    sysbus WriteDoubleWord ${DWT_CTRL} 0x00000001

Set RCC PLL1
    [Arguments]    ${pll1divr}
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} 0x00000022
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL1DIVR} ${pll1divr}
    # DIVP1EN (bit16) so PLL1P is produced, PLL1RGE=10 (4–8 MHz), FRACEN (bit0).
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00010009
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x01010000
    Execute Command    sysbus WriteDoubleWord ${RCC_CFGR} 0x00000003

*** Test Cases ***
CYCCNTENA Control Flag Round Trips
    New DWT Machine
    Enable CYCCNT
    ${on}=    Execute Command    sysbus ReadDoubleWord ${DWT_CTRL}
    Should Contain    ${on}    0x00000001
    Execute Command    sysbus WriteDoubleWord ${DWT_CTRL} 0x00000000
    ${off}=    Execute Command    sysbus ReadDoubleWord ${DWT_CTRL}
    Should Contain    ${off}    0x00000000

Writing CYCCNT Sets The Counter
    New DWT Machine
    Execute Command    sysbus WriteDoubleWord ${DWT_CYCCNT} 0x12345678
    ${v}=    Execute Command    sysbus ReadDoubleWord ${DWT_CYCCNT}
    Should Contain    ${v}    0x12345678

CYCCNT Is Frozen While Disabled
    New DWT Machine
    Execute Command    emulation RunFor "${ONE_MS}"
    ${v}=    Execute Command    sysbus ReadDoubleWord ${DWT_CYCCNT}
    Should Contain    ${v}    0x00000000

CYCCNT Counts 400000 In 1ms At Default 400 MHz
    New DWT Machine
    Enable CYCCNT
    Execute Command    emulation RunFor "${ONE_MS}"
    ${v}=    Execute Command    sysbus ReadDoubleWord ${DWT_CYCCNT}
    # 1 ms × 400 MHz = 400000 = 0x00061A80.
    Should Contain    ${v}    0x00061A80

RCC At 200 MHz Halves The Count Rate
    New DWT Machine
    # PLL1DIVR 0x663 → DIVP1 ÷4 → 200 MHz sys_ck drives the DWT.
    Set RCC PLL1    0x00000663
    Enable CYCCNT
    Execute Command    emulation RunFor "${ONE_MS}"
    ${v}=    Execute Command    sysbus ReadDoubleWord ${DWT_CYCCNT}
    # 1 ms × 200 MHz = 200000 = 0x00030D40.
    Should Contain    ${v}    0x00030D40

RCC At 480 MHz Speeds Up The Count
    New DWT Machine
    # PLL1DIVR 0x277 → DIVN1 120 → 480 MHz sys_ck drives the DWT.
    Set RCC PLL1    0x00000277
    Enable CYCCNT
    Execute Command    emulation RunFor "${ONE_MS}"
    ${v}=    Execute Command    sysbus ReadDoubleWord ${DWT_CYCCNT}
    # 1 ms × 480 MHz = 480000 = 0x00075300.
    Should Contain    ${v}    0x00075300
