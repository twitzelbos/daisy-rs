*** Settings ***
Documentation    Register- and behaviour-level tests for STM32H7_RCC_Clocked,
...              grounded in RM0433 §8.7 (RCC). Covers: every enable→ready pair
...              the HAL polls (CR); the SW→SWS clock-switch mirror (CFGR); the
...              sys_ck derivation from PLL1; and — the point of the model — the
...              PLL2/PLL3 P/Q/R outputs, their per-output DIVxyEN + PLLxON +
...              FRACEN gating, the FMC (D1CCIPR) and SAI1 (D2CCIP1R) kernel
...              clock muxes, and the §8.7.13 RGE/VCOSEL range validation.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}        ${CURDIR}/daisy_seed.repl
${RCC_CR}          0x58024400
${RCC_CFGR}        0x58024410
${RCC_D1CFGR}      0x58024418
${RCC_PLLCKSELR}   0x58024428
${RCC_PLLCFGR}     0x5802442C
${RCC_PLL1DIVR}    0x58024430
${RCC_PLL1FRACR}   0x58024434
${RCC_PLL2DIVR}    0x58024438
${RCC_PLL2FRACR}   0x5802443C
${RCC_PLL3DIVR}    0x58024440
${RCC_PLL3FRACR}   0x58024444
${RCC_D1CCIPR}     0x5802444C
${RCC_D2CCIP1R}    0x58024450

*** Keywords ***
New RCC Machine
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision Clocked RCC And DWT

Assert CR Ready
    [Documentation]    Write an enable bit to CR and confirm the paired ready
    ...                bit reads back set (RM0433 §8.7.2).
    [Arguments]    ${written}    ${expected}
    New RCC Machine
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} ${written}
    ${v}=    Execute Command    sysbus ReadDoubleWord ${RCC_CR}
    Should Contain    ${v}    ${expected}

Configure PLL1 And Switch
    [Documentation]    Program PLLCKSELR/PLL1DIVR/PLL1FRACR, enable + select
    ...                PLL1 as system clock. PLLCFGR sets DIVP1EN (bit16, so
    ...                PLL1P is produced), PLL1RGE=10 (4–8 MHz, matches the
    ...                8 MHz VCO input) and PLL1FRACEN (bit0, so FRACR counts).
    [Arguments]    ${pllckselr}    ${pll1divr}    ${fracr}=0x00000000    ${cfgr}=0x00000003    ${d1cfgr}=0x00000000
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} ${pllckselr}
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL1DIVR} ${pll1divr}
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL1FRACR} ${fracr}
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00010009
    Execute Command    sysbus WriteDoubleWord ${RCC_D1CFGR} ${d1cfgr}
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x01010000
    Execute Command    sysbus WriteDoubleWord ${RCC_CFGR} ${cfgr}

Configure PLL2 FMC
    [Documentation]    PLL2R = 200 MHz for the FMC/SDRAM kernel clock.
    ...                HSE 16 / DIVM2=8 = 2 MHz; ×200 = 400 MHz VCO; DIVR2 ÷2 =
    ...                200 MHz. Only DIVR2EN is set (P/Q stay 0). RGE=01 (2–4 MHz).
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} 0x00008002
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL2DIVR} 0x010000C7
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00200040
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x04000000

Configure PLL3 SAI
    [Documentation]    PLL3P = 48 MHz for the SAI1 kernel clock.
    ...                HSE 16 / DIVM3=8 = 2 MHz; ×192 = 384 MHz VCO; DIVP3 ÷8 =
    ...                48 MHz. Only DIVP3EN is set. RGE=01 (2–4 MHz).
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} 0x00800002
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL3DIVR} 0x00000EBF
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00400400
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x10000000

*** Test Cases ***
# --- CR enable → ready mirroring (RM0433 §8.7.2) ---
HSI Enable Sets HSIRDY
    Assert CR Ready    0x00000001    0x00000005

CSI Enable Sets CSIRDY
    Assert CR Ready    0x00000080    0x00000180

HSI48 Enable Sets HSI48RDY
    Assert CR Ready    0x00001000    0x00003000

HSE Enable Sets HSERDY
    Assert CR Ready    0x00010000    0x00030000

PLL1 Enable Sets PLL1RDY
    Assert CR Ready    0x01000000    0x03000000

PLL2 Enable Sets PLL2RDY
    Assert CR Ready    0x04000000    0x0C000000

PLL3 Enable Sets PLL3RDY
    Assert CR Ready    0x10000000    0x30000000

# --- CFGR SW → SWS mirroring (RM0433 §8.7.6) ---
SW HSI Reflected In SWS
    New RCC Machine
    Execute Command    sysbus WriteDoubleWord ${RCC_CFGR} 0x00000000
    ${v}=    Execute Command    sysbus ReadDoubleWord ${RCC_CFGR}
    Should Contain    ${v}    0x00000000

SW CSI Reflected In SWS
    New RCC Machine
    Execute Command    sysbus WriteDoubleWord ${RCC_CFGR} 0x00000001
    ${v}=    Execute Command    sysbus ReadDoubleWord ${RCC_CFGR}
    Should Contain    ${v}    0x00000009

SW HSE Reflected In SWS
    New RCC Machine
    Execute Command    sysbus WriteDoubleWord ${RCC_CFGR} 0x00000002
    ${v}=    Execute Command    sysbus ReadDoubleWord ${RCC_CFGR}
    Should Contain    ${v}    0x00000012

SW PLL1 Reflected In SWS
    New RCC Machine
    Execute Command    sysbus WriteDoubleWord ${RCC_CFGR} 0x00000003
    ${v}=    Execute Command    sysbus ReadDoubleWord ${RCC_CFGR}
    Should Contain    ${v}    0x0000001B

# --- sys_ck derivation (RM0433 §8.7.11/13/14; verified via the RCC log) ---
SysCk 400 MHz From HSE DIVM2 DIVN100 DIVP2
    New RCC Machine
    Create Log Tester    5
    # HSE 16 MHz / 2 × 100 / 2 = 400 MHz.
    Configure PLL1 And Switch    0x00000022    0x00000263
    Wait For Log Entry    sys_ck = 400000000

SysCk 200 MHz With DIVP4
    New RCC Machine
    Create Log Tester    5
    # Same VCO, DIVP1=3 (÷4) → 200 MHz.
    Configure PLL1 And Switch    0x00000022    0x00000663
    Wait For Log Entry    sys_ck = 200000000

SysCk 480 MHz From HSE DIVN120
    New RCC Machine
    Create Log Tester    5
    # 16/2 × 120 / 2 = 480 MHz. DIVN1=119 (0x77), DIVP1=1 → 0x77 | 0x200 = 0x277.
    Configure PLL1 And Switch    0x00000022    0x00000277
    Wait For Log Entry    sys_ck = 480000000

SysCk Fractional 401 MHz Via FRACN
    New RCC Machine
    Create Log Tester    5
    # 16/2=8 MHz VCO-in; N=100, FRACN=2048 (¼) → ×100.25 = 802 MHz /2 = 401 MHz.
    # PLL1FRACEN is set by the helper, so the ¼ step is applied.
    Configure PLL1 And Switch    0x00000022    0x00000263    0x00004000
    Wait For Log Entry    sys_ck = 401000000

SysCk Falls Back To HSI When SW Selects HSI
    New RCC Machine
    Create Log Tester    5
    # Configure PLL1 but leave SW=HSI (0) → sys_ck = 64 MHz (HSI).
    Configure PLL1 And Switch    0x00000022    0x00000263    0x00000000    0x00000000
    Wait For Log Entry    sys_ck = 64000000

SysCk Halved By D1CPRE Prescaler
    New RCC Machine
    Create Log Tester    5
    # 400 MHz PLL1 with D1CPRE = 0b1000 (÷2) → 200 MHz sys_ck.
    Configure PLL1 And Switch    0x00000022    0x00000263    0x00000000    0x00000003    0x00000800
    Wait For Log Entry    sys_ck = 200000000

# --- PLL2 → FMC kernel clock (RM0433 §8.7.13/14, D1CCIPR §9.5.44) ---
PLL2R 200 MHz Drives FMC Kernel Clock
    New RCC Machine
    Create Log Tester    5
    Configure PLL2 FMC
    # FMCSEL = 10 → pll2_r_ck.
    Execute Command    sysbus WriteDoubleWord ${RCC_D1CCIPR} 0x00000002
    Wait For Log Entry    pll2r = 200000000
    Wait For Log Entry    fmc_ker = 200000000

PLL2R Is Zero Without DIVR2EN
    New RCC Machine
    Create Log Tester    5
    # Identical to the FMC config but PLLCFGR omits DIVR2EN (bit21).
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} 0x00008002
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL2DIVR} 0x010000C7
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00000040
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x04000000
    Wait For Log Entry    pll2r = 0

PLL2R Is Zero When PLL2 Off
    New RCC Machine
    Create Log Tester    5
    # DIVR2EN set, but PLL2ON (CR bit26) never asserted → output stays 0.
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} 0x00008002
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL2DIVR} 0x010000C7
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00200040
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x00000000
    Wait For Log Entry    pll2r = 0

PLL2 Fractional Adds Quarter Step Only With FRACEN
    New RCC Machine
    Create Log Tester    5
    # DIVN2=199 (×200) + FRACN=2048 (¼) → ×200.25 = 400.5 MHz; ÷2 = 200.25 MHz.
    # PLL2FRACEN (bit4) must be set for the ¼ to count.
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} 0x00008002
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL2DIVR} 0x010000C7
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL2FRACR} 0x00004000
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00200050
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x04000000
    Wait For Log Entry    pll2r = 200250000

FMC Mux Defaults To Hclk3
    New RCC Machine
    Create Log Tester    5
    # No D1CCIPR write (FMCSEL=00 = rcc_hclk3). sys_ck 400 MHz, HPRE=1 → 400 MHz.
    Configure PLL1 And Switch    0x00000022    0x00000263
    Wait For Log Entry    fmc_ker = 400000000

FMC Mux Per Ck Is Reported Unmodelled
    New RCC Machine
    Create Log Tester    5
    Configure PLL2 FMC
    # FMCSEL = 11 (per_ck) — not modelled; returns 0 with a stated warning.
    Execute Command    sysbus WriteDoubleWord ${RCC_D1CCIPR} 0x00000003
    Wait For Log Entry    does not compute

# --- PLL3 → SAI1 kernel clock (RM0433 §8.7.13/14, D2CCIP1R §9.5.45) ---
PLL3P 48 MHz Drives SAI1 Kernel Clock
    New RCC Machine
    Create Log Tester    5
    Configure PLL3 SAI
    # SAI1SEL = 010 → pll3_p_ck.
    Execute Command    sysbus WriteDoubleWord ${RCC_D2CCIP1R} 0x00000002
    Wait For Log Entry    pll3p = 48000000
    Wait For Log Entry    sai1_ker = 48000000

PLL3P Is Zero Without DIVP3EN
    New RCC Machine
    Create Log Tester    5
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} 0x00800002
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL3DIVR} 0x00000EBF
    # PLLCFGR has RGE but no DIVP3EN (bit22).
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00000400
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x10000000
    Wait For Log Entry    pll3p = 0

SAI1 Mux Can Select PLL2P
    New RCC Machine
    Create Log Tester    5
    # PLL2 with DIVP2EN (bit19) → pll2_p_ck = 400 MHz VCO ÷2 = 200 MHz; SAI1SEL=001.
    # DIVN2=199 (×200), DIVP2 field=1 (÷2) → 0xC7 | (1<<9) = 0x2C7.
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} 0x00008002
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL2DIVR} 0x000002C7
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00080040
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x04000000
    Execute Command    sysbus WriteDoubleWord ${RCC_D2CCIP1R} 0x00000001
    Wait For Log Entry    sai1_ker = 200000000

# --- RM0433 §8.7.13 range validation ---
PLL2 Warns When VCO Input Outside RGE Band
    New RCC Machine
    Create Log Tester    5
    # DIVM2=1 → vcoIn = 16 MHz, but PLL2RGE=01 (2–4 MHz) → out-of-band warning.
    # DIVN2=25 (×25) → 400 MHz VCO; DIVR2 ÷2 → 200 MHz (frequency still computed).
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} 0x00001002
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL2DIVR} 0x01000018
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00200040
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x04000000
    Wait For Log Entry    outside PLLxRGE band
