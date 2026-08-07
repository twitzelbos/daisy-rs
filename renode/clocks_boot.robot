*** Settings ***
Documentation    End-to-end clock validation: boot the real bootloader (which
...              runs `daisy_bsp::clocks::init`) under the clock-COMPUTING RCC
...              model, and assert the HAL's PLL builder actually lands PLL2R =
...              200 MHz (FMC kernel), a non-trivial PLL3P (SAI kernel) and
...              sys_ck = 400 MHz. This closes the loop between the HAL clock
...              config and the RM0433 clock math — coverage that the isolated
...              register tests (rcc_clock.robot) and the LED-blink boot tests
...              each miss.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${BOOTLOADER}    ${CURDIR}/../target/renode/thumbv7em-none-eabihf/release/daisy-boot
${PLATFORM}      ${CURDIR}/daisy_seed.repl

*** Test Cases ***
Bootloader clocks init lands PLL2R And SysCk
    Execute Command    mach create "daisy-clocks"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    # Peripheral stubs (FLASH/PWR/SYSCFG/QSPI) so clocks::init + the HAL freeze
    # run, then swap the stock RCC for the clock-computing model.
    Apply H7 Peripheral Stubs
    Provision Clocked RCC And DWT
    Create Log Tester    10
    Execute Command    sysbus LoadELF @${BOOTLOADER}
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:00.1"
    # Assert the fully-configured clock tree in ONE match (Wait For Log Entry is
    # forward-only, so the final settled line must be matched as a single entry):
    #   sys_ck 400 MHz, PLL2R = 200 MHz (FMC kernel), and PLL3P ≈ 49.152 MHz
    #   (SAI kernel; `4915` tolerates the fractional Hz).
    #   PLL2P = 40 MHz: it exists only to enable PLL2 (for PLL2R) and is the
    #   default ADC kernel clock, which is capped at 80 MHz at VOS1 (RM0433
    #   Table 59) — at 200 MHz the HAL's Adc::adc1 assert panics during ADC
    #   bring-up. It shares PLL2's 400 MHz VCO (DIVP=10), so PLL2R stays 200 MHz.
    #   This pins clocks::init's ADC-safe PLL2 config; a regression that raises
    #   PLL2P above 80 MHz is caught here rather than hanging the app on silicon.
    Wait For Log Entry    sys_ck = 400000000 pll1p = 400000000 pll1q = 0 pll1r = 200000000 pll2p = 40000000 pll2q = 0 pll2r = 200000000 pll3p = 4915

    # The bootloader stashed the frozen CoreClocks into Backup SRAM (0x38800000)
    # for the XIP app to recover: a magic word + the sys_ck guard (400 MHz =
    # 0x17D78400). Reading them back proves the hand-off round-trips.
    ${magic}=    Execute Command    sysbus ReadDoubleWord 0x38800000
    Should Contain    ${magic}    0xDA15C0C0
    ${guard}=    Execute Command    sysbus ReadDoubleWord 0x38800004
    Should Contain    ${guard}    0x17D78400
