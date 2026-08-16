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

Rev V IDCODE Boots At 480 MHz Under VOS0
    [Documentation]    clocks::init reads DBGMCU_IDCODE (0x5C00_1000) and, on a
    ...                rev-V H750 (REV_ID ≥ 0x2003, DEV_ID 0x450), takes the
    ...                480 MHz / VOS0 overdrive path. Plant a rev-V IDCODE, boot,
    ...                and assert the HAL's iterative PLL1 builder lands
    ...                sys_ck = 480 MHz — while PLL2R (FMC) stays 200 MHz. This
    ...                exercises the 480/VOS0 path, which Renode could never reach
    ...                before (an unmodelled IDCODE read returned 0 → always 400).
    Execute Command    mach create "daisy-clocks-revv"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision Clocked RCC And DWT
    Create Log Tester    10
    Execute Command    sysbus LoadELF @${BOOTLOADER}
    # rev V: REV_ID=0x2003 (bits 31:16), DEV_ID=0x450 (bits 11:0).
    Execute Command    sysbus WriteDoubleWord 0x5C001000 0x20030450
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:00.1"
    # One match (Wait For Log Entry is forward-only): sys_ck 480 MHz via PLL1P,
    # while PLL2R = 200 MHz (FMC kernel) and PLL3P ≈ 49.152 MHz (SAI) stay put —
    # PLL1's overdrive VCO doesn't perturb the independent PLL2/PLL3 kernels.
    Wait For Log Entry    sys_ck = 480000000 pll1p = 480000000 pll1q = 0 pll1r = 240000000 pll2p = 40000000 pll2q = 0 pll2r = 200000000 pll3p = 4915
    # Hand-off guard word = sys_ck = 480000000 = 0x1C9C3800.
    ${guard}=    Execute Command    sysbus ReadDoubleWord 0x38800004
    Should Contain    ${guard}    0x1C9C3800

Rev Y IDCODE Stays At 400 MHz
    [Documentation]    A rev-Y H750 has DEV_ID 0x450 but REV_ID 0x1003 — below
    ...                the rev-V threshold — so the gate must KEEP it at 400 MHz /
    ...                VOS1 (forcing VOS0 on rev Y hangs the overdrive handshake on
    ...                silicon). Plant a rev-Y IDCODE and assert sys_ck = 400 MHz,
    ...                proving the REV_ID check (not just DEV_ID) gates the path.
    Execute Command    mach create "daisy-clocks-revy"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision Clocked RCC And DWT
    Create Log Tester    10
    Execute Command    sysbus LoadELF @${BOOTLOADER}
    # rev Y: REV_ID=0x1003, DEV_ID=0x450 — DEV_ID matches but REV_ID is too low.
    Execute Command    sysbus WriteDoubleWord 0x5C001000 0x10030450
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:00.1"
    Wait For Log Entry    sys_ck = 400000000 pll1p = 400000000
