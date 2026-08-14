*** Settings ***
Documentation    Run the REAL daisy_bsp::sdram::init() firmware (via sdram-exerciser)
...              against the STM32H7_FMC_SDRAM gating model and prove the 64 MiB
...              window becomes usable: the actual init() must drive the JEDEC
...              power-up command sequence the model gates on, then a data sweep
...              must round-trip. Complements sdram_fmc.robot, which drives the
...              model with HAND-WRITTEN command words — this proves the firmware
...              produces the same, correctly-ordered sequence. (Not a DRAM cell
...              test: Renode has a perfect backing store; cell/timing faults are
...              covered by the daisy-sdram-test CDC app on hardware.)
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}    ${CURDIR}/daisy_seed.repl
${ELF}         ${CURDIR}/../target/thumbv7em-none-eabihf/release/sdram-exerciser
${M_STAGE}     0x2001F000
${M_ERRORS}    0x2001F004
${M_DONE}      0x2001F008

*** Test Cases ***
Real sdram init Brings Up The Window And Data Round-Trips
    Execute Command    mach create "sdram-init"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision FMC SDRAM
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:00.100"

    # Completion sentinel (written last), so the reads below are consistent.
    ${done}=    Execute Command    sysbus ReadDoubleWord ${M_DONE}
    Should Contain    ${done}    0x0000D09E
    # Last stage reached = dense sweep done → init() ran + both sweeps completed.
    ${stage}=    Execute Command    sysbus ReadDoubleWord ${M_STAGE}
    Should Contain    ${stage}    0x5D2A1104
    # Zero read-back mismatches. If init() had driven a wrong/mis-ordered command
    # sequence the window would stay gated (reads return 0) → nonzero here.
    ${errs}=    Execute Command    sysbus ReadDoubleWord ${M_ERRORS}
    Should Contain    ${errs}    0x00000000
