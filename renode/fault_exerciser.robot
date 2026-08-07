*** Settings ***
Documentation    Verify Renode's Cortex-M7 exception model against the vector
...              table by running the fault-exerciser firmware, which drives
...              each vector and records a marker in DTCM:
...                reset → main reached, SysTick (exc 15), an external NVIC
...                interrupt (TIM2), and a MemManage fault (exc 4) via an MPU
...                no-access region that the handler recovers from.
...              This turns "the architecture is modelled" into "the vectors
...              actually fire and return", and confirms tlib enforces the MPU.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${ELF}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/fault-exerciser
${M_RESET}       0x2001F000
${M_SYSTICK}     0x2001F004
${M_TIM2}        0x2001F008
${M_MEMMANAGE}   0x2001F00C
${M_DONE}        0x2001F010
${M_RECOVERED}   0x2001F014

*** Test Cases ***
All Fault And Interrupt Vectors Fire
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Execute Command    sysbus LoadELF @${ELF}
    # RunFor starts from paused, runs the given virtual time, then pauses —
    # long enough for main to drive every vector synchronously.
    Execute Command    emulation RunFor "00:00:00.010"

    ${done}=    Execute Command    sysbus ReadDoubleWord ${M_DONE}
    Log    done sentinel: ${done.strip()}
    Should Contain    ${done}    0x0000D09E

    ${reset}=    Execute Command    sysbus ReadDoubleWord ${M_RESET}
    Should Contain    ${reset}    0x00000001

    ${systick}=    Execute Command    sysbus ReadDoubleWord ${M_SYSTICK}
    Should Contain    ${systick}    0x00000001

    ${tim2}=    Execute Command    sysbus ReadDoubleWord ${M_TIM2}
    Should Contain    ${tim2}    0x00000001

    ${mm}=    Execute Command    sysbus ReadDoubleWord ${M_MEMMANAGE}
    Should Contain    ${mm}    0x00000001

    # Recovery proof: after the MemManage handler disabled the MPU and
    # returned, the faulting load re-executed and read the primed value.
    ${rec}=    Execute Command    sysbus ReadDoubleWord ${M_RECOVERED}
    Should Contain    ${rec}    0x00C0FFEE
