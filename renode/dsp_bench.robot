*** Settings ***
Documentation    Functional smoke for the daisy-dsp cycle-bench firmware. This
...              runs each DSP processor (OnePole, Biquad, DelayLine, Env,
...              FdnReverb, Freeze, PadDrone) and the real FFT at 256/512/1024/
...              2048 on the emulated Cortex-M7 and proves each executes to
...              completion WITHOUT FAULTING — the firmware OR's a stage bit into
...              a DTCM bitmask after each returns, and we assert all eleven bits
...              plus the done sentinel.
...
...              It deliberately does NOT assert the cycles/block VALUES. Renode
...              is a functional translator: CYCCNT advances with virtual time
...              (~instruction count), not the real M7 pipeline/cache/FPU
...              latencies (see STM32H7_DWT_Clocked's "Fidelity boundary" header
...              and feedback_renode_timing_fidelity). The authoritative cycle
...              budget is read from the SAME array on real hardware via
...              probe-rs. Here we only additionally confirm the DWT integration
...              is live end-to-end: with the clock-driven DWT provisioned, the
...              measured markers come back non-zero (the counter ran).
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${ELF}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/dsp-bench
# Results array (DTCM). Must match src/main.rs R_* indices.
${R_RESET}       0x20018000
${R_ONEPOLE}     0x20018010
${R_BIQUAD}      0x20018014
${R_DELAY}       0x20018018
${R_FDNREVERB}   0x2001801C
${R_FREEZE}      0x20018020
${R_PADDRONE}    0x20018024
${R_ENV}         0x20018028
${R_STAGES}      0x2001802C
${R_DONE}        0x20018030
${R_FFT256}      0x20018034
${R_FFT512}      0x20018038
${R_FFT1024}     0x2001803C
${R_FFT2048}     0x20018040
# Seven processors (0x7F) + four real-FFT sizes (bits 7..10) = 0x7FF.
${ALL_STAGES}    0x000007FF

*** Keywords ***
Read Reg
    [Arguments]    ${addr}
    ${v}=    Execute Command    sysbus ReadDoubleWord ${addr}
    ${i}=    Convert To Integer    ${v.strip()}    16
    [Return]    ${i}

*** Test Cases ***
Every DSP Processor Executes On The M7 Without Faulting
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    # Clock-driven DWT so CYCCNT actually advances during the run.
    Provision Clocked RCC And DWT
    Execute Command    sysbus LoadELF @${ELF}
    # Enough virtual time for all seven benches (each measured ITERS times) to
    # run and the firmware to reach its wfi loop.
    Execute Command    emulation RunFor "00:00:02.000"

    ${reset}=    Read Reg    ${R_RESET}
    Should Be Equal As Integers    ${reset}    1    main was not reached

    ${stages}=    Read Reg    ${R_STAGES}
    Should Be Equal As Integers    ${stages}    ${ALL_STAGES}
    ...    not all DSP processors ran to completion (stage bitmask ${stages} != ${ALL_STAGES})

    ${done}=    Read Reg    ${R_DONE}
    Should Be Equal As Integers    ${done}    0xD09E    done sentinel not written

DWT Integration Is Live End-To-End
    [Documentation]    With the clock-driven DWT, the measured markers come back
    ...                non-zero — proving CYCCNT ran and the bench bracketed real
    ...                execution. These VALUES are NOT a budget (functional
    ...                translator); the real numbers are read on hardware.
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision Clocked RCC And DWT
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    emulation RunFor "00:00:02.000"

    FOR    ${addr}    IN    ${R_ONEPOLE}    ${R_BIQUAD}    ${R_DELAY}    ${R_FDNREVERB}    ${R_FREEZE}    ${R_PADDRONE}    ${R_ENV}    ${R_FFT256}    ${R_FFT512}    ${R_FFT1024}    ${R_FFT2048}
        ${c}=    Read Reg    ${addr}
        Should Be True    ${c} > 0    a processor measured 0 cycles — CYCCNT did not advance for ${addr}
    END
