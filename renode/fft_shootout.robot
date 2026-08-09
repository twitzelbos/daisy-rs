*** Settings ***
Documentation    Functional smoke for the FFT shootout firmware. Runs each
...              real-FFT entrant (ours + microfft) at 256/512/1024/2048 on the
...              emulated Cortex-M7 and proves each executes to completion WITHOUT
...              FAULTING — a DTCM stage bitmask gets a bit per completed bench,
...              and we assert all eight bits plus the done sentinel.
...
...              It deliberately does NOT rank by the cycle VALUES: Renode is a
...              functional translator (CYCCNT ≈ instruction count, not real M7
...              pipeline/cache/FPU latency — see feedback_renode_timing_fidelity).
...              The authoritative ranking is read from the SAME array on hardware
...              via probe-rs. Here we only additionally confirm CYCCNT advanced
...              (every entrant's marker is non-zero).
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${ELF}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/fft-shootout
# Results array (DTCM). Must match src/main.rs R_* indices.
${R_RESET}       0x2001F000
${R_STAGES}      0x2001F004
${R_MINE256}     0x2001F010
${R_MINE512}     0x2001F014
${R_MINE1024}    0x2001F018
${R_MINE2048}    0x2001F01C
${R_MFFT256}     0x2001F020
${R_MFFT512}     0x2001F024
${R_MFFT1024}    0x2001F028
${R_MFFT2048}    0x2001F02C
${R_DONE}        0x2001F03C
# ours ×4 + microfft ×4 = eight stage bits.
${ALL_STAGES}    0x000000FF

*** Keywords ***
Read Reg
    [Arguments]    ${addr}
    ${v}=    Execute Command    sysbus ReadDoubleWord ${addr}
    ${i}=    Convert To Integer    ${v.strip()}    16
    [Return]    ${i}

*** Test Cases ***
Every FFT Entrant Executes On The M7 Without Faulting
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    # Clock-driven DWT so CYCCNT actually advances during the run.
    Provision Clocked RCC And DWT
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    emulation RunFor "00:00:02.000"

    ${reset}=    Read Reg    ${R_RESET}
    Should Be Equal As Integers    ${reset}    1    main was not reached

    ${stages}=    Read Reg    ${R_STAGES}
    Should Be Equal As Integers    ${stages}    ${ALL_STAGES}
    ...    not all FFT entrants ran to completion (stage bitmask ${stages} != ${ALL_STAGES})

    ${done}=    Read Reg    ${R_DONE}
    Should Be Equal As Integers    ${done}    0xD09E    done sentinel not written

DWT Integration Is Live End-To-End
    [Documentation]    Every entrant's cycle marker comes back non-zero, proving
    ...                CYCCNT ran and bracketed real execution. These VALUES are
    ...                NOT a ranking (functional translator) — the real numbers are
    ...                read on hardware.
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision Clocked RCC And DWT
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    emulation RunFor "00:00:02.000"

    FOR    ${addr}    IN    ${R_MINE256}    ${R_MINE512}    ${R_MINE1024}    ${R_MINE2048}    ${R_MFFT256}    ${R_MFFT512}    ${R_MFFT1024}    ${R_MFFT2048}
        ${c}=    Read Reg    ${addr}
        Should Be True    ${c} > 0    an entrant measured 0 cycles — CYCCNT did not advance for ${addr}
    END
