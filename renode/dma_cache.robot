*** Settings ***
Documentation    Functional smoke test for dma-cache-exerciser: prove the
...              firmware's DMA1 memory-to-memory programming actually moves data
...              (source→dest, TCIF completion, markers reach DONE). This is the
...              part Renode CAN validate.
...
...              What it deliberately does NOT prove: the D-cache/DMA coherency
...              DIVERGENCE. Renode has no cache model, and a firmware-kicked DMA
...              copy runs in the CPU's context (so the CacheCoherencyChecker
...              can't classify it foreign) — so both the buggy and the correct
...              variant read the FRESH value here. On silicon the buggy markers
...              hold the STALE value instead; that is the whole point of running
...              this firmware on hardware (docs/hardware-tests §6). The sim-side
...              coherency proof stays with cache_coherency.robot +
...              CacheCoherencyChecker. See docs/renode-fidelity.md §1.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}          ${CURDIR}/daisy_seed.repl
${ELF}               ${CURDIR}/../target/thumbv7em-none-eabihf/release/dma-cache-exerciser
# DTCM markers (see src/main.rs).
${M_DMA_OK}          0x2001F000
${M_STALE_BUGGY}     0x2001F004
${M_STALE_CORRECT}   0x2001F008
${M_DIRTY_BUGGY}     0x2001F00C
${M_DIRTY_CORRECT}   0x2001F010
${M_DONE}            0x2001F014
# Patterns (see src/main.rs).
${P_SANITY}          0x11111111
${P1_NEW}            0xC0FFEE01
${P2_DIRTY}          0xD1547000
${DONE}              0x0000D09E

*** Test Cases ***
DMA Mem2Mem Copies And Completes All Phases
    [Documentation]    Boot the exerciser and let it run every phase to DONE,
    ...                then read back the DTCM markers.
    Execute Command    mach create "dma-cache"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:00.05"

    # The firmware ran to completion.
    ${done}=    Execute Command    sysbus ReadDoubleWord ${M_DONE}
    Should Contain    ${done}    ${DONE}

    # DMA1 mem2mem actually moved data: the sanity copy landed in SINK.
    ${ok}=    Execute Command    sysbus ReadDoubleWord ${M_DMA_OK}
    Log    M_DMA_OK: ${ok.strip()}
    Should Contain    ${ok}    ${P_SANITY}

    # Correct-variant markers are the fresh values in BOTH sim and on HW.
    ${sc}=    Execute Command    sysbus ReadDoubleWord ${M_STALE_CORRECT}
    Should Contain    ${sc}    ${P1_NEW}
    ${dc}=    Execute Command    sysbus ReadDoubleWord ${M_DIRTY_CORRECT}
    Should Contain    ${dc}    ${P2_DIRTY}

Buggy Variants Read Fresh In Sim No Cache Model
    [Documentation]    Pin the documented fidelity boundary: because Renode has
    ...                no cache, the buggy markers hold the FRESH value here, i.e.
    ...                they EQUAL the correct markers. On silicon they would hold
    ...                the stale value (P1_OLD / P2_BASE) and differ — the whole
    ...                reason the firmware exists. If a future Renode gained a
    ...                cache model this test would flip, flagging that the sim
    ...                boundary moved.
    Execute Command    mach create "dma-cache-boundary"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:00.05"

    ${sb}=    Execute Command    sysbus ReadDoubleWord ${M_STALE_BUGGY}
    Log    M_STALE_BUGGY (sim=fresh, HW=stale): ${sb.strip()}
    Should Contain    ${sb}    ${P1_NEW}
    ${db}=    Execute Command    sysbus ReadDoubleWord ${M_DIRTY_BUGGY}
    Log    M_DIRTY_BUGGY (sim=fresh, HW=stale): ${db.strip()}
    Should Contain    ${db}    ${P2_DIRTY}
