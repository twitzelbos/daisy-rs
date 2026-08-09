*** Settings ***
Documentation    Prove the CacheCoherencyChecker catches the M7 D-cache / DMA
...              non-coherency bug class that is otherwise INVISIBLE in Renode
...              (the Cortex-M translator has no cache, so cacheability and the
...              SCB clean/invalidate ops are inert). See docs/renode-fidelity.md
...              §1 — this is the #1 sim-fidelity gap and the exact class that bit
...              this project on real hardware (QSPI/DMA buffer corruption).
...
...              The cache-coherency-exerciser drives BOTH M7 coherency hazards
...              against a buffer at 0x3000_0000, in one firmware run:
...                Phase 1 (stale READ): CPU caches the buffer; the harness
...                  injects a foreign-master (DMA) write — a monitor sysbus write
...                  has no current CPU, so the checker classifies it foreign and
...                  marks the cached line stale — then FIX1 tells firmware whether
...                  to invalidate (DCIMVAC) before re-reading.
...                Phase 2 (stale DMA read of a DIRTY line): CPU writes the buffer
...                  (line dirty), FIX2 tells firmware whether to clean (DCCMVAC),
...                  then the harness performs a foreign-master (DMA) read.
...
...              Buggy variant (no maintenance) must raise each violation; correct
...              variant (maintenance by MVA) must raise NEITHER — no false
...              positive. The violation COUNTERS are the signal: Renode keeps
...              backing memory coherent (no real cache), so the read-back value is
...              always fresh; only the counters reflect what silicon would do.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${ELF}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/cache-coherency-exerciser
${BUF}           0x30000000
${M_CACHED}      0x2001F000
${M_GO1}         0x2001F004
${M_FIX1}        0x2001F008
${M_RESULT}      0x2001F00C
${M_DONE1}       0x2001F010
${M_DIRTY}       0x2001F014
${M_GO2}         0x2001F018
${M_FIX2}        0x2001F01C
${M_DONE2}       0x2001F020
${M_FIXI}        0x2001F024
${M_DONE3}       0x2001F028

*** Test Cases ***
Missing Cache Maintenance Is Flagged In Both Directions
    [Documentation]    Buggy variant, no maintenance anywhere. The checker must
    ...                report: one stale CPU read (phase 1, missing DCIMVAC); one
    ...                stale DMA read of a dirty line (phase 2, missing DCCMVAC);
    ...                and two stale instruction fetches (phase 3, code written
    ...                but not cleaned to PoU; phase 4, code modified without
    ...                ICIALLU).
    ${chk}=    Provision And Load
    Run Both Phases    fix1=0    fix2=0    fixi=0

    ${stale}=    Execute Command    ${chk} StaleReadViolations
    Log    StaleReadViolations (buggy): ${stale.strip()}
    Should Contain    ${stale}    0x00000001
    ${dirty}=    Execute Command    ${chk} DirtyDmaReadViolations
    Log    DirtyDmaReadViolations (buggy): ${dirty.strip()}
    Should Contain    ${dirty}    0x00000001
    ${ifetch}=    Execute Command    ${chk} IStaleFetchViolations
    Log    IStaleFetchViolations (buggy): ${ifetch.strip()}
    Should Contain    ${ifetch}    0x00000002

Correct Cache Maintenance Produces No Violation And No False Positive
    [Documentation]    Correct variant: DCIMVAC before the re-read, DCCMVAC after
    ...                the write, and — for the code — DCCMVAU + ICIALLU before
    ...                each execute. Every maintenance watchpoint clears the line
    ...                state, so NONE of the three directions is flagged.
    ${chk}=    Provision And Load
    Run Both Phases    fix1=1    fix2=1    fixi=1

    ${stale}=    Execute Command    ${chk} StaleReadViolations
    Log    StaleReadViolations (correct): ${stale.strip()}
    Should Contain    ${stale}    0x00000000
    ${dirty}=    Execute Command    ${chk} DirtyDmaReadViolations
    Log    DirtyDmaReadViolations (correct): ${dirty.strip()}
    Should Contain    ${dirty}    0x00000000
    ${ifetch}=    Execute Command    ${chk} IStaleFetchViolations
    Log    IStaleFetchViolations (correct): ${ifetch.strip()}
    Should Contain    ${ifetch}    0x00000000

*** Keywords ***
Provision And Load
    [Documentation]    Fresh machine + checker overlay at 0x3000_0000, load the
    ...                exerciser. Returns the checker's monitor path.
    Execute Command    mach create "cache-coherency"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision Cache Coherency Checker
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    cpu VectorTableOffset 0x08000000
    RETURN    sysbus.cacheChecker

Run Both Phases
    [Documentation]    Drive the full two-phase handshake. Both FIX markers must
    ...                be set before the phase-1 release, because after GO1 the
    ...                firmware runs straight through phase 1 AND phase 2 (writing
    ...                the buffer and reading FIX2 to decide whether to clean) in
    ...                one slice — it only pauses again at the phase-2 GO2 spin.
    [Arguments]    ${fix1}    ${fix2}    ${fixi}
    # Phase 1 setup: let firmware cache the buffer and reach the GO1 spin.
    Execute Command    emulation RunFor "00:00:00.002"
    ${cached}=    Execute Command    sysbus ReadDoubleWord ${M_CACHED}
    Should Contain    ${cached}    0x00000001
    # Foreign-master (DMA) write of the CPU-cached line → line goes stale.
    Execute Command    sysbus WriteDoubleWord ${BUF} 0xDEADBEEF
    Execute Command    sysbus WriteDoubleWord ${M_FIX1} ${fix1}
    Execute Command    sysbus WriteDoubleWord ${M_FIX2} ${fix2}
    Execute Command    sysbus WriteDoubleWord ${M_FIXI} ${fixi}
    Execute Command    sysbus WriteDoubleWord ${M_GO1} 0x00000001
    # Firmware finishes phase 1, then runs phase 2 up to the GO2 spin (it has now
    # written the buffer — dirty — and, if fix2, cleaned it by MVA).
    Execute Command    emulation RunFor "00:00:00.002"
    ${done1}=    Execute Command    sysbus ReadDoubleWord ${M_DONE1}
    Should Contain    ${done1}    0x0000D09E
    ${dirty}=    Execute Command    sysbus ReadDoubleWord ${M_DIRTY}
    Should Contain    ${dirty}    0x00000001
    # Phase 2: foreign-master (DMA) read; flags a violation only if still dirty.
    # After GO2, firmware finishes phase 2 AND runs phases 3-4 (I-cache) straight
    # through — those need no handshake, only FIXI, already set above.
    Execute Command    sysbus ReadDoubleWord ${BUF}
    Execute Command    sysbus WriteDoubleWord ${M_GO2} 0x00000001
    Execute Command    emulation RunFor "00:00:00.002"
    ${done2}=    Execute Command    sysbus ReadDoubleWord ${M_DONE2}
    Should Contain    ${done2}    0x0000D09E
    ${done3}=    Execute Command    sysbus ReadDoubleWord ${M_DONE3}
    Should Contain    ${done3}    0x0000D09E
