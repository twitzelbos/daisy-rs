*** Settings ***
Documentation    Verify the GapGuard detects accesses to an UNMAPPED RAM gap
...              (DTCM→AXI) that stock Renode would silently swallow. gap-exerciser
...              writes to 0x2002_0000 (first address past DTCM); the guard's
...              access counter must be non-zero.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${APP}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/gap-exerciser
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${M_STARTED}     0x2001F000

*** Test Cases ***
Gap Access Is Detected
    Execute Command    mach create "gap-exerciser"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision Gap Guards
    Execute Command    sysbus LoadELF @${APP}
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:00.010"

    ${started}=    Execute Command    sysbus ReadDoubleWord ${M_STARTED}
    Should Contain    ${started}    0x00000001    firmware never started
    # The guard must have SEEN the gap access — stock Renode would swallow it.
    ${n}=    Execute Command    sysbus.gapDtcmAxi Accesses
    ${n_int}=    Evaluate    int(str('''${n}''').strip(), 0)
    Should Be True    ${n_int} > 0    guard saw no gap access — Renode swallowed it (guard broken)
