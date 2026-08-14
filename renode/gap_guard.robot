*** Settings ***
Documentation    Verify the GapGuard: a firmware access to an UNMAPPED RAM gap
...              (DTCM→AXI) must fault like silicon, not be silently swallowed by
...              Renode. gap-exerciser writes STARTED, touches 0x2100_0000, then
...              writes DONE — with the guard it CPU-aborts before DONE.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${APP}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/gap-exerciser
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${M_STARTED}     0x2001F000
${M_DONE}        0x2001F004

*** Test Cases ***
Gap Access Faults Like Silicon
    Execute Command    mach create "gap-exerciser"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision Gap Guards
    Execute Command    sysbus LoadELF @${APP}
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:00.010"

    ${started}=    Execute Command    sysbus ReadDoubleWord ${M_STARTED}
    Should Contain    ${started}    0x00000001    firmware never started
    # The gap write must have faulted BEFORE the DONE sentinel was written.
    ${done}=    Execute Command    sysbus ReadDoubleWord ${M_DONE}
    Should Not Contain    ${done}    0x0000D09E    gap access did NOT fault — Renode swallowed it (guard broken)
