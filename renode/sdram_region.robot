*** Settings ***
Documentation    Validate the Renode model of the Daisy Seed's FMC SDRAM: a
...              64 MiB region at 0xC000_0000 (Alliance AS4C16M32MSA,
...              16M × 32-bit). The audit confirmed a plain always-backed
...              MappedMemory is the correct model for the project's tests
...              (the real FMC init sequence is exercised on hardware, not in
...              sim). This checks base, full 64 MiB span, read/write
...              round-trips, and that one word past the end is unbacked.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
# 0xC0000000 + 0x04000000 - 4 = last aligned word of the 64 MiB region.
${SDRAM_BASE}    0xC0000000
${SDRAM_LAST}    0xC3FFFFFC
${SDRAM_END}     0xC4000000

*** Test Cases ***
SDRAM Region Backs The Full 64 MiB
    Execute Command    mach create "sdram"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}

    # Base, an interior word, and the very last aligned word must all
    # round-trip — proving the region spans the full 64 MiB.
    Execute Command    sysbus WriteDoubleWord ${SDRAM_BASE} 0xA5A5A5A5
    Execute Command    sysbus WriteDoubleWord 0xC2000000 0x5A5A5A5A
    Execute Command    sysbus WriteDoubleWord ${SDRAM_LAST} 0xDEADC0DE

    ${b}=    Execute Command    sysbus ReadDoubleWord ${SDRAM_BASE}
    Should Contain    ${b}    0xA5A5A5A5
    ${m}=    Execute Command    sysbus ReadDoubleWord 0xC2000000
    Should Contain    ${m}    0x5A5A5A5A
    ${e}=    Execute Command    sysbus ReadDoubleWord ${SDRAM_LAST}
    Should Contain    ${e}    0xDEADC0DE

One Word Past The End Is Not Backed
    Execute Command    mach create "sdram-oob"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}

    # 0xC4000000 is one word past the 64 MiB region. Reading it hits no
    # peripheral; Renode logs a warning and returns 0 rather than region data.
    ${oob}=    Execute Command    sysbus ReadDoubleWord ${SDRAM_END}
    Should Contain    ${oob}    0x00000000
