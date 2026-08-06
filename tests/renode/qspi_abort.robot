*** Settings ***
Documentation    Verify our patched STM32H7_QuadSPI_Fixed correctly
...              self-clears CR.ABORT (bit 1) per RM0433 §23.6.1. The
...              upstream Renode STM32H7_QuadSPI leaves ABORT set after
...              a write, which hangs stm32h7xx-hal's exit_memory_mapped
...              poll `while qspi.cr.read().abort().bit_is_set() {}`.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${QSPI_CR}       0x52005000

*** Test Cases ***
QSPI CR ABORT Self Clears On Read
    [Documentation]    Write 1 to QSPI_CR.ABORT (bit 1). Read back. Must
    ...                come back with bit 1 clear because RM0433 says the
    ...                hardware self-clears the bit once the abort finishes,
    ...                which in Renode is effectively "immediately".
    Execute Command    mach create "qspi-abort"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs

    # Enable QSPI (CR.EN = bit 0) so the peripheral is operational, and
    # simultaneously request ABORT (CR.ABORT = bit 1). Then read back.
    Execute Command    sysbus WriteDoubleWord ${QSPI_CR} 0x00000003
    ${cr}=             Execute Command    sysbus ReadDoubleWord ${QSPI_CR}
    ${cr_int}=         Convert To Integer    ${cr.strip()}    16
    Log To Console     \nQSPI_CR after write 0x3: ${cr.strip()} (${cr_int})

    # bit 1 must be 0 (ABORT self-cleared); bit 0 (Enable) is a normal
    # RW flag so it stays set.
    ${abort_bit}=      Evaluate    ${cr_int} & 0x2
    ${enable_bit}=     Evaluate    ${cr_int} & 0x1
    Should Be Equal As Integers    ${abort_bit}     0
    Should Be Equal As Integers    ${enable_bit}    1
