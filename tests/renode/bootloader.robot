*** Settings ***
Documentation    daisy-boot behaviour in Renode: alive-blink, service mode,
...              and jump-to-QSPI-app.
Suite Setup      Setup
Suite Teardown   Teardown
Test Setup       Reset Emulation
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Library          ${CURDIR}/daisy_seed_helpers.py

*** Variables ***
${SCENARIO}      ${CURDIR}/bootloader.resc
# System Control Block Vector Table Offset Register — where our jump code
# writes the app base before branching. Reading this after boot tells us
# whether the jump was taken.
${SCB_VTOR}      0xE000ED08
${QSPI_BASE}     0x90000000

*** Keywords ***
Boot Bootloader
    [Documentation]    Load the platform + bootloader ELF and create the
    ...                LED tester. Does NOT start emulation — tests that
    ...                need `emulation RunFor` must invoke it directly;
    ...                tests that want continuous LED tracking should
    ...                `Start Emulation` themselves after this.
    Execute Script    ${SCENARIO}
    Create LED Tester    sysbus.gpioPortC.userLed

Write Plausible Vector Table To Qspi
    [Documentation]    Plant a two-word "vector table" in the QSPI window:
    ...                MSP pointing into AXI SRAM, reset vector into QSPI
    ...                with the Thumb bit set. Both words satisfy the
    ...                heuristic in daisy_bsp::boot_check::is_plausible_image.
    Execute Command    sysbus WriteDoubleWord 0x90000000 0x24040000
    Execute Command    sysbus WriteDoubleWord 0x90000004 0x90000101
    # Give the "app" a Thumb infinite loop (b .) at 0x90000100 so the CPU
    # doesn't wander into 0xFFFFFFFF-land after the jump. Halfword 0xE7FE
    # = `b .` in Thumb.
    Execute Command    sysbus WriteWord 0x90000100 0xE7FE

*** Test Cases ***
Bootloader Toggles LED Four Times On Boot
    [Documentation]    Verify daisy-boot runs its 4-blink "alive" sequence.
    ...                Each cycle is 250 ms on + 250 ms off, total 2 s
    ...                (plus a small QSPI init tail).
    Boot Bootloader
    # First rising edge — LED high shortly after clock init.
    Assert LED State    true    timeout=1.0
    # Seven more toggles for the remaining alive-blink pattern.
    FOR    ${i}    IN RANGE    7
        Assert LED Is Blinking    timeout=1.0    testerId=sysbus.gpioPortC.userLed
    END

Bootloader Enters Service Mode When Qspi Is Blank
    [Documentation]    With QSPI zero-initialised, is_plausible_image()
    ...                rejects the vector table and we fall through to
    ...                the 10 Hz service-mode blink loop. Count several
    ...                fast toggles inside a short window.
    Boot Bootloader
    # Let the 4-blink alive phase pass (2 s virtual).
    Execute Command    emulation RunFor "00:00:02.500"
    # Service mode toggles every 50 ms. Expect ~10 in a 500 ms window.
    FOR    ${i}    IN RANGE    5
        Assert LED Is Blinking    timeout=0.100    testerId=sysbus.gpioPortC.userLed
    END

Bootloader Jumps To Qspi When Vector Table Is Plausible
    [Documentation]    With a valid-shaped vector table planted in QSPI,
    ...                is_plausible_image() returns true, jump_to_app()
    ...                runs, and SCB.VTOR is retargeted to 0x9000_0000.
    Execute Script    ${SCENARIO}
    Write Plausible Vector Table To Qspi
    Start Emulation
    # Run past the alive-blink window plus QSPI init tail.
    Execute Command    emulation RunFor "00:00:03.000"
    # VTOR should now read as the app base — the smoking gun that the
    # jump path was taken.
    ${vtor}=    Execute Command    sysbus ReadDoubleWord ${SCB_VTOR}
    Should Contain    ${vtor}    0x90000000
