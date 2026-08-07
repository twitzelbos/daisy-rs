*** Settings ***
Documentation    OTG enumeration — the whole stack, end to end. The real
...              usb-device CDC firmware runs against the STM32H7_OTG model while
...              this test plays the HOST side through the model's stimulus
...              hooks: a bus reset + speed-enum (USBRST/ENUMDNE), then the
...              SET_ADDRESS and SET_CONFIGURATION control transfers a host issues
...              during enumeration. The firmware's real stack must walk
...              Default → Addressed → Configured, and the assigned address must
...              land in DCFG.DAD — driven purely by the modelled register
...              behaviour (RM0433 §59.15), with no host controller in the loop.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${ELF}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/usb-enum-exerciser
${DCFG}          0x40080800
${MARK_STATE}    0x20010000
${MARK_POLLS}    0x20010004
${MARK_MAXSTATE}    0x20010008
# GINTSTS event bits.
${USBRST}        0x00001000
${ENUMDNE}       0x00002000
# SETUP packets, 8 bytes packed little-endian into (low, high) words.
# SET_ADDRESS(7):        00 05 07 00 00 00 00 00
${SETADDR_LOW}   0x00070500
${SETADDR_HIGH}  0x00000000
# SET_CONFIGURATION(1):  00 09 01 00 00 00 00 00
${SETCFG_LOW}    0x00010900
${SETCFG_HIGH}   0x00000000
# Device-state codes the firmware writes (state_code()).
${ST_DEFAULT}    0
${ST_ADDRESSED}  1
${ST_CONFIGURED}    2

*** Keywords ***
Read Mem
    [Arguments]    ${addr}
    ${v}=    Execute Command    sysbus ReadDoubleWord ${addr}
    ${i}=    Convert To Integer    ${v.strip()}    16
    [Return]    ${i}

Run A Little
    Execute Command    emulation RunFor "00:00:00.1"

*** Test Cases ***
Real usb-device Stack Enumerates To Configured
    Execute Command    mach create "otg-enum"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision Clocked RCC And DWT
    Provision USB OTG
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    cpu VectorTableOffset 0x08000000

    # Boot: clocks::init + UsbBus::new + device build, then into the poll loop.
    Execute Command    emulation RunFor "00:00:00.2"
    ${polls}=    Read Mem    ${MARK_POLLS}
    Should Be True    ${polls} > 0    firmware never reached its poll loop
    ${st}=    Read Mem    ${MARK_STATE}
    Should Be Equal As Integers    ${st}    ${ST_DEFAULT}    device did not start in Default

    # Host: bus reset + speed enumeration done → the stack resets and (re)arms
    # EP0. synopsys-usb-otg returns PollResult::Reset on ENUMDNE.
    Execute Command    otg2 RaiseEvent ${USBRST}
    Run A Little
    Execute Command    otg2 RaiseEvent ${ENUMDNE}
    Run A Little
    ${st}=    Read Mem    ${MARK_STATE}
    Should Be Equal As Integers    ${st}    ${ST_DEFAULT}    device left Default before addressing

    # Host: SET_ADDRESS(7). No data stage; the device sends the IN ZLP status,
    # the model completes it, and the stack advances to Addressed with the
    # address committed to DCFG.DAD (QUIRK_SET_ADDRESS_BEFORE_STATUS).
    Execute Command    otg2 ReceiveSetup ${SETADDR_LOW} ${SETADDR_HIGH}
    Run A Little
    ${dcfg}=    Read Mem    ${DCFG}
    ${dad}=    Evaluate    (${dcfg} >> 4) & 0x7F
    Should Be Equal As Integers    ${dad}    7    SET_ADDRESS did not land in DCFG.DAD
    ${maxst}=    Read Mem    ${MARK_MAXSTATE}
    Should Be True    ${maxst} >= ${ST_ADDRESSED}    stack never reached Addressed

    # Host: SET_CONFIGURATION(1) → IN ZLP status → Configured.
    Execute Command    otg2 ReceiveSetup ${SETCFG_LOW} ${SETCFG_HIGH}
    Run A Little
    ${maxst}=    Read Mem    ${MARK_MAXSTATE}
    Should Be Equal As Integers    ${maxst}    ${ST_CONFIGURED}    stack did not reach Configured
    ${st}=    Read Mem    ${MARK_STATE}
    Should Be Equal As Integers    ${st}    ${ST_CONFIGURED}    device is not Configured at rest
