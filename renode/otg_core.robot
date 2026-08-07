*** Settings ***
Documentation    OTG (Synopsys DWC_OTG) global core register behaviour —
...              RM0433 §59.15.1-3. Phase 1: core soft-reset + FIFO-flush
...              self-clear, AHB-idle, current-mode (device), core ID, and the
...              GINTSTS/GINTMSK/GAHBCFG interrupt-status mechanism. Each is the
...              behaviour synopsys-usb-otg's core-init depends on.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
# OTG2 global registers @ 0x40080000 (RM0433 §59.15).
${GAHBCFG}       0x40080008
${GRSTCTL}       0x40080010
${GINTSTS}       0x40080014
${GINTMSK}       0x40080018
${CID}           0x4008003C

*** Keywords ***
Provision OTG Machine
    Execute Command    mach create "otg"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision USB OTG

Read OTG Reg
    [Arguments]    ${addr}
    ${v}=    Execute Command    sysbus ReadDoubleWord ${addr}
    ${i}=    Convert To Integer    ${v.strip()}    16
    [Return]    ${i}

Bit Should Be Set
    [Arguments]    ${val}    ${bit}    ${msg}
    ${b}=    Evaluate    (${val} >> ${bit}) & 1
    Should Be Equal As Integers    ${b}    1    ${msg}

Bit Should Be Clear
    [Arguments]    ${val}    ${bit}    ${msg}
    ${b}=    Evaluate    (${val} >> ${bit}) & 1
    Should Be Equal As Integers    ${b}    0    ${msg}

*** Test Cases ***
AHB Master Reads Idle
    [Documentation]    GRSTCTL.AHBIDL (bit 31) must read 1; core-init spins on it.
    Provision OTG Machine
    ${g}=    Read OTG Reg    ${GRSTCTL}
    Bit Should Be Set    ${g}    31    AHBIDL never idle — core init would hang

Core Soft Reset Self Clears
    [Documentation]    GRSTCTL.CSRST (bit 0) set by software, cleared by hardware
    ...                when the reset completes (§59.15.2).
    Provision OTG Machine
    Execute Command    sysbus WriteDoubleWord ${GRSTCTL} 0x00000001
    ${g}=    Read OTG Reg    ${GRSTCTL}
    Bit Should Be Clear    ${g}    0    CSRST did not self-clear — init would hang

FIFO Flush Bits Self Clear
    [Documentation]    GRSTCTL.RXFFLSH (bit 4) / TXFFLSH (bit 5) self-clear once
    ...                the flush completes (§59.15.2).
    Provision OTG Machine
    Execute Command    sysbus WriteDoubleWord ${GRSTCTL} 0x00000010
    ${g}=    Read OTG Reg    ${GRSTCTL}
    Bit Should Be Clear    ${g}    4    RXFFLSH did not self-clear
    Execute Command    sysbus WriteDoubleWord ${GRSTCTL} 0x00000020
    ${g}=    Read OTG Reg    ${GRSTCTL}
    Bit Should Be Clear    ${g}    5    TXFFLSH did not self-clear

Current Mode Is Device
    [Documentation]    GINTSTS.CMOD (bit 0) = 0 (device). The HAL waits for this
    ...                after forcing device mode.
    Provision OTG Machine
    ${i}=    Read OTG Reg    ${GINTSTS}
    Bit Should Be Clear    ${i}    0    CMOD reports host mode, not device

Core ID Reads H7 Value
    [Documentation]    CID = 0x4F54310A — the H7 arm of synopsys-usb-otg's
    ...                core_id match; a wrong value takes the wrong config path.
    Provision OTG Machine
    ${cid}=    Execute Command    sysbus ReadDoubleWord ${CID}
    Should Contain    ${cid}    0x4F54310A

Interrupt Status Is Write One To Clear
    [Documentation]    GINTSTS flags are set by hardware events and cleared by
    ...                writing 1 (§59.15.3). RaiseEvent injects a bus-reset
    ...                (USBRST, bit 12) like the hardware would.
    Provision OTG Machine
    Execute Command    otg2 RaiseEvent 0x00001000
    ${i}=    Read OTG Reg    ${GINTSTS}
    Bit Should Be Set    ${i}    12    USBRST event not latched in GINTSTS
    Execute Command    sysbus WriteDoubleWord ${GINTSTS} 0x00001000
    ${i}=    Read OTG Reg    ${GINTSTS}
    Bit Should Be Clear    ${i}    12    USBRST not cleared by write-1-to-clear
