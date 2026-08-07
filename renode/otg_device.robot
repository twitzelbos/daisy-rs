*** Settings ***
Documentation    OTG device-mode register behaviour — RM0433 §59.15 (device
...              section). Phase 2: device address (DCFG.DAD), soft-disconnect
...              (DCTL.SDIS), enumerated-speed status (DSTS.ENUMSPD), and the
...              endpoint interrupt tree — per-endpoint DIEPINTx/DOEPINTx are
...              write-1-to-clear and aggregate through DAINT (masked by
...              DIEPMSK/DOEPMSK) up to GINTSTS.IEPINT/OEPINT (masked by
...              DAINTMSK), the exact path the device driver's ISR walks.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
# OTG2 device registers (RM0433 §59.15).
${GINTSTS}       0x40080014
${DCFG}          0x40080800
${DCTL}          0x40080804
${DSTS}          0x40080808
${DIEPMSK}       0x40080810
${DAINT}         0x40080818
${DAINTMSK}      0x4008081C
${DIEPINT0}      0x40080908

*** Keywords ***
Provision OTG Machine
    Execute Command    mach create "otg-dev"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision USB OTG

Read OTG Reg
    [Arguments]    ${addr}
    ${v}=    Execute Command    sysbus ReadDoubleWord ${addr}
    ${i}=    Convert To Integer    ${v.strip()}    16
    [Return]    ${i}

Field Should Be
    [Arguments]    ${val}    ${shift}    ${mask}    ${expected}    ${msg}
    ${f}=    Evaluate    (${val} >> ${shift}) & ${mask}
    Should Be Equal As Integers    ${f}    ${expected}    ${msg}

Bit Should Be Set
    [Arguments]    ${val}    ${bit}    ${msg}
    ${b}=    Evaluate    (${val} >> ${bit}) & 1
    Should Be Equal As Integers    ${b}    1    ${msg}

Bit Should Be Clear
    [Arguments]    ${val}    ${bit}    ${msg}
    ${b}=    Evaluate    (${val} >> ${bit}) & 1
    Should Be Equal As Integers    ${b}    0    ${msg}

*** Test Cases ***
Device Address Is Programmable
    [Documentation]    DCFG.DAD (bits 10:4) holds the device address assigned by
    ...                SET_ADDRESS; DSPD (bits 1:0) the speed.
    Provision OTG Machine
    # Program address 42 (0x2A) + Full-speed (DSPD=0b11).
    Execute Command    sysbus WriteDoubleWord ${DCFG} 0x000002A3
    ${d}=    Read OTG Reg    ${DCFG}
    Field Should Be    ${d}    4    0x7F    42    DCFG.DAD did not hold the device address

Soft Disconnect Is Observable
    [Documentation]    DCTL.SDIS (bit 1) drives the D+ pull-up: 1 = disconnected.
    Provision OTG Machine
    Execute Command    sysbus WriteDoubleWord ${DCTL} 0x00000002
    ${c}=    Read OTG Reg    ${DCTL}
    Bit Should Be Set    ${c}    1    SDIS not observable
    Execute Command    sysbus WriteDoubleWord ${DCTL} 0x00000000
    ${c}=    Read OTG Reg    ${DCTL}
    Bit Should Be Clear    ${c}    1    SDIS not cleared (soft reconnect)

Device Status Reports Full Speed
    [Documentation]    DSTS.ENUMSPD (bits 2:1) = 0b11 — Full speed via the
    ...                internal FS PHY (the Daisy's on-board USB).
    Provision OTG Machine
    ${s}=    Read OTG Reg    ${DSTS}
    Field Should Be    ${s}    1    0x3    3    ENUMSPD is not Full speed

Endpoint Interrupt Aggregates Through DAINT To GINTSTS And Is W1C
    [Documentation]    The full device interrupt tree: a DIEPINT0.XFRC event,
    ...                enabled by DIEPMSK + DAINTMSK, must show in DAINT.IEPINT0
    ...                and GINTSTS.IEPINT, and clearing DIEPINT0 (W1C) must
    ...                retract both.
    Provision OTG Machine
    # Unmask XFRC (DIEPMSK bit0) and IN-EP0 (DAINTMSK bit0).
    Execute Command    sysbus WriteDoubleWord ${DIEPMSK} 0x00000001
    Execute Command    sysbus WriteDoubleWord ${DAINTMSK} 0x00000001
    # Inject an IN-EP0 transfer-complete (DIEPINT0.XFRC, bit 0).
    Execute Command    otg2 RaiseEndpointInterrupt true 0 0x00000001
    ${daint}=    Read OTG Reg    ${DAINT}
    Bit Should Be Set    ${daint}    0    DAINT.IEPINT0 not set by the endpoint interrupt
    ${g}=    Read OTG Reg    ${GINTSTS}
    Bit Should Be Set    ${g}    18    GINTSTS.IEPINT not aggregated from DAINT
    # Clear the endpoint interrupt (W1C) → tree retracts.
    Execute Command    sysbus WriteDoubleWord ${DIEPINT0} 0x00000001
    ${daint}=    Read OTG Reg    ${DAINT}
    Bit Should Be Clear    ${daint}    0    DAINT.IEPINT0 not cleared after W1C
    ${g}=    Read OTG Reg    ${GINTSTS}
    Bit Should Be Clear    ${g}    18    GINTSTS.IEPINT still set after clearing the endpoint
