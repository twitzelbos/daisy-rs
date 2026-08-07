*** Settings ***
Documentation    OTG packet path — RM0433 §59.15.6 (device receive) + transmit.
...              Phase 3: the RxFIFO status-pop protocol a received SETUP/OUT
...              packet drives (GINTSTS.RXFLVL, GRXSTSR peek vs GRXSTSP pop, and
...              the DFIFO data readback), and the IN path (DIEPCTL.EPENA +
...              writing the TX FIFO completing the transfer as DIEPINTx.XFRC).
...              This is exactly the sequence synopsys-usb-otg's poll() walks:
...              GRXSTSR → GRXSTSP → fill_from_fifo, and write() → XFRC.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${GINTSTS}       0x40080014
${GRXSTSR}       0x4008001C
${GRXSTSP}       0x40080020
${DIEPMSK}       0x40080810
${DAINT}         0x40080818
${DAINTMSK}      0x4008081C
${DIEPCTL0}      0x40080900
${DIEPINT0}      0x40080908
${DIEPTSIZ0}     0x40080910
${DFIFO0}        0x40081000
# GET_DESCRIPTOR(device, wLength=0x40): 80 06 00 01 00 00 40 00, packed LE.
${SETUP_LOW}     0x01000680
${SETUP_HIGH}    0x00400000

*** Keywords ***
Provision OTG Machine
    Execute Command    mach create "otg-fifo"
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

Model Call
    [Arguments]    ${call}
    ${v}=    Execute Command    otg2 ${call}
    ${i}=    Convert To Integer    ${v.strip()}    16
    [Return]    ${i}

*** Test Cases ***
Received Setup Packet Drives The RxFIFO Status-Pop Protocol
    [Documentation]    A SETUP packet asserts RXFLVL and pushes a SETUP-data
    ...                status (PKTSTS=0x06, BCNT=8) that GRXSTSR peeks without
    ...                consuming and GRXSTSP pops, exposing the 8 data bytes on
    ...                the DFIFO, then a SETUP-complete status (PKTSTS=0x04).
    Provision OTG Machine
    Execute Command    otg2 ReceiveSetup ${SETUP_LOW} ${SETUP_HIGH}

    ${g}=    Read OTG Reg    ${GINTSTS}
    Bit Should Be Set    ${g}    4    RXFLVL not asserted by the received packet

    # GRXSTSR peeks: PKTSTS=0x06, EPNUM=0, BCNT=8 — and is non-destructive.
    ${r}=    Read OTG Reg    ${GRXSTSR}
    Field Should Be    ${r}    17    0xF    6     GRXSTSR PKTSTS is not SETUP-data-received
    Field Should Be    ${r}    4     0x7FF    8    GRXSTSR BCNT is not 8
    Field Should Be    ${r}    0     0xF    0     GRXSTSR EPNUM is not 0
    ${r2}=    Read OTG Reg    ${GRXSTSR}
    Should Be Equal As Integers    ${r}    ${r2}    GRXSTSR peek consumed the status word

    # GRXSTSP pops the same status word.
    ${p}=    Read OTG Reg    ${GRXSTSP}
    Field Should Be    ${p}    17    0xF    6    GRXSTSP did not return the SETUP-data status

    # The 8 SETUP bytes come off the DFIFO, low word first.
    ${w0}=    Read OTG Reg    ${DFIFO0}
    Should Be Equal As Integers    ${w0}    ${SETUP_LOW}     first SETUP word wrong
    ${w1}=    Read OTG Reg    ${DFIFO0}
    Should Be Equal As Integers    ${w1}    ${SETUP_HIGH}    second SETUP word wrong

    # Next in the FIFO: the SETUP-complete status (PKTSTS=0x04).
    ${c}=    Read OTG Reg    ${GRXSTSR}
    Field Should Be    ${c}    17    0xF    4    trailing status is not SETUP-complete
    Read OTG Reg    ${GRXSTSP}

    # FIFO drained → RXFLVL deasserts.
    ${g2}=    Read OTG Reg    ${GINTSTS}
    Bit Should Be Clear    ${g2}    4    RXFLVL still set after draining the FIFO

Received Out ZLP Frames As OUT-Received Then OUT-Complete
    [Documentation]    A control status-stage ZLP: PKTSTS=0x02 (BCNT=0) then
    ...                PKTSTS=0x03, RXFLVL held until both are popped.
    Provision OTG Machine
    Execute Command    otg2 ReceiveOut 0 0 0 0
    ${g}=    Read OTG Reg    ${GINTSTS}
    Bit Should Be Set    ${g}    4    RXFLVL not asserted by the OUT packet
    ${p1}=    Read OTG Reg    ${GRXSTSP}
    Field Should Be    ${p1}    17    0xF    2    first OUT status is not OUT-received
    ${g1}=    Read OTG Reg    ${GINTSTS}
    Bit Should Be Set    ${g1}    4    RXFLVL dropped before the complete status
    ${p2}=    Read OTG Reg    ${GRXSTSP}
    Field Should Be    ${p2}    17    0xF    3    second OUT status is not OUT-complete
    ${g2}=    Read OTG Reg    ${GINTSTS}
    Bit Should Be Clear    ${g2}    4    RXFLVL still set after draining the OUT packet

In Transfer Completes With XFRC And Captures The Data
    [Documentation]    Program DIEPTSIZ0 (one 4-byte packet), arm DIEPCTL0.EPENA,
    ...                write the word to the TX FIFO → DIEPINT0.XFRC sets, it
    ...                aggregates to DAINT (XFRC unmasked), and the exact bytes
    ...                are captured for inspection.
    Provision OTG Machine
    # Unmask XFRC so completion shows in DAINT.
    Execute Command    sysbus WriteDoubleWord ${DIEPMSK} 0x00000001
    Execute Command    sysbus WriteDoubleWord ${DAINTMSK} 0x00000001
    # DIEPTSIZ0: PKTCNT=1 (bit19), XFRSIZ=4.
    Execute Command    sysbus WriteDoubleWord ${DIEPTSIZ0} 0x00080004
    # Arm: DIEPCTL0.EPENA (bit31) | CNAK (bit26).
    Execute Command    sysbus WriteDoubleWord ${DIEPCTL0} 0x84000000
    # No completion yet — the data word has not been written.
    ${i0}=    Read OTG Reg    ${DIEPINT0}
    Bit Should Be Clear    ${i0}    0    XFRC set before any data was written
    # Write the IN packet to EP0's TX FIFO.
    Execute Command    sysbus WriteDoubleWord ${DFIFO0} 0xDEADBEEF
    ${i1}=    Read OTG Reg    ${DIEPINT0}
    Bit Should Be Set    ${i1}    0    DIEPINT0.XFRC not set after the packet was sent
    ${d}=    Read OTG Reg    ${DAINT}
    Bit Should Be Set    ${d}    0    DAINT.IEPINT0 not aggregated from the completed IN transfer
    # Captured bytes are little-endian 0xDEADBEEF.
    ${len}=    Model Call    InPacketLength 0
    Should Be Equal As Integers    ${len}    4    captured IN packet length wrong
    ${b0}=    Model Call    InPacketByte 0 0
    ${b3}=    Model Call    InPacketByte 0 3
    Should Be Equal As Integers    ${b0}    0xEF    IN byte 0 wrong
    Should Be Equal As Integers    ${b3}    0xDE    IN byte 3 wrong
    # EPENA self-clears on completion (hardware side effect).
    ${ctl}=    Read OTG Reg    ${DIEPCTL0}
    Bit Should Be Clear    ${ctl}    31    EPENA not cleared after transfer completed

In Zero-Length Packet Completes Immediately On EPENA
    [Documentation]    A status-stage IN ZLP: XFRSIZ=0, so arming DIEPCTL0.EPENA
    ...                completes at once (no TX-FIFO write follows).
    Provision OTG Machine
    Execute Command    sysbus WriteDoubleWord ${DIEPTSIZ0} 0x00080000
    Execute Command    sysbus WriteDoubleWord ${DIEPCTL0} 0x84000000
    ${i}=    Read OTG Reg    ${DIEPINT0}
    Bit Should Be Set    ${i}    0    XFRC not set for the zero-length IN packet
