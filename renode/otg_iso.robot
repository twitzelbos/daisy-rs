*** Settings ***
Documentation    OTG isochronous data path — the UAC audio endpoints, in sim,
...              serviced from the OTG_FS interrupt. The usb-iso-exerciser
...              firmware polls the stack ONLY in its OTG_FS handler (its main
...              loop just wfi's) and, while the host has selected alt 1, loops
...              iso EP1 OUT playback back to EP1 IN capture. So reaching
...              Configured and looping frames both prove the interrupt path
...              services USB: this test enumerates, SET_INTERFACEs to alt 1,
...              injects an isochronous OUT audio frame (a deterministic byte
...              ramp), checks the firmware read it off iso OUT and the same bytes
...              came back on iso IN (captured by the model), then SET_INTERFACEs
...              to alt 0 and checks the stream goes idle. Drives real usb-device
...              iso read()/write() over the Rx/TX FIFOs, the SOF cadence
...              (GINTSTS.SOF / DSTS.FNSOF), and the GINTSTS→NVIC(OTG_FS) delivery
...              path — RM0433 §59.15.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${ELF}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/usb-iso-exerciser
${USBRST}        0x00001000
${ENUMDNE}       0x00002000
# SET_ADDRESS(5) / SET_CONFIGURATION(1), packed LE (low, high).
${SETADDR_LOW}   0x00050500
${SETCFG_LOW}    0x00010900
# SET_INTERFACE(iface 0): bytes [01,0B,alt,00, 00,00,00,00] → low word only.
# alt 1 = start streaming, alt 0 = stop. usb-device forwards these to the class's
# set_alt_setting; without that hook it would STALL a non-zero alt.
${SETIF1_LOW}    0x00010B01
${SETIF0_LOW}    0x00000B01
${ZERO}          0x00000000
# The iso audio frame: 192 bytes, ramp starting at 0x10.
${ISO_EP}        1
${ISO_LEN}       192
${ISO_SEED}      0x10
${MARK_STATE}    0x20010000
${MARK_RXCOUNT}  0x20010004
${MARK_RXBYTES}  0x20010008
${MARK_FIRST}    0x2001000C
${MARK_ALT}      0x20010010
${MARK_ISR}      0x20010014
${ST_CONFIGURED}    2

*** Keywords ***
Read Mem
    [Arguments]    ${addr}
    ${v}=    Execute Command    sysbus ReadDoubleWord ${addr}
    ${i}=    Convert To Integer    ${v.strip()}    16
    [Return]    ${i}

Model Call
    [Arguments]    ${call}
    ${v}=    Execute Command    otg2 ${call}
    ${i}=    Convert To Integer    ${v.strip()}    16
    [Return]    ${i}

Enumerate To Configured
    Execute Command    otg2 RaiseEvent ${USBRST}
    Execute Command    emulation RunFor "00:00:00.1"
    Execute Command    otg2 RaiseEvent ${ENUMDNE}
    Execute Command    emulation RunFor "00:00:00.1"
    Execute Command    otg2 ReceiveSetup ${SETADDR_LOW} ${ZERO}
    Execute Command    emulation RunFor "00:00:00.1"
    Execute Command    otg2 ReceiveSetup ${SETCFG_LOW} ${ZERO}
    Execute Command    emulation RunFor "00:00:00.1"

*** Test Cases ***
Isochronous Audio Frame Loops Playback To Capture
    Execute Command    mach create "otg-iso"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision Clocked RCC And DWT
    Provision USB OTG
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    cpu VectorTableOffset 0x08000000

    Execute Command    emulation RunFor "00:00:00.2"
    Enumerate To Configured
    ${st}=    Read Mem    ${MARK_STATE}
    Should Be Equal As Integers    ${st}    ${ST_CONFIGURED}    device is not Configured — iso EPs not armed

    # The main loop only wfi's, so enumeration reaching Configured could only have
    # happened via the OTG_FS interrupt handler — assert it actually ran.
    ${isr}=    Read Mem    ${MARK_ISR}
    Should Be True    ${isr} > 0    OTG_FS interrupt never fired — USB was not serviced

    # The streaming interface starts at its default (idle) alt setting.
    ${alt_idle}=    Read Mem    ${MARK_ALT}
    Should Be Equal As Integers    ${alt_idle}    0    class did not start idle at alt 0

    # Host activates the stream: SET_INTERFACE(alt 1). usb-device 0.3.2 forwards
    # this to the class's set_alt_setting, which returns true → the transfer is
    # ACCEPTED (not STALLed) and the class latches alt 1. This is the alt-setting
    # gap under test: without the hook, usb-device would STALL a non-zero alt and
    # the stream could never start.
    Execute Command    otg2 ReceiveSetup ${SETIF1_LOW} ${ZERO}
    Execute Command    emulation RunFor "00:00:00.1"
    ${alt1}=    Read Mem    ${MARK_ALT}
    Should Be Equal As Integers    ${alt1}    1    class never saw SET_INTERFACE(alt 1) — the alt-setting gap

    # Host sends one isochronous OUT (playback) frame: 192 bytes, ramp @0x10.
    Execute Command    otg2 ReceiveOutRamp ${ISO_EP} ${ISO_LEN} ${ISO_SEED}
    Execute Command    emulation RunFor "00:00:00.1"

    # The firmware read the frame off iso OUT.
    ${rxc}=    Read Mem    ${MARK_RXCOUNT}
    Should Be True    ${rxc} >= 1    firmware never read the isochronous OUT frame
    ${rxb}=    Read Mem    ${MARK_RXBYTES}
    Should Be Equal As Integers    ${rxb}    ${ISO_LEN}    iso OUT frame length wrong
    ${first}=    Read Mem    ${MARK_FIRST}
    # bytes[0]=0x10, bytes[1]=0x11 → 0x1110
    Should Be Equal As Integers    ${first}    0x1110    iso OUT ramp payload wrong

    # And the same bytes came straight back out on iso IN (captured by the model
    # from the device's TX FIFO): full loopback through the real endpoint code.
    ${len}=    Model Call    InPacketLength ${ISO_EP}
    Should Be Equal As Integers    ${len}    ${ISO_LEN}    iso IN capture length wrong
    ${b0}=    Model Call    InPacketByte ${ISO_EP} 0
    Should Be Equal As Integers    ${b0}    0x10    iso IN byte 0 not the ramp start
    ${b191}=    Model Call    InPacketByte ${ISO_EP} 191
    # (0x10 + 191) & 0xFF = 0xCF
    Should Be Equal As Integers    ${b191}    0xCF    iso IN last byte not the ramp end

    # A second frame streams too (endpoint re-used frame after frame).
    Execute Command    otg2 ReceiveOutRamp ${ISO_EP} ${ISO_LEN} 0x40
    Execute Command    emulation RunFor "00:00:00.1"
    ${rxc2}=    Read Mem    ${MARK_RXCOUNT}
    Should Be True    ${rxc2} > ${rxc}    second isochronous frame was not streamed
    ${b0b}=    Model Call    InPacketByte ${ISO_EP} 0
    Should Be Equal As Integers    ${b0b}    0x40    second frame did not loop through

    # The driver re-armed iso OUT EP1 (DOEPCTL.EPENA) for the next frame — the
    # behaviour the corrected OTG_CID=0x1200 restores (the re-arm branch keys on
    # it). With delivery gated on EPENA, the second frame above could only have
    # streamed because the re-arm fired; this asserts the mechanism directly.
    ${doepctl1}=    Read Mem    0x40080B20
    ${epena}=    Evaluate    (${doepctl1} >> 31) & 1
    Should Be Equal As Integers    ${epena}    1    driver did not re-arm iso OUT EP1 after the frame

    # The SOF pacer advanced the frame number while the device ran.
    ${frame}=    Model Call    FrameNumber
    Should Be True    ${frame} > 0    SOF frame number never advanced

    # Host stops the stream: SET_INTERFACE(alt 0). The class returns to idle and
    # the app-level gate stops touching the iso endpoints — a subsequently
    # injected frame is NOT consumed (rx_count frozen). This proves the gate is
    # real, not just that alt 1 happened to work.
    Execute Command    otg2 ReceiveSetup ${SETIF0_LOW} ${ZERO}
    Execute Command    emulation RunFor "00:00:00.1"
    ${alt0}=    Read Mem    ${MARK_ALT}
    Should Be Equal As Integers    ${alt0}    0    class did not return to idle on SET_INTERFACE(alt 0)
    ${rxc_before}=    Read Mem    ${MARK_RXCOUNT}
    Execute Command    otg2 ReceiveOutRamp ${ISO_EP} ${ISO_LEN} 0x70
    Execute Command    emulation RunFor "00:00:00.1"
    ${rxc_after}=    Read Mem    ${MARK_RXCOUNT}
    Should Be Equal As Integers    ${rxc_after}    ${rxc_before}    idle stream (alt 0) still consumed an iso OUT frame
