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
...              to alt 0 and checks the stream goes idle. It also checks the
...              explicit-feedback endpoint (iso IN EP2) carries the nominal 10.14
...              Ff, and drives a Feature Unit control round-trip: SET_CUR(VOLUME)
...              + SET_CUR(MUTE) (control transfers WITH a data stage) then
...              GET_CUR(VOLUME) read back on EP0 IN. Drives real usb-device iso
...              read()/write() over the Rx/TX FIFOs, EP0 control data stages, the
...              SOF cadence (GINTSTS.SOF / DSTS.FNSOF), and the GINTSTS→NVIC(OTG_FS)
...              delivery path — RM0433 §59.15.
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
# The explicit-feedback endpoint (iso IN EP2, allocated after the two data EPs).
${FB_EP}         2
${MARK_STATE}    0x20010000
${MARK_RXCOUNT}  0x20010004
${MARK_RXBYTES}  0x20010008
${MARK_FIRST}    0x2001000C
${MARK_ALT}      0x20010010
${MARK_ISR}      0x20010014
${MARK_MUTE}     0x20010018
${MARK_VOL}      0x2001001C
${ST_CONFIGURED}    2
# Feature Unit control (entity 5). Each SETUP packed LE (low, high); the data
# stage follows as a control OUT on EP0.
#   SET_CUR(VOLUME): bmReqType 0x21, bReq 0x01, wValue 0x0200, wIndex 0x0500, wLen 2
${SETVOL_LOW}    0x02000121
${SETVOL_HIGH}   0x00020500
# volume data = -20 dB = -5120 = 0xEC00 (LE bytes 00 EC)
${VOLDATA_LOW}   0x0000EC00
${VOL_EXPECT}    0xEC00
#   SET_CUR(MUTE): wValue 0x0100, wLen 1, data = 1
${SETMUTE_LOW}   0x01000121
${SETMUTE_HIGH}  0x00010500
${MUTEDATA_LOW}  0x00000001
#   GET_CUR(VOLUME): bmReqType 0xA1, bReq 0x81, wValue 0x0200, wIndex 0x0500, wLen 2
${GETVOL_LOW}    0x020081A1
${GETVOL_HIGH}   0x00020500
#   mic Feature Unit (entity 6): SET_CUR reuses SETVOL_LOW / GET_CUR reuses
#   GETVOL_LOW; only the high word changes (wIndex 0x0600). Data -10 dB = 0xF600.
${SETMICVOL_HIGH}  0x00020600
${MICVOLDATA_LOW}  0x0000F600

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

    # The explicit-feedback endpoint (iso IN EP2) carries the nominal Ff — 48.0
    # samples/frame in full-speed 10.14 = 0x0C0000, little-endian [00, 00, 0C].
    # The device writes it each servicing pass while streaming; a host reads it to
    # rate-match its OUT stream to the device (codec) clock. Proves the feedback
    # endpoint plumbing; the actual value's control loop is hardware-tuned.
    ${fblen}=    Model Call    InPacketLength ${FB_EP}
    Should Be Equal As Integers    ${fblen}    3    feedback packet is not a 3-byte 10.14 Ff
    ${fb0}=    Model Call    InPacketByte ${FB_EP} 0
    Should Be Equal As Integers    ${fb0}    0x00    feedback Ff byte 0 wrong
    ${fb1}=    Model Call    InPacketByte ${FB_EP} 1
    Should Be Equal As Integers    ${fb1}    0x00    feedback Ff byte 1 wrong
    ${fb2}=    Model Call    InPacketByte ${FB_EP} 2
    Should Be Equal As Integers    ${fb2}    0x0C    feedback Ff byte 2 (48.0 in 10.14) wrong

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

    # --- Feature Unit control plane: set speaker volume + mute, read volume back.
    # SET_CUR(VOLUME = -20 dB) is a control transfer WITH a data stage (SETUP then
    # a 2-byte OUT on EP0) — the class's control_out must receive the data and
    # latch it. This is the first data-stage control transfer the suite drives.
    Execute Command    otg2 ReceiveSetup ${SETVOL_LOW} ${SETVOL_HIGH}
    Execute Command    emulation RunFor "00:00:00.05"
    Execute Command    otg2 ReceiveOut 0 ${VOLDATA_LOW} ${ZERO} 2
    Execute Command    emulation RunFor "00:00:00.05"
    ${vol}=    Read Mem    ${MARK_VOL}
    Should Be Equal As Integers    ${vol}    ${VOL_EXPECT}    class did not latch SET_CUR(VOLUME)

    # SET_CUR(MUTE = 1): SETUP + 1-byte data OUT.
    Execute Command    otg2 ReceiveSetup ${SETMUTE_LOW} ${SETMUTE_HIGH}
    Execute Command    emulation RunFor "00:00:00.05"
    Execute Command    otg2 ReceiveOut 0 ${MUTEDATA_LOW} ${ZERO} 1
    Execute Command    emulation RunFor "00:00:00.05"
    ${mute}=    Read Mem    ${MARK_MUTE}
    Should Be Equal As Integers    ${mute}    1    class did not latch SET_CUR(MUTE)

    # GET_CUR(VOLUME): the device answers on EP0 IN with the 2-byte value we set
    # (0xEC00 → LE bytes 00 EC). Closes the control round-trip.
    Execute Command    otg2 ReceiveSetup ${GETVOL_LOW} ${GETVOL_HIGH}
    Execute Command    emulation RunFor "00:00:00.05"
    ${gv0}=    Model Call    InPacketByte 0 0
    Should Be Equal As Integers    ${gv0}    0x00    GET_CUR(VOLUME) byte 0 wrong
    ${gv1}=    Model Call    InPacketByte 0 1
    Should Be Equal As Integers    ${gv1}    0xEC    GET_CUR(VOLUME) byte 1 wrong

    # The mic Feature Unit (entity 6) is independent. SET_CUR(VOLUME = -10 dB) on
    # it, then GET_CUR(mic VOLUME) reads -10 dB back off EP0 IN (00 F6), while
    # GET_CUR(speaker VOLUME) still reads the -20 dB set earlier (00 EC) — proving
    # the two units route to separate state, not one shared field.
    Execute Command    otg2 ReceiveSetup ${SETVOL_LOW} ${SETMICVOL_HIGH}
    Execute Command    emulation RunFor "00:00:00.05"
    Execute Command    otg2 ReceiveOut 0 ${MICVOLDATA_LOW} ${ZERO} 2
    Execute Command    emulation RunFor "00:00:00.05"
    Execute Command    otg2 ReceiveSetup ${GETVOL_LOW} ${SETMICVOL_HIGH}
    Execute Command    emulation RunFor "00:00:00.05"
    ${mv1}=    Model Call    InPacketByte 0 1
    Should Be Equal As Integers    ${mv1}    0xF6    GET_CUR(mic VOLUME) did not read back -10 dB
    Execute Command    otg2 ReceiveSetup ${GETVOL_LOW} ${GETVOL_HIGH}
    Execute Command    emulation RunFor "00:00:00.05"
    ${sv1}=    Model Call    InPacketByte 0 1
    Should Be Equal As Integers    ${sv1}    0xEC    speaker volume changed — mic write leaked

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
