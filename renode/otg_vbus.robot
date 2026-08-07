*** Settings ***
Documentation    OTG VBUS / session configuration — the H7 must run in device
...              mode WITHOUT a physical VBUS-sense pin (the Daisy's on-board
...              micro-USB has none wired to the sense input). Per ST's own
...              stm32h7xx_ll_usb.c USB_DevInit, that means GCCFG.VBDEN = 0 plus
...              GOTGCTL.BVALOEN + BVALOVAL = 1 (force B-session valid). If VBDEN
...              is left set with no VBUS, the core never sees a valid session
...              and the device does not enumerate.
...
...              This boots the real HAL USB init (usb-init-exerciser →
...              synopsys-usb-otg enable()) against the now register-accurate OTG
...              model (OTG_CID=0x1200, GSNPSID=0x4F54310A) and checks the driver
...              left the H7-correct VBUS config. It FAILS while the driver keys
...              its H7 arm on CID==0x4F54310A (which the real CID 0x1200 never
...              matches → the F429 arm runs, setting VBDEN=1); it PASSES once the
...              driver keys the H7 arm on GSNPSID.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${ELF}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/usb-init-exerciser
${GOTGCTL}       0x40080000
${GCCFG}         0x40080038

*** Keywords ***
Read Reg
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
Driver Configures H7 VBUS For Sessionless Device Operation
    Execute Command    mach create "otg-vbus"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision Clocked RCC And DWT
    Provision USB OTG
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    cpu VectorTableOffset 0x08000000
    # enable() (the VBUS/GCCFG setup) runs inside UsbBus::new during boot.
    Execute Command    emulation RunFor "00:00:00.2"

    ${gccfg}=    Read Reg    ${GCCFG}
    Bit Should Be Clear    ${gccfg}    21    GCCFG.VBDEN left set — H7 needs VBUS sensing OFF to run without a VBUS pin

    ${gotgctl}=    Read Reg    ${GOTGCTL}
    Bit Should Be Set    ${gotgctl}    6    GOTGCTL.BVALOEN not set — B-session override not enabled
    Bit Should Be Set    ${gotgctl}    7    GOTGCTL.BVALOVAL not set — B-session not forced valid
