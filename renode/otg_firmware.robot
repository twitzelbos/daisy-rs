*** Settings ***
Documentation    End-to-end proof that the STM32H7_OTG model lets the REAL HAL
...              USB device init complete: the usb-init-exerciser runs
...              clocks::init → freeze-free USB2 → UsbBus::new (the DWC_OTG core
...              reset + PHY handshake) → build a CDC device → poll it. Without
...              the OTG model those inner polls (GRSTCTL.AHBIDL/CSRST,
...              GINTSTS.CMOD) spin forever; here the firmware reaches its poll
...              loop and keeps polling, with no host attached.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${ELF}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/usb-init-exerciser
${MARK_STAGE}    0x20010000
${MARK_POLLS}    0x20010004

*** Test Cases ***
Real UsbBus New And Device Poll Do Not Hang
    Execute Command    mach create "otg-fw"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision Clocked RCC And DWT
    Provision USB OTG
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:00.5"

    # Stage 0x16 = clocks::init + UsbBus::new (core reset handshake) + device
    # build + 100 clean poll() calls, none hung. 0x14 alone already proves the
    # core init (the polls that would spin) completed.
    ${stage}=    Execute Command    sysbus ReadDoubleWord ${MARK_STAGE}
    Log    USB firmware stage: ${stage.strip()}
    Should Contain    ${stage}    0x00000016

    ${polls}=    Execute Command    sysbus ReadDoubleWord ${MARK_POLLS}
    ${polls_int}=    Convert To Integer    ${polls.strip()}    16
    Log    USB poll count: ${polls_int}
    Should Be True    ${polls_int} >= 100    poll() stalled before 100 iterations
