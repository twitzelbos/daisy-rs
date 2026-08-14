*** Settings ***
Documentation    Boot the daisy-usb-audio CODEC build (renode_test,codec) from
...              QSPI XIP and assert (a) it REACHES main() + the renode_test loop
...              — cortex-m-rt's .bss/.data init (with the codec statics) and the
...              codec pre_init (D2-SRAM + backup-access enable) all completed,
...              shown by PC7 toggling — and (b) the GapGuards saw ZERO accesses,
...              i.e. startup never strayed across a RAM-region gap (the
...              __ebss-in-D2 bug class, which would show a non-zero guard count).
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${APP}           ${CURDIR}/../target/renode-codec/thumbv7em-none-eabihf/release/daisy-usb-audio
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${GPIOC_ODR}     0x58020814

*** Keywords ***
Sample LED Bit
    ${odr}=      Execute Command    sysbus ReadDoubleWord ${GPIOC_ODR}
    ${odr_int}=  Convert To Integer    ${odr.strip()}    16
    ${bit}=      Evaluate    (${odr_int} >> 7) & 1
    RETURN    ${bit}

*** Test Cases ***
Codec Build Reaches Main After Startup Init
    Execute Command    mach create "daisy-usb-audio-codec"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision Gap Guards
    Execute Command    sysbus LoadELF @${APP}
    Execute Command    cpu VectorTableOffset 0x90000000

    @{bits}=    Create List
    FOR    ${i}    IN RANGE    8
        Execute Command    emulation RunFor "00:00:00.25"
        ${b}=    Sample LED Bit
        Append To List    ${bits}    ${b}
    END
    Log To Console    \nGPIO PC7 samples: ${bits}
    Should Contain    ${bits}    ${1}    codec build never toggled PC7 HIGH — didn't reach main()
    Should Contain    ${bits}    ${0}    codec build never toggled PC7 LOW — heartbeat not running

    # And startup init must not have crossed a RAM-region gap.
    ${g1}=    Execute Command    sysbus.gapDtcmAxi Accesses
    ${g1i}=    Evaluate    int(str('''${g1}''').strip(), 0)
    Should Be Equal As Integers    ${g1i}    0    startup init crossed the DTCM→AXI gap (__ebss dragged into a foreign region?)
    ${g2}=    Execute Command    sysbus.gapAxiD2 Accesses
    ${g2i}=    Evaluate    int(str('''${g2}''').strip(), 0)
    Should Be Equal As Integers    ${g2i}    0    startup init crossed the AXI→D2 gap
