*** Settings ***
Documentation    Boot the daisy-usb-audio CODEC build (renode_test,codec) from
...              QSPI XIP and assert it REACHES main() — i.e. cortex-m-rt's
...              .bss/.data init completed without locking up. With the GapGuards
...              provisioned, a startup loop crossing a RAM-region gap (the
...              __ebss-in-D2 bug class) CPU-aborts and never sets the marker.
...              main() writes bmark(2) = 0x00000002 to Backup SRAM 0x3880_0200
...              right after startup init (before the renode_test LED loop).
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${APP}           ${CURDIR}/../target/renode-codec/thumbv7em-none-eabihf/release/daisy-usb-audio
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${BKP_MARK}      0x38800200

*** Test Cases ***
Codec Build Reaches Main After Startup Init
    Execute Command    mach create "daisy-usb-audio-codec"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    # QSPI XIP window (0x9000_0000) + the RAM-gap guards.
    Apply H7 Peripheral Stubs
    Provision Gap Guards
    Execute Command    sysbus LoadELF @${APP}
    Execute Command    cpu VectorTableOffset 0x90000000
    Execute Command    emulation RunFor "00:00:00.050"

    ${m}=    Execute Command    sysbus ReadDoubleWord ${BKP_MARK}
    Should Contain    ${m}    0x00000002    codec build did not reach main() — startup init locked up (RAM-gap crossing?)
