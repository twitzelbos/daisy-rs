*** Settings ***
Documentation    Tests for STM32H7_PWR, grounded in RM0433 §7.5. Covers the
...              backup-domain write-protection bit, the D3CR voltage-scaling
...              select with its VOSRDY handshake, the CSR1 ACTVOS/ACTVOSRDY
...              mirror the HAL polls, and the CR3 USB 3.3 V regulator ready.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${PWR_CR1}       0x58024800
${PWR_CSR1}      0x58024804
${PWR_CR3}       0x5802480C
${PWR_D3CR}      0x58024818

*** Keywords ***
New PWR Machine
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision PWR

*** Test Cases ***
CR1 DBP Round Trips
    [Documentation]    RM0433 §7.5.1: DBP (bit 8) disables backup write-protect.
    New PWR Machine
    Execute Command    sysbus WriteDoubleWord ${PWR_CR1} 0x00000100
    ${v}=    Execute Command    sysbus ReadDoubleWord ${PWR_CR1}
    Should Contain    ${v}    0x00000100

VOS Scale1 Sets VOSRDY
    [Documentation]    RM0433 §7.5.9: D3CR.VOS=Scale1 (11) → VOSRDY (bit 13).
    New PWR Machine
    Execute Command    sysbus WriteDoubleWord ${PWR_D3CR} 0x0000C000
    ${v}=    Execute Command    sysbus ReadDoubleWord ${PWR_D3CR}
    # VOS(0xC000) | VOSRDY(0x2000) = 0xE000.
    Should Contain    ${v}    0x0000E000

VOS Scale2 Sets VOSRDY
    New PWR Machine
    Execute Command    sysbus WriteDoubleWord ${PWR_D3CR} 0x00008000
    ${v}=    Execute Command    sysbus ReadDoubleWord ${PWR_D3CR}
    Should Contain    ${v}    0x0000A000

VOS Scale3 Sets VOSRDY
    New PWR Machine
    Execute Command    sysbus WriteDoubleWord ${PWR_D3CR} 0x00004000
    ${v}=    Execute Command    sysbus ReadDoubleWord ${PWR_D3CR}
    Should Contain    ${v}    0x00006000

CSR1 ACTVOS Mirrors The Selected Scale
    [Documentation]    RM0433 §7.5.2: ACTVOS[15:14] reflects the active scale,
    ...                ACTVOSRDY (bit 13) tracks it — both polled by freeze().
    New PWR Machine
    Execute Command    sysbus WriteDoubleWord ${PWR_D3CR} 0x0000C000
    ${v}=    Execute Command    sysbus ReadDoubleWord ${PWR_CSR1}
    # ACTVOS = Scale1 (0xC000) | ACTVOSRDY (0x2000) = 0xE000.
    Should Contain    ${v}    0x0000E000

CSR1 Reflects Scale3 Reset Default
    New PWR Machine
    ${v}=    Execute Command    sysbus ReadDoubleWord ${PWR_CSR1}
    # RM0433 §7.5.9: D3CR resets to Scale 3 (01). ACTVOS(0x4000) |
    # ACTVOSRDY(0x2000) = 0x6000.
    Should Contain    ${v}    0x00006000

CR3 USB33DEN Sets USB33RDY
    [Documentation]    RM0433 §7.5.4: USB33DEN (bit 24) → USB33RDY (bit 26).
    New PWR Machine
    Execute Command    sysbus WriteDoubleWord ${PWR_CR3} 0x01000000
    ${v}=    Execute Command    sysbus ReadDoubleWord ${PWR_CR3}
    # USB33DEN(0x1000000) | USB33RDY(0x4000000) = 0x5000000.
    Should Contain    ${v}    0x05000000

CR3 LDOEN And SCUEN Are Sticky
    New PWR Machine
    Execute Command    sysbus WriteDoubleWord ${PWR_CR3} 0x00000006
    ${v}=    Execute Command    sysbus ReadDoubleWord ${PWR_CR3}
    Should Contain    ${v}    0x00000006
