*** Settings ***
Documentation    Register- and behaviour-level tests for STM32H7_FMC_SDRAM,
...              grounded in RM0433 §22.7 (FMC SDRAM). Covers the control /
...              timing / command / refresh / status registers, the JEDEC
...              power-up command sequence gating, and the 64 MiB data window
...              (byte/half/word access, boundaries, out-of-bounds).
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${SDCR1}         0x52004140
${SDCR2}         0x52004144
${SDTR1}         0x52004148
${SDTR2}         0x5200414C
${SDCMR}         0x52004150
${SDRTR}         0x52004154
${SDSR}          0x52004158
${SDRAM}         0xC0000000
${BCR1}          0x52004000
# RCC_AHB3ENR (FMC bus-clock gate, bit 12) — for the clock-gate check.
${RCC_AHB3ENR}   0x580244D4

# SDCMR command words (MODE[2:0] | CTB1 bit4 | NRFS bits[8:5] | MRD bits[22:9]).
${CMD_CLK}       0x00000011
${CMD_PALL}      0x00000012
${CMD_REFRESH}   0x00000073
${CMD_LOADMODE}  0x00046414

*** Keywords ***
New FMC Machine
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision FMC SDRAM

Enable FMC Controller
    [Documentation]    Set BCR1.FMCEN (bit 31), the global FMC controller enable.
    ...                Without it the model ignores all SDCMR commands (RM0433
    ...                §22.7.2), so every real bring-up must set it first.
    Execute Command    sysbus WriteDoubleWord ${BCR1} 0x80000000

Run FMC Init Sequence
    Enable FMC Controller
    Execute Command    sysbus WriteDoubleWord ${SDCR1} 0x000019E9
    Execute Command    sysbus WriteDoubleWord ${SDTR1} 0x09F27461
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_CLK}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_PALL}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_REFRESH}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_LOADMODE}
    Execute Command    sysbus WriteDoubleWord ${SDRTR} 0x000005F2

New Initialised FMC Machine
    New FMC Machine
    Run FMC Init Sequence

*** Test Cases ***
# --- Control / timing / refresh registers (RM0433 §22.7.1/.3/.5) ---
SDCR1 Round Trips
    New FMC Machine
    Execute Command    sysbus WriteDoubleWord ${SDCR1} 0x000019E9
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDCR1}
    Should Contain    ${v}    0x000019E9

SDCR2 Round Trips
    New FMC Machine
    Execute Command    sysbus WriteDoubleWord ${SDCR2} 0x000001D0
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDCR2}
    Should Contain    ${v}    0x000001D0

SDTR1 Round Trips
    New FMC Machine
    Execute Command    sysbus WriteDoubleWord ${SDTR1} 0x09F27461
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDTR1}
    Should Contain    ${v}    0x09F27461

SDTR2 Round Trips
    New FMC Machine
    Execute Command    sysbus WriteDoubleWord ${SDTR2} 0x00000961
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDTR2}
    Should Contain    ${v}    0x00000961

SDRTR Count Round Trips
    New FMC Machine
    # COUNT occupies bits[13:1]; 0x5F2 = 0x2F9 << 1.
    Execute Command    sysbus WriteDoubleWord ${SDRTR} 0x000005F2
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRTR}
    Should Contain    ${v}    0x000005F2

SDSR Reports Ready And Not Busy
    New FMC Machine
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDSR}
    # BUSY (bit5), MODES1/2, RE all 0 → 0x00000000.
    Should Contain    ${v}    0x00000000

# --- Gating before initialisation ---
Read Before Init Returns Zero
    New FMC Machine
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRAM}
    Should Contain    ${v}    0x00000000

Write Before Init Is Dropped
    New FMC Machine
    Execute Command    sysbus WriteDoubleWord ${SDRAM} 0xDEADBEEF
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRAM}
    Should Contain    ${v}    0x00000000

# --- Init-sequence ordering (all four commands, in order, required) ---
Skipping Clock Enable Leaves SDRAM Gated
    New FMC Machine
    Enable FMC Controller
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_PALL}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_REFRESH}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_LOADMODE}
    Execute Command    sysbus WriteDoubleWord ${SDRAM} 0xDEADBEEF
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRAM}
    Should Contain    ${v}    0x00000000

Skipping Precharge Leaves SDRAM Gated
    New FMC Machine
    Enable FMC Controller
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_CLK}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_REFRESH}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_LOADMODE}
    Execute Command    sysbus WriteDoubleWord ${SDRAM} 0xDEADBEEF
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRAM}
    Should Contain    ${v}    0x00000000

Skipping AutoRefresh Leaves SDRAM Gated
    New FMC Machine
    Enable FMC Controller
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_CLK}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_PALL}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_LOADMODE}
    Execute Command    sysbus WriteDoubleWord ${SDRAM} 0xDEADBEEF
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRAM}
    Should Contain    ${v}    0x00000000

Load Mode Register First Leaves SDRAM Gated
    New FMC Machine
    Enable FMC Controller
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_LOADMODE}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_CLK}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_PALL}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_REFRESH}
    Execute Command    sysbus WriteDoubleWord ${SDRAM} 0xDEADBEEF
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRAM}
    Should Contain    ${v}    0x00000000

# --- Enable gates (RM0433 §22.7.2): the controller AND the bus clock ---
Controller Disabled BCR1 FMCEN Zero Leaves SDRAM Gated
    [Documentation]    The FMC controller enable (BCR1.FMCEN, bit 31) is a
    ...                precondition: run the full, correctly-ordered JEDEC
    ...                sequence but NEVER set it — the model must ignore every
    ...                command and leave the window unusable. Pins the "forgot
    ...                BCR1.FMCEN" bug (the real HW cold-init bug) with its own
    ...                test (previously the gate had none).
    New FMC Machine
    # NB: deliberately no `Enable FMC Controller`.
    Execute Command    sysbus WriteDoubleWord ${SDCR1} 0x000019E9
    Execute Command    sysbus WriteDoubleWord ${SDTR1} 0x09F27461
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_CLK}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_PALL}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_REFRESH}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_LOADMODE}
    Execute Command    sysbus WriteDoubleWord ${SDRAM} 0xDEADBEEF
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRAM}
    Should Contain    ${v}    0x00000000

FMC Bus Clock Gated RCC AHB3ENR FMCEN Zero Leaves SDRAM Gated
    [Documentation]    The FMC also needs its AHB3 *bus* clock
    ...                (RCC_AHB3ENR.FMCEN, bit 12) — separate from the controller
    ...                enable. With the check armed (fmc RccAhb3enrAddress set)
    ...                but the gate bit clear, the model must ignore the init
    ...                commands even though BCR1.FMCEN is set. Uses the stub RCC
    ...                (sticky RW) so the gate bit is fully controlled.
    Execute Command    mach create "fmc-busclock-off"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision FMC SDRAM
    Execute Command    fmc RccAhb3enrAddress ${RCC_AHB3ENR}
    # Controller enabled, but the AHB3 bus-clock gate left OFF.
    Enable FMC Controller
    Execute Command    sysbus WriteDoubleWord ${SDCR1} 0x000019E9
    Execute Command    sysbus WriteDoubleWord ${SDTR1} 0x09F27461
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_CLK}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_PALL}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_REFRESH}
    Execute Command    sysbus WriteDoubleWord ${SDCMR} ${CMD_LOADMODE}
    Execute Command    sysbus WriteDoubleWord ${SDRAM} 0xDEADBEEF
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRAM}
    Should Contain    ${v}    0x00000000

FMC Bus Clock Enabled RCC AHB3ENR FMCEN One Makes SDRAM Usable
    [Documentation]    Same setup, but with RCC_AHB3ENR.FMCEN set — the init
    ...                sequence now completes and the window round-trips. Proves
    ...                the clock-gate check passes when the bus clock is on (so
    ...                the check isn't just always-fail).
    Execute Command    mach create "fmc-busclock-on"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision FMC SDRAM
    Execute Command    fmc RccAhb3enrAddress ${RCC_AHB3ENR}
    Execute Command    sysbus WriteDoubleWord ${RCC_AHB3ENR} 0x00001000
    Run FMC Init Sequence
    Execute Command    sysbus WriteDoubleWord ${SDRAM} 0xDEADBEEF
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRAM}
    Should Contain    ${v}    0xDEADBEEF

# --- Data access after a complete init ---
DoubleWord Round Trips After Init
    New Initialised FMC Machine
    Execute Command    sysbus WriteDoubleWord ${SDRAM} 0xDEADBEEF
    ${v}=    Execute Command    sysbus ReadDoubleWord ${SDRAM}
    Should Contain    ${v}    0xDEADBEEF

Byte Access Round Trips After Init
    New Initialised FMC Machine
    Execute Command    sysbus WriteByte 0xC0001000 0xA7
    ${v}=    Execute Command    sysbus ReadByte 0xC0001000
    Should Contain    ${v}    0xA7

HalfWord Access Round Trips After Init
    New Initialised FMC Machine
    Execute Command    sysbus WriteWord 0xC0002000 0xBEEF
    ${v}=    Execute Command    sysbus ReadWord 0xC0002000
    Should Contain    ${v}    0xBEEF

Interior And Last Word Round Trip After Init
    New Initialised FMC Machine
    Execute Command    sysbus WriteDoubleWord 0xC2000000 0x5A5A5A5A
    Execute Command    sysbus WriteDoubleWord 0xC3FFFFFC 0x1234ABCD
    ${m}=    Execute Command    sysbus ReadDoubleWord 0xC2000000
    Should Contain    ${m}    0x5A5A5A5A
    ${e}=    Execute Command    sysbus ReadDoubleWord 0xC3FFFFFC
    Should Contain    ${e}    0x1234ABCD

One Word Past 64 MiB Is Unbacked
    New Initialised FMC Machine
    ${v}=    Execute Command    sysbus ReadDoubleWord 0xC4000000
    Should Contain    ${v}    0x00000000

Distinct Addresses Hold Distinct Values
    New Initialised FMC Machine
    Execute Command    sysbus WriteDoubleWord 0xC0000000 0x11111111
    Execute Command    sysbus WriteDoubleWord 0xC0000004 0x22222222
    ${a}=    Execute Command    sysbus ReadDoubleWord 0xC0000000
    Should Contain    ${a}    0x11111111
    ${b}=    Execute Command    sysbus ReadDoubleWord 0xC0000004
    Should Contain    ${b}    0x22222222
