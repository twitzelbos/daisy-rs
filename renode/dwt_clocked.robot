*** Settings ***
Documentation    STM32H7_DWT_Clocked — the Cortex-M7 Data Watchpoint & Trace
...              unit. DWT is an ARM core peripheral, so the authority is the
...              ARMv7-M Architecture Reference Manual (DDI 0403E §C1.8) and the
...              Cortex-M7 TRM, NOT ST's RM0433. Covers the DWT_CTRL capability
...              fields + control bits, the CYCCNT counter gated on CYCCNTENA AND
...              DEMCR.TRCENA (count rate, freeze, wrap, write), the profiling
...              counters (register-accurate but event-inert in a functional
...              core), PCSR, the comparators, the software-lock registers, and
...              the RCC clock-tree integration that rescales the rate with sys_ck.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${PLATFORM}        ${CURDIR}/daisy_seed.repl
${DEMCR}           0xE000EDFC
${DWT_CTRL}        0xE0001000
${DWT_CYCCNT}      0xE0001004
${DWT_CPICNT}      0xE0001008
${DWT_PCSR}        0xE000101C
${DWT_COMP0}       0xE0001020
${DWT_MASK0}       0xE0001024
${DWT_FUNC0}       0xE0001028
${DWT_LSR}         0xE0001FB4
${RCC_CR}          0x58024400
${RCC_CFGR}        0x58024410
${RCC_PLLCKSELR}   0x58024428
${RCC_PLLCFGR}     0x5802442C
${RCC_PLL1DIVR}    0x58024430
${ONE_MS}          00:00:00.001

*** Keywords ***
New DWT Machine
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Provision Clocked RCC And DWT

Enable Trace
    # DEMCR.TRCENA (bit 24) — required for CYCCNT to count (ARM ARM §C1.8.1).
    Execute Command    sysbus WriteDoubleWord ${DEMCR} 0x01000000

Enable CYCCNT
    Enable Trace
    Execute Command    sysbus WriteDoubleWord ${DWT_CTRL} 0x00000001

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

Set RCC PLL1
    [Arguments]    ${pll1divr}
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCKSELR} 0x00000022
    Execute Command    sysbus WriteDoubleWord ${RCC_PLL1DIVR} ${pll1divr}
    Execute Command    sysbus WriteDoubleWord ${RCC_PLLCFGR} 0x00010009
    Execute Command    sysbus WriteDoubleWord ${RCC_CR} 0x01010000
    Execute Command    sysbus WriteDoubleWord ${RCC_CFGR} 0x00000003

*** Test Cases ***
Control Capability Fields Match The Cortex-M7 DWT
    [Documentation]    DWT_CTRL[31:24] are read-only capability fields. The H7's
    ...                fully-featured DWT has NUMCOMP=4 and NOPRFCNT/NOCYCCNT/
    ...                NOEXTTRIG/NOTRCPKT all 0 → reset value 0x40000000. The
    ...                cortex-m crate reads NUMCOMP/NOCYCCNT/NOPRFCNT.
    New DWT Machine
    ${ctrl}=    Read Reg    ${DWT_CTRL}
    Should Be Equal As Integers    ${ctrl}    0x40000000    DWT_CTRL reset/capability value wrong
    ${numcomp}=    Evaluate    (${ctrl} >> 28) & 0xF
    Should Be Equal As Integers    ${numcomp}    4    NUMCOMP is not 4
    Bit Should Be Clear    ${ctrl}    25    NOCYCCNT set — cycle counter reported absent
    Bit Should Be Clear    ${ctrl}    24    NOPRFCNT set — profiling counters reported absent

Capability Fields Are Read Only
    [Documentation]    Writing the capability field bits must not change them.
    New DWT Machine
    Execute Command    sysbus WriteDoubleWord ${DWT_CTRL} 0xF3000000
    ${ctrl}=    Read Reg    ${DWT_CTRL}
    Should Be Equal As Integers    ${ctrl}    0x40000000    capability bits were writable

CYCCNTENA Control Flag Round Trips
    New DWT Machine
    Enable CYCCNT
    ${on}=    Read Reg    ${DWT_CTRL}
    Bit Should Be Set    ${on}    0    CYCCNTENA did not set
    Execute Command    sysbus WriteDoubleWord ${DWT_CTRL} 0x00000000
    ${off}=    Read Reg    ${DWT_CTRL}
    Bit Should Be Clear    ${off}    0    CYCCNTENA did not clear

Event Enable And Sampling Bits Round Trip
    [Documentation]    PCSAMPLENA (12) and the event-enable bits EXCTRCENA (16)
    ...                .. CYCEVTENA (22) are R/W control bits.
    New DWT Machine
    # PCSAMPLENA | EXCTRCENA | CYCEVTENA = bits 12,16,22.
    Execute Command    sysbus WriteDoubleWord ${DWT_CTRL} 0x00411000
    ${c}=    Read Reg    ${DWT_CTRL}
    Bit Should Be Set    ${c}    12    PCSAMPLENA not stored
    Bit Should Be Set    ${c}    16    EXCTRCENA not stored
    Bit Should Be Set    ${c}    22    CYCEVTENA not stored

CYCCNT Requires Both CYCCNTENA And TRCENA
    [Documentation]    ARM ARM §C1.8.1: CYCCNT counts only while BOTH
    ...                DWT_CTRL.CYCCNTENA and DEMCR.TRCENA are set. With
    ...                CYCCNTENA set but TRCENA clear, it stays frozen.
    New DWT Machine
    # CYCCNTENA set, but DEMCR.TRCENA left clear.
    Execute Command    sysbus WriteDoubleWord ${DWT_CTRL} 0x00000001
    Execute Command    emulation RunFor "${ONE_MS}"
    ${frozen}=    Read Reg    ${DWT_CYCCNT}
    Should Be Equal As Integers    ${frozen}    0    CYCCNT counted without TRCENA
    # Enable trace, then (re)arm CYCCNTENA — the firmware order. The DWT
    # re-evaluates the CYCCNTENA∧TRCENA gate on this DWT write; now it counts.
    Enable Trace
    Execute Command    sysbus WriteDoubleWord ${DWT_CTRL} 0x00000001
    Execute Command    emulation RunFor "${ONE_MS}"
    ${moved}=    Read Reg    ${DWT_CYCCNT}
    Should Be True    ${moved} > 0    CYCCNT did not count once TRCENA was set

CYCCNT Is Frozen While CYCCNTENA Disabled
    New DWT Machine
    Enable Trace
    Execute Command    emulation RunFor "${ONE_MS}"
    ${v}=    Read Reg    ${DWT_CYCCNT}
    Should Be Equal As Integers    ${v}    0    CYCCNT counted while disabled

Writing CYCCNT Sets The Counter
    New DWT Machine
    Execute Command    sysbus WriteDoubleWord ${DWT_CYCCNT} 0x12345678
    ${v}=    Read Reg    ${DWT_CYCCNT}
    Should Be Equal As Integers    ${v}    0x12345678    CYCCNT write not observable

CYCCNT Counts At The Reset HSI Clock 64 MHz
    [Documentation]    At reset the STM32H7 sys_ck is HSI ÷1 = 64 MHz (RM0433
    ...                §8.5.2; RCC_CR reset 0x83, HSIDIV=00). With no PLL
    ...                configured the RCC drives the DWT at that clock, so 1 ms =
    ...                64000 cycles — the real reset rate, not a hard-coded value.
    New DWT Machine
    Enable CYCCNT
    Execute Command    emulation RunFor "${ONE_MS}"
    ${v}=    Read Reg    ${DWT_CYCCNT}
    # 1 ms × 64 MHz = 64000 = 0x0000FA00.
    Should Be Equal As Integers    ${v}    0x0000FA00    CYCCNT count at the reset HSI clock wrong

CYCCNT Wraps At 2 To The 32
    [Documentation]    CYCCNT is a 32-bit free-running counter (§C1.8.8). Seeded
    ...                near the top, 64000 more cycles (1 ms @ reset 64 MHz) wrap
    ...                it modulo 2^32: 0xFFFFFF00 + 0xFA00 = 0x1_0000_F900 →
    ...                0x0000F900.
    New DWT Machine
    Enable CYCCNT
    Execute Command    sysbus WriteDoubleWord ${DWT_CYCCNT} 0xFFFFFF00
    Execute Command    emulation RunFor "${ONE_MS}"
    ${v}=    Read Reg    ${DWT_CYCCNT}
    Should Be Equal As Integers    ${v}    0x0000F900    CYCCNT did not wrap at 2^32

Profiling Counters Are Present And Event Inert
    [Documentation]    CPICNT/EXCCNT/... exist and round-trip, but the functional
    ...                core produces none of the micro-arch events they measure,
    ...                so they do NOT auto-count (documented fidelity boundary).
    New DWT Machine
    Execute Command    sysbus WriteDoubleWord ${DWT_CPICNT} 0x00000042
    ${w}=    Read Reg    ${DWT_CPICNT}
    Should Be Equal As Integers    ${w}    0x42    CPICNT not writable/8-bit
    Enable CYCCNT
    Execute Command    emulation RunFor "${ONE_MS}"
    ${after}=    Read Reg    ${DWT_CPICNT}
    Should Be Equal As Integers    ${after}    0x42    CPICNT phantom-counted (should be event-inert)

PC Sample Register Reads Unavailable
    [Documentation]    PCSR (§C1.8.14) — PC sampling is not modelled in a
    ...                functional core, so it reads the "not available" value.
    New DWT Machine
    ${pcsr}=    Read Reg    ${DWT_PCSR}
    Should Be Equal As Integers    ${pcsr}    0xFFFFFFFF    PCSR not the unavailable value

Comparator Registers Round Trip
    [Documentation]    COMP0/MASK0 store their programmed values (§C1.8.15-16).
    New DWT Machine
    Execute Command    sysbus WriteDoubleWord ${DWT_COMP0} 0x24000000
    Execute Command    sysbus WriteDoubleWord ${DWT_MASK0} 0x0000001F
    ${comp}=    Read Reg    ${DWT_COMP0}
    ${mask}=    Read Reg    ${DWT_MASK0}
    Should Be Equal As Integers    ${comp}    0x24000000    COMP0 not stored
    Should Be Equal As Integers    ${mask}    0x1F         MASK0 not stored (5-bit)

Lock Status Reports No Software Lock
    [Documentation]    LSR.SLI=0 → no software lock implemented, DWT always
    ...                accessible (§C1.8.5-6).
    New DWT Machine
    ${lsr}=    Read Reg    ${DWT_LSR}
    Should Be Equal As Integers    ${lsr}    0    LSR reports a software lock

RCC At 200 MHz Halves The Count Rate
    New DWT Machine
    # PLL1DIVR 0x663 → DIVP1 ÷4 → 200 MHz sys_ck drives the DWT.
    Set RCC PLL1    0x00000663
    Enable CYCCNT
    Execute Command    emulation RunFor "${ONE_MS}"
    ${v}=    Read Reg    ${DWT_CYCCNT}
    # 1 ms × 200 MHz = 200000 = 0x00030D40.
    Should Be Equal As Integers    ${v}    0x00030D40    CYCCNT rate did not follow sys_ck to 200 MHz

RCC At 480 MHz Speeds Up The Count
    New DWT Machine
    # PLL1DIVR 0x277 → DIVN1 120 → 480 MHz sys_ck drives the DWT.
    Set RCC PLL1    0x00000277
    Enable CYCCNT
    Execute Command    emulation RunFor "${ONE_MS}"
    ${v}=    Read Reg    ${DWT_CYCCNT}
    # 1 ms × 480 MHz = 480000 = 0x00075300.
    Should Be Equal As Integers    ${v}    0x00075300    CYCCNT rate did not follow sys_ck to 480 MHz
