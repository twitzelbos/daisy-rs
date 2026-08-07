*** Settings ***
Documentation    Verify Renode's Cortex-M7 MPU model against the ARMv7-M ARM
...              (DDI 0403E §B3.5) by running the mpu-exerciser firmware, which
...              programs MPU regions and performs one access per behaviour,
...              recording in DTCM whether each access faulted (MemManage). This
...              turns "tlib has an MPU" into "the MPU enforces the architecture":
...              region count, AP permissions, region priority, subregion
...              disable, PRIVDEFENA background mapping, MPU-disabled, and the
...              MMFSR/MMFAR fault status a data violation leaves.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}

*** Variables ***
${PLATFORM}      ${CURDIR}/daisy_seed.repl
${ELF}           ${CURDIR}/../target/thumbv7em-none-eabihf/release/mpu-exerciser
${M_DONE}        0x2001F000
${M_DREGION}     0x2001F004
${M_MMFSR}       0x2001F008
${M_MMFAR}       0x2001F00C
${M_T1}          0x2001F010
${M_T2}          0x2001F014
${M_T3}          0x2001F018
${M_T4}          0x2001F01C
${M_T5_OFF}      0x2001F020
${M_T5_ON}       0x2001F024
${M_T6}          0x2001F028
${M_T7}          0x2001F02C
${M_T8}          0x2001F030
${TEST_ADDR}     0x24000000

*** Keywords ***
Read Marker
    [Arguments]    ${addr}
    ${v}=    Execute Command    sysbus ReadDoubleWord ${addr}
    ${i}=    Convert To Integer    ${v.strip()}    16
    [Return]    ${i}

*** Test Cases ***
MPU Enforces The ARMv7-M Access Rules
    Execute Command    mach create
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Execute Command    sysbus LoadELF @${ELF}
    Execute Command    emulation RunFor "00:00:00.010"

    # The firmware reached the end — every sub-test ran.
    ${done}=    Read Marker    ${M_DONE}
    Should Be Equal As Integers    ${done}    0x00C0FFEE    firmware did not complete the MPU sequence

    # MPU_TYPE.DREGION — the STM32H7 Cortex-M7 has 16 regions.
    ${dregion}=    Read Marker    ${M_DREGION}
    Should Be Equal As Integers    ${dregion}    16    MPU_TYPE.DREGION is not 16

    # AP=000 (no access): a read faults.
    ${t1}=    Read Marker    ${M_T1}
    Should Be Equal As Integers    ${t1}    1    AP=000 no-access region did not fault a read

    # A data violation sets MMFSR.DACCVIOL (bit 1) and, with MMARVALID (bit 7),
    # records the faulting address in MMFAR.
    ${mmfsr}=    Read Marker    ${M_MMFSR}
    ${daccviol}=    Evaluate    (${mmfsr} >> 1) & 1
    Should Be Equal As Integers    ${daccviol}    1    MMFSR.DACCVIOL not set on a data violation
    ${mmarvalid}=    Evaluate    (${mmfsr} >> 7) & 1
    Should Be Equal As Integers    ${mmarvalid}    1    MMFSR.MMARVALID not set
    ${mmfar}=    Read Marker    ${M_MMFAR}
    Should Be Equal As Integers    ${mmfar}    ${TEST_ADDR}    MMFAR is not the faulting address

    # AP=110 (read-only): a write faults, a read succeeds.
    ${t2}=    Read Marker    ${M_T2}
    Should Be Equal As Integers    ${t2}    1    AP=110 read-only region did not fault a write
    ${t3}=    Read Marker    ${M_T3}
    Should Be Equal As Integers    ${t3}    0    AP=110 read-only region wrongly faulted a read

    # Region priority: a higher-numbered full-access region over a lower
    # no-access region wins → no fault.
    ${t4}=    Read Marker    ${M_T4}
    Should Be Equal As Integers    ${t4}    0    higher-numbered region did not take priority

    # Subregion disable: the disabled subregion falls through (no fault); an
    # enabled subregion of the same no-access region still faults.
    ${t5off}=    Read Marker    ${M_T5_OFF}
    Should Be Equal As Integers    ${t5off}    0    disabled subregion still enforced no-access
    ${t5on}=    Read Marker    ${M_T5_ON}
    Should Be Equal As Integers    ${t5on}    1    enabled subregion did not fault

    # PRIVDEFENA background map — a DOCUMENTED RENODE DEVIATION.
    #
    # Per the ARM ARM (§B3.5.3), with PRIVDEFENA=0 a privileged access outside
    # every region must fault (background fault); PRIVDEFENA=1 backs it with the
    # default map. Renode's Cortex-M core sets ARM_FEATURE_MPU but NOT
    # ARM_FEATURE_PMSA (only Cortex-R does), so its background path always uses
    # cortexm_check_default_mapping for privileged accesses — i.e. it behaves as
    # if PRIVDEFENA were permanently 1 and never faults a privileged background
    # access. Explicit regions ARE enforced (T1-T5); only this background-disable
    # is unmodelled. We assert Renode's ACTUAL behaviour so this stays a tested,
    # visible fact (and flags if Renode ever implements PRIVDEFENA):
    #   T6 (PRIVDEFENA=0) SHOULD be 1 on silicon, but Renode yields 0.
    ${t6}=    Read Marker    ${M_T6}
    Should Be Equal As Integers    ${t6}    0    Renode PRIVDEFENA=0 behaviour changed (now faults — it may have been fixed)
    ${t7}=    Read Marker    ${M_T7}
    Should Be Equal As Integers    ${t7}    0    PRIVDEFENA=1 background map wrongly faulted

    # MPU disabled (CTRL.ENABLE=0): a no-access region has no effect.
    ${t8}=    Read Marker    ${M_T8}
    Should Be Equal As Integers    ${t8}    0    disabled MPU still enforced a region
