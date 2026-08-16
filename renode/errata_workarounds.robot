*** Settings ***
Documentation    Boot-time silicon-errata workarounds that pass SILENTLY in sim
...              if omitted (the same failure class as the SDNWE=PH5 pin bug and
...              the FMC BCR1.FMCEN gate): a required register poke that Renode
...              can't otherwise force, because the underlying hazard is analog /
...              multi-master and not modellable. This suite boots the real
...              bootloader and asserts the poke actually happened.
...
...              Covered: ES0392 §2.2.10 — "Reading from AXI SRAM may lead to
...              data read corruption" on silicon revisions before rev V. The
...              workaround sets READ_ISS_OVERRIDE (bit 0) in AXI_TARG7_FN_MOD
...              (0x5100_8108), reducing the AXI read-issuing capability to 1 for
...              target 7 (the AXI SRAM at 0x2400_0000). We rely on AXI SRAM for
...              every TUI heap, so a regression here would corrupt those apps on
...              rev-Y/W/X boards only — invisible on the rev-V dev unit and in
...              sim. The bootloader gates the write on DBGMCU_IDCODE
...              (0x5C00_1000): apply when REV_ID < 0x2000, skip on rev V+ where
...              the erratum is fixed and the field already reads 1.
Suite Setup      Setup
Suite Teardown   Teardown
Test Teardown    Test Teardown
Resource         ${RENODEKEYWORDS}
Resource         stubs.robot

*** Variables ***
${BOOTLOADER}          ${CURDIR}/../target/renode/thumbv7em-none-eabihf/release/daisy-boot
${PLATFORM}            ${CURDIR}/daisy_seed.repl
${AXI_TARG7_FN_MOD}    0x51008108
${DBGMCU_IDCODE}       0x5C001000

*** Keywords ***
Boot Bootloader
    [Documentation]    Bring up the machine with the peripheral stubs + clocked
    ...                RCC (so clocks::init/freeze run), load the real bootloader,
    ...                optionally plant an IDCODE, and run just long enough for
    ...                the very first thing main() does — the §2.2.10 AXI poke,
    ...                which precedes clock config — to execute.
    [Arguments]    ${name}    ${idcode}=${EMPTY}
    Execute Command    mach create "${name}"
    Execute Command    machine LoadPlatformDescription @${PLATFORM}
    Apply H7 Peripheral Stubs
    Provision Clocked RCC And DWT
    Execute Command    sysbus LoadELF @${BOOTLOADER}
    Run Keyword If    "${idcode}" != "${EMPTY}"
    ...    Execute Command    sysbus WriteDoubleWord ${DBGMCU_IDCODE} ${idcode}
    Execute Command    cpu VectorTableOffset 0x08000000
    Execute Command    emulation RunFor "00:00:00.1"

*** Test Cases ***
Pre Rev V Silicon Applies AXI SRAM Read Issuing Workaround
    [Documentation]    Default IDCODE = 0 → REV_ID 0x0000 < 0x2000 → pre-rev-V →
    ...                the bootloader must set AXI_TARG7_FN_MOD.READ_ISS_OVERRIDE.
    Boot Bootloader    daisy-axi-prev
    ${v}=    Execute Command    sysbus ReadDoubleWord ${AXI_TARG7_FN_MOD}
    Should Contain    ${v}    0x00000001

Rev Y Silicon Applies AXI SRAM Read Issuing Workaround
    [Documentation]    An explicit rev-Y IDCODE (REV_ID 0x1003, DEV_ID 0x450) is
    ...                still below the rev-V threshold, so the workaround applies.
    ...                Proves the gate keys on REV_ID, not merely on IDCODE == 0.
    Boot Bootloader    daisy-axi-revy    0x10030450
    ${v}=    Execute Command    sysbus ReadDoubleWord ${AXI_TARG7_FN_MOD}
    Should Contain    ${v}    0x00000001

Rev V Silicon Skips AXI SRAM Workaround
    [Documentation]    A rev-V IDCODE (REV_ID 0x2003 ≥ 0x2000) → the erratum is
    ...                fixed and the field already reads 1 out of reset, so the
    ...                bootloader must NOT write it. The modelled register starts
    ...                at 0 and must stay 0, proving the gate actually skips.
    Boot Bootloader    daisy-axi-revv    0x20030450
    ${v}=    Execute Command    sysbus ReadDoubleWord ${AXI_TARG7_FN_MOD}
    Should Contain    ${v}    0x00000000
