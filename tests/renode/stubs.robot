*** Settings ***
Documentation    Shared setup for STM32H7 peripheral overrides. Every
...              Renode test that boots real firmware through
...              stm32h7xx-hal's `clocks::init` needs the H7 peripheral
...              stubs. Tests that also exercise the QSPI service path
...              (or any code that hits `qspi::exit_memory_mapped`,
...              enters memory-mapped mode, or fetches XIP code) need
...              the patched QUADSPI + IS25LP064A models on top.
...              See peripherals/NOTICE.md.
Resource         ${RENODEKEYWORDS}

*** Keywords ***
Apply H7 Peripheral Stubs
    [Documentation]    Unregisters Renode's under-modelled RCC/FLASH/SYSCFG
    ...                (PWR is only a tag in the base H743 platform) and
    ...                overlays our Python-peripheral stubs with absolute
    ...                paths interpolated from ${CURDIR}. Also swaps the
    ...                built-in `STM32H7_QuadSPI` for our patched
    ...                `STM32H7_QuadSPI_Fixed`, which:
    ...                  * self-clears CR.ABORT (RM0433 §23.6.1) so the
    ...                    bootloader's `exit_memory_mapped()` poll can
    ...                    finish;
    ...                  * computes dummy-byte counts from DCYC + DMODE
    ...                    (upstream Renode only knew single-lane), so
    ...                    0xEB Fast Read Quad I/O with DCYC=6 sends the
    ...                    datasheet's 3 bytes of dummy time in quad mode;
    ...                  * on the CCR-write transition INTO memory-mapped
    ...                    mode, drives the SPI protocol against the
    ...                    IS25LP064A model for the first 256 bytes and
    ...                    blits the protocol-produced bytes back into
    ...                    the MappedMemory at 0x9000_0000. That way a
    ...                    broken CCR config (like the missing mode-bits
    ...                    phase we hit on hardware) corrupts the CPU-
    ...                    visible vector table and the CPU faults, the
    ...                    same failure mode real silicon exhibits.
    ...                And attaches the datasheet-accurate IS25LP064A
    ...                model (SPI protocol per §8.7, all libDaisy-used
    ...                commands, AX Continuous Read Mode) whose backing
    ...                store is the same MappedMemory at 0x9000_0000
    ...                that the CPU executes from.
    ...
    ...                Idempotent per test — assumes a fresh machine
    ...                created by the caller.
    ...
    ...                Inner quotes are escaped as \\" — Renode's tokenizer
    ...                otherwise ends the outer LoadPlatformDescriptionFromString
    ...                string at the first inner quote. Same pattern the
    ...                Renode 1.16 gdb.robot suite uses.
    Execute Command    sysbus Unregister rcc
    Execute Command    sysbus Unregister flashController
    Execute Command    sysbus Unregister syscfg
    Execute Command    sysbus Unregister qspi
    # AdHoc-compile our patched QSPI controller AND the datasheet-accurate
    # IS25LP064A flash-chip model. `include @path.cs` uses Roslyn in-process
    # — no dotnet SDK needed. See peripherals/NOTICE.md.
    # Compile-order matters: STM32H7_QuadSPI_Fixed references
    # IS25LP064A_Fixed by name in its protocol-validation blit path,
    # so the flash-chip class must exist as a compiled assembly first.
    Execute Command    include @${CURDIR}/peripherals/IS25LP064A_Fixed.cs
    Execute Command    include @${CURDIR}/peripherals/STM32H7_QuadSPI_Fixed.cs
    ${stubs}=          Catenate    SEPARATOR=
    ...    pwr: Python.PythonPeripheral @ sysbus 0x58024800 { size: 0x400; initable: true; filename: \\"${CURDIR}/peripherals/stm32h7_pwr.py\\" }
    ...    ${\n}
    ...    rcc: Python.PythonPeripheral @ sysbus 0x58024400 { size: 0x400; initable: true; filename: \\"${CURDIR}/peripherals/stm32h7_rcc.py\\" }
    ...    ${\n}
    ...    flashController: Python.PythonPeripheral @ sysbus 0x52002000 { size: 0x400; initable: true; filename: \\"${CURDIR}/peripherals/stm32h7_flash.py\\" }
    ...    ${\n}
    ...    syscfg: Python.PythonPeripheral @ sysbus 0x58000400 { size: 0x400; initable: true; filename: \\"${CURDIR}/peripherals/stm32h7_syscfg.py\\" }
    ...    ${\n}
    ...    qspi: SPI.STM32H7_QuadSPI_Fixed @ { sysbus 0x52005000; sysbus new Bus.BusMultiRegistration { address: 0x90000000; size: 0x800000; region: \\"xip\\" } }
    ...    ${\n}
    ...    qspiFlash: SPI.IS25LP064A_Fixed @ qspi { underlyingMemory: qspiFlashBacking }
    ...    ${\n}
    Execute Command    machine LoadPlatformDescriptionFromString "${stubs}"
    # Flag the 0x9000_0000 XIP window executable-IO so the CPU can fetch
    # instructions from it through the controller's protocol-driven reads.
    # The automatic flagging in Machine.PostCreationActions() already ran when
    # the base platform loaded — before these stubs registered the controller —
    # so we flag the range explicitly here now that qspi's "xip" region exists.
    Execute Command    cpu RegisterAccessFlags 0x90000000 0x800000 true
