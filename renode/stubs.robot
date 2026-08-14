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
    ...                `STM32H7_QuadSPI`, which:
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
    # Compile-order matters: STM32H7_QuadSPI references
    # IS25LP064A by name in its protocol-validation blit path,
    # so the flash-chip class must exist as a compiled assembly first.
    Execute Command    include @${CURDIR}/peripherals/IS25LP064A.cs
    Execute Command    include @${CURDIR}/peripherals/STM32H7_QuadSPI.cs
    ${stubs}=          Catenate    SEPARATOR=
    ...    pwr: Python.PythonPeripheral @ sysbus 0x58024800 { size: 0x400; initable: true; filename: \\"${CURDIR}/peripherals/stm32h7_pwr.py\\" }
    ...    ${\n}
    ...    rcc: Python.PythonPeripheral @ sysbus 0x58024400 { size: 0x400; initable: true; filename: \\"${CURDIR}/peripherals/stm32h7_rcc.py\\" }
    ...    ${\n}
    ...    flashController: Python.PythonPeripheral @ sysbus 0x52002000 { size: 0x400; initable: true; filename: \\"${CURDIR}/peripherals/stm32h7_flash.py\\" }
    ...    ${\n}
    ...    syscfg: Python.PythonPeripheral @ sysbus 0x58000400 { size: 0x400; initable: true; filename: \\"${CURDIR}/peripherals/stm32h7_syscfg.py\\" }
    ...    ${\n}
    ...    qspi: SPI.Daisy.STM32H7_QuadSPI @ { sysbus 0x52005000; sysbus new Bus.BusMultiRegistration { address: 0x90000000; size: 0x800000; region: \\"xip\\" } }
    ...    ${\n}
    ...    qspiFlash: SPI.Daisy.IS25LP064A @ qspi { underlyingMemory: qspiFlashBacking }
    ...    ${\n}
    Execute Command    machine LoadPlatformDescriptionFromString "${stubs}"
    # Flag the 0x9000_0000 XIP window executable-IO so the CPU can fetch
    # instructions from it through the controller's protocol-driven reads.
    # The automatic flagging in Machine.PostCreationActions() already ran when
    # the base platform loaded — before these stubs registered the controller —
    # so we flag the range explicitly here now that qspi's "xip" region exists.
    Execute Command    cpu RegisterAccessFlags 0x90000000 0x800000 true

Provision SAI And Circular DMA
    [Documentation]    Replace DMA1 with the circular/half-transfer-capable
    ...                fork (STM32H7_DMA_Circular) and attach the SAI1 model,
    ...                wiring SAI1_A/B DMA requests to DMAMUX1 lines 87/88.
    ...                Re-establishes DMA1's NVIC IRQ lines and the DMAMUX1→DMA1
    ...                edges that unregistering DMA1 drops. Assumes a fresh
    ...                machine with the Daisy platform loaded.
    Execute Command    sysbus Unregister dma1
    Execute Command    include @${CURDIR}/peripherals/STM32H7_DMA_Circular.cs
    Execute Command    include @${CURDIR}/peripherals/STM32H7_SAI.cs
    ${p}=    Catenate    SEPARATOR=
    ...    dma1: DMA.STM32H7_DMA_Circular @ sysbus 0x40020000${\n}
    ...    ${SPACE * 4}\[0-7] -> nvic@[11-17, 47]${\n}
    ...    dmamux1:${\n}
    ...    ${SPACE * 4}\[0-7] -> dma1@[0-7]${\n}
    ...    sai1: Sound.STM32H7_SAI @ sysbus 0x40015800${\n}
    ...    ${SPACE * 4}DmaRequestA -> dmamux1@87${\n}
    ...    ${SPACE * 4}DmaRequestB -> dmamux1@88
    Execute Command    machine LoadPlatformDescriptionFromString "${p}"

Provision PWR
    [Documentation]    Attach the C# STM32H7_PWR model over the PWR block
    ...                (0x58024800), replacing the base platform's PWR tag.
    Execute Command    include @${CURDIR}/peripherals/STM32H7_PWR.cs
    Execute Command    machine LoadPlatformDescriptionFromString "pwr: Miscellaneous.STM32H7_PWR @ sysbus 0x58024800"

Provision Gap Guards
    [Documentation]    Register fault-on-access guards over the two unmapped
    ...                on-chip RAM gaps — DTCM→AXI (0x2002_0000..0x2400_0000)
    ...                and AXI→D2 (0x2408_0000..0x3000_0000). Any access CPU-
    ...                aborts, so a startup .bss/.data loop that crosses a gap
    ...                (the __ebss-in-D2 class) locks up in sim like silicon
    ...                instead of Renode silently returning 0. Load on a fresh
    ...                machine with the Daisy platform.
    Execute Command    include @${CURDIR}/peripherals/GapGuard.cs
    Execute Command    machine LoadPlatformDescriptionFromString "gapDtcmAxi: Miscellaneous.GapGuard @ sysbus <0x20020000, +0x3FE0000> { size: 0x3FE0000 }"
    Execute Command    machine LoadPlatformDescriptionFromString "gapAxiD2: Miscellaneous.GapGuard @ sysbus <0x24080000, +0xBF80000> { size: 0xBF80000 }"

Provision ADC
    [Documentation]    Replace the base platform's F0-generation ADC model at
    ...                0x40022000 (ADC1/2) with the H7 (v4) ADC, so the HAL's
    ...                Adc bring-up handshake (LDORDY/ADCAL/ADRDY) completes in
    ...                sim instead of spinning on bits the F0 model doesn't have.
    ...                Assumes a fresh machine with the Daisy platform loaded.
    Execute Command    sysbus Unregister adcM1S2
    Execute Command    include @${CURDIR}/peripherals/STM32H7_ADC.cs
    Execute Command    machine LoadPlatformDescriptionFromString "adc1: Analog.STM32H7_ADC @ sysbus 0x40022000"

Provision USB OTG
    [Documentation]    Attach the DWC_OTG device-mode model over OTG2
    ...                (0x40080000), which the base platform only TAGS, and wire
    ...                its interrupt to NVIC line 101 (OTG_FS). Lets the HAL's
    ...                UsbBus core-init complete instead of spinning on
    ...                GRSTCTL.AHBIDL/CSRST and GINTSTS.CMOD. Assumes a fresh
    ...                machine with the Daisy platform loaded.
    Execute Command    include @${CURDIR}/peripherals/STM32H7_OTG.cs
    ${p}=    Catenate    SEPARATOR=
    ...    otg2: USB.STM32H7_OTG @ sysbus 0x40080000${\n}
    ...    ${SPACE * 4}IRQ -> nvic@101
    Execute Command    machine LoadPlatformDescriptionFromString "${p}"

Provision Clocked RCC And DWT
    [Documentation]    Replace the base RCC (Python stub / stock model) and the
    ...                fixed-frequency DWT with the clock-computing C# RCC and
    ...                the runtime-settable DWT, so the CYCCNT tick rate tracks
    ...                the real sys_ck. The DWT bootstraps at the RM0433 reset
    ...                clock (HSI ÷1 = 64 MHz); the RCC then drives it from the
    ...                computed clock tree (64 MHz at reset, PLL1 sys_ck once
    ...                configured). Assumes a fresh machine with the Daisy
    ...                platform loaded.
    Execute Command    sysbus Unregister rcc
    Execute Command    sysbus Unregister dwt
    Execute Command    include @${CURDIR}/peripherals/STM32H7_DWT_Clocked.cs
    Execute Command    include @${CURDIR}/peripherals/STM32H7_RCC_Clocked.cs
    ${clk}=            Catenate    SEPARATOR=
    ...    dwt: Miscellaneous.STM32H7_DWT_Clocked @ sysbus 0xE0001000 { frequency: 64000000 }
    ...    ${\n}
    ...    rcc: Miscellaneous.STM32H7_RCC_Clocked @ sysbus 0x58024400
    Execute Command    machine LoadPlatformDescriptionFromString "${clk}"

Provision FMC SDRAM
    [Documentation]    Replace the plain always-live SDRAM MappedMemory with
    ...                the STM32H7_FMC_SDRAM controller model, which owns the
    ...                FMC control registers (0x52004000) and gates the 64 MiB
    ...                data window (0xC0000000) until the FMC init command
    ...                sequence completes. Lets firmware SDRAM bring-up be
    ...                validated in sim. Assumes a fresh machine with the
    ...                Daisy platform loaded.
    Execute Command    sysbus Unregister sdramBank1
    Execute Command    include @${CURDIR}/peripherals/STM32H7_FMC_SDRAM.cs
    ${fmc}=            Catenate    SEPARATOR=
    ...    fmc: MTD.STM32H7_FMC_SDRAM @ { sysbus 0x52004000; sysbus new Bus.BusMultiRegistration { address: 0xC0000000; size: 0x4000000; region: \\"sdram\\" } }
    Execute Command    machine LoadPlatformDescriptionFromString "${fmc}"

Provision Cache Coherency Checker
    [Documentation]    Overlay the D2 SRAM1 region (0x30000000) with the
    ...                CacheCoherencyChecker model. Renode's Cortex-M has no
    ...                cache, so the M7 D-cache/DMA non-coherency bug class is
    ...                invisible in sim. This model classifies each access by
    ...                master (CPU via TryGetCurrentCPU vs foreign/DMA), tracks
    ...                the coherency-relevant per-line state, honours the SCB
    ...                cache-maintenance ops (DCIMVAC/... via watchpoints), and
    ...                flags CPU reads of a line a foreign master wrote after the
    ...                CPU cached it (stale read) and DMA reads of CPU-dirty lines.
    ...                baseAddress must match the registration address so the
    ...                MVA maintenance ops map onto the overlaid lines. The
    ...                execBase/execSize window (AXI SRAM, left as real memory the
    ...                CPU runs from) enables the I-cache checks: writes to it are
    ...                code modifications and its per-line PC hooks catch fetches,
    ...                flagging a fetch of code not cleaned to PoU (DCCMVAU) or not
    ...                invalidated (ICIALLU). Assumes a fresh machine with the
    ...                Daisy platform loaded.
    Execute Command    sysbus Unregister sram1
    Execute Command    include @${CURDIR}/peripherals/CacheCoherencyChecker.cs
    ${chk}=            Catenate    SEPARATOR=
    ...    cacheChecker: Miscellaneous.CacheCoherencyChecker @ sysbus 0x30000000 { size: 0x20000; baseAddress: 0x30000000; execBase: 0x24000000; execSize: 0x40 }
    Execute Command    machine LoadPlatformDescriptionFromString "${chk}"

Set Flash Quad Enable
    [Documentation]    Set the IS25LP064A QE bit (status bit 6) via WREN +
    ...                WRSR — the datasheet §8.7 prerequisite for 0xEB Fast
    ...                Read Quad I/O, which the bootloader performs before
    ...                entering memory-mapped mode. Tests that drive XIP
    ...                reads directly (without booting firmware) must call
    ...                this first, or the flash model refuses the quad read.
    ...                Assumes the controller is enabled (CR.EN=1).
    Execute Command    sysbus WriteDoubleWord 0x52005014 0x00000106
    Execute Command    sysbus WriteDoubleWord 0x52005010 0x00000000
    Execute Command    sysbus WriteDoubleWord 0x52005014 0x01000101
    Execute Command    sysbus WriteDoubleWord 0x52005020 0x00000040
