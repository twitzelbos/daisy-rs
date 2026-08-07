//
// Copyright (c) 2010-2024 Antmicro
// Copyright (c) 2026 daisy-rs contributors (patch only)
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt' in the Renode
// source tree; a copy is inlined at the top of NOTICE.md in this
// directory.
//
// Vendored from renode/renode-infrastructure master (2026-08-02) with
// the following patches relative to upstream:
//
//   1. QSPI_CR.ABORT (bit 1) is a self-clearing hardware-write bit per
//      RM0433 rev8 §23.6.1: "It is automatically reset once the abort
//      is complete." Upstream uses `.WithFlag(1, writeCallback: ...)`,
//      which leaves the bit at 1 after a write, causing the
//      stm32h7xx-hal `exit_memory_mapped` polling loop
//      `while qspi.cr.read().abort().bit_is_set() {}` to hang forever.
//      Patched by adding `FieldMode.Write` so reads always return 0
//      — which matches the observable behaviour on real silicon (the
//      abort completes in ~cycles, faster than any read the CPU can
//      issue on the AHB).
//
//   2. Container generic changed from `GenericSpiFlash` →
//      `ISPIPeripheral`, so we can plug in datasheet-accurate models
//      like `IS25LP064A_Fixed` that don't inherit GenericSpiFlash.
//
//   3. Protocol-validation-on-CCR-write. Upstream comment:
//      "we can't map a flash memory to address range here, this has
//      to be a property of the flash itself (e.g. GenericSpiFlash:
//      underlyingMemory)." That means AHB reads at 0x9000_0000
//      bypass the QSPI controller entirely and hit the flash chip's
//      MappedMemory directly, so any config bug in memory-mapped
//      mode (missing mode-bits phase, wrong instruction, bad DCYC)
//      goes undetected on the direct read path.
//
//      Ideal fix would be to serve XIP reads via a
//      `[ConnectionRegion("xip")]` that drives the SPI protocol per
//      byte — but Renode's CPU translator refuses to fetch code from
//      a non-memory sysbus region ("PC does not lay in memory"), so
//      that approach breaks XIP execution entirely.
//
//      Instead we hook the CCR write callback: whenever the
//      controller enters memory-mapped mode we drive one round of
//      the CCR-configured SPI protocol against the attached
//      IS25LP064A chip for the first N bytes of flash, then blit
//      the protocol-returned bytes back into the MappedMemory via
//      `IS25LP064A_Fixed.DirectWrite`. A correct config means the
//      protocol returns the same bytes that were already there and
//      the blit is a no-op; a broken config (e.g. 0xEB with
//      ABMODE=0) makes the flash chip mis-parse phases and return
//      shifted/garbage bytes, so the MappedMemory gets corrupted
//      and CPU fetches fault. See
//      `ProtocolValidateMemoryMappedMode`.
//
// The class is renamed to `STM32H7_QuadSPI_Fixed` so it doesn't
// collide with the built-in `STM32H7_QuadSPI` short-name resolution.
//
using System;
using System.Collections.Generic;
using System.Threading;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.SPI
{
    // Container generic changed from GenericSpiFlash → ISPIPeripheral so
    // we can plug in datasheet-accurate flash-chip models like our
    // IS25LP064A_Fixed. Upstream only ever used GenericSpiFlash's SPI
    // interface (Transmit / FinishTransmission), so the wider generic
    // is functionally equivalent for any spec-compliant flash model.
    public sealed class STM32H7_QuadSPI_Fixed : NullRegistrationPointPeripheralContainer<ISPIPeripheral>, IKnownSize,
        IDoubleWordPeripheral, IWordPeripheral, IBytePeripheral,
        // XIP-fidelity: the controller also serves the 0x9000_0000 execute-in-place
        // window (a BusMultiRegistration "xip" region). IMemory lets the CPU point
        // VTOR/PC there; IExecutableInPlaceMemory flags the range executable-IO so
        // instruction fetches route through the bus read callback into our
        // protocol-driven [ConnectionRegion("xip")] reads; IPerConnectionRegionMultibyteAccess
        // serves block LoadELF/LoadBinary of the app image into flash.
        IMemory, IExecutableInPlaceMemory, IPerConnectionRegionMultibyteAccess
    {
        // NOTE: Current implementation does not support DMA interface
        public STM32H7_QuadSPI_Fixed(IMachine machine) : base(machine)
        {
            this.ownerMachine = machine;
            registers = new DoubleWordRegisterCollection(this);
            DefineRegisters();
        }

        // --- NCS pin-mux fidelity (Bug D) ------------------------------------
        //
        // On silicon the QSPI chip-select (NCS) reaches the flash pad only while
        // the pin is muxed to its QSPI alternate function. On the Daisy that pin
        // is PG6 = AF10. If firmware resets GPIOG while executing from XIP — e.g.
        // the HAL's GPIOG.split() does prec.enable().reset(), a peripheral-reset
        // pulse — PG6 reverts to input, NCS is no longer driven, the flash is
        // deselected, and the very next instruction fetch reads 0xFF and the CPU
        // faults. See daisy-hothouse's split_without_reset fix.
        //
        // We model this with true pin-mux fidelity: when NcsGpioModerAddress is
        // set, every XIP fetch checks the LIVE GPIO MODER + AFR for the NCS pin
        // and, if it is not the QSPI alternate function, returns 0xFF (chip
        // deselected). Left 0 (default) the check is disabled so register-poke
        // XIP tests that don't model the GPIO mux are unaffected; firmware tests
        // and the Bug D test set it. MODER offset 0x00, AFRL offset 0x20.
        public ulong NcsGpioModerAddress { get; set; } = 0;
        public int NcsPin { get; set; } = 6;
        public int NcsAlternateFunction { get; set; } = 10; // AF10 = QUADSPI on PG6

        private bool IsNcsPadRoutedToFlash()
        {
            if(NcsGpioModerAddress == 0)
            {
                return true; // pin-mux modelling disabled
            }
            try
            {
                var bus = ownerMachine.GetSystemBus(this);
                var moder = bus.ReadDoubleWord(NcsGpioModerAddress);
                var pinMode = (moder >> (NcsPin * 2)) & 0x3u; // 0b10 = alternate function
                if(pinMode != 0x2u)
                {
                    return false;
                }
                // AFRL (offset 0x20) selects WHICH alternate function; pins 0-7
                // are 4 bits each. Confirm it is the QSPI AF, so a reset that
                // clears AFR (or a remux to another AF) also disconnects NCS.
                var afrl = bus.ReadDoubleWord(NcsGpioModerAddress + 0x20);
                var pinAf = (afrl >> (NcsPin * 4)) & 0xFu;
                return pinAf == (ulong)NcsAlternateFunction;
            }
            catch(System.Exception e)
            {
                this.Log(LogLevel.Warning, "NCS pin-mux check failed ({0}); assuming connected", e.Message);
                return true;
            }
        }

        public override void Reset()
        {
            lock(locker)
            {
                pollingTokenSource.Cancel();
                pollingTokenSource = new CancellationTokenSource();

                registers.Reset();
                ClearTransferFifo();

                skipInstruction = true;
                skipAddress = true;
                skipAlternateBytes = true;
                skipData = true;
                dummyCyclesConfigured = 0;
                memoryMappedEntryPending = false;
                memoryMappedInstructionSent = false;

                IRQ.Unset();
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            return registers.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            registers.Write(offset, value);
        }

        public ushort ReadWord(long offset)
        {
            if(!CheckDataRegisterOffset(offset))
            {
                return 0;
            }
            return (ushort)ReadFromDataRegister(16);
        }

        public void WriteWord(long offset, ushort value)
        {
            if(!CheckDataRegisterOffset(offset))
            {
                return;
            }
            WriteToDataRegister(value, 16);
        }

        public byte ReadByte(long offset)
        {
            if(!CheckDataRegisterOffset(offset))
            {
                return 0;
            }
            return (byte)ReadFromDataRegister(8);
        }

        public void WriteByte(long offset, byte value)
        {
            if(!CheckDataRegisterOffset(offset))
            {
                return;
            }
            WriteToDataRegister(value, 8);
        }

        public long Size => 0x1000;

        public GPIO IRQ { get; } = new GPIO();

        // This value can be used to affect the interval between next polling events
        // since on real HW it depends on the clock speed that is configured for QSPI kernel
        public ulong PollingMultiplier { get; set; } = 1;

        private bool CheckDataRegisterOffset(long offset)
        {
            if(offset != (long)Registers.Data)
            {
                this.Log(LogLevel.Error, "This peripheral can only handle non 32-bit access at {0}. This operation has no effect", Registers.Data);
                return false;
            }
            return true;
        }

        private void DefineRegisters()
        {
            Registers.Control.Define(registers)
                .WithFlag(0, out enabled, name: "Enable")
                // ABORT self-clears once the abort completes (RM0433
                // rev8 §23.6.1). On real silicon this takes on the order
                // of nanoseconds, faster than any AHB read the CPU can
                // issue, so the observable behaviour is "always reads 0".
                // FieldMode.Write enforces this: writes still trigger the
                // callback below, but any read returns 0 regardless of
                // what was last written.
                .WithFlag(1, FieldMode.Write,
                    writeCallback: (_, value) =>
                    {
                        if(value)
                        {
                            lock(locker)
                            {
                                pollingTokenSource.Cancel();
                                pollingTokenSource = new CancellationTokenSource();
                                remainingBytesToTransfer = null;
                            }
                        }
                    },
                    name: "Abort")
                .WithReservedBits(2, 1)
                .WithTaggedFlag("Timeout counter enable", 3)
                .WithTaggedFlag("Sample shift", 4)
                .WithReservedBits(5, 1)
                .WithTaggedFlag("Dual-flash mode", 6)
                .WithTaggedFlag("Flash memory selection", 7)
                .WithValueField(8, 5, out fifoThreshold, name: "FIFO threshold level")
                .WithReservedBits(13, 3)
                .WithTaggedFlag("Transfer error interrupt enable", 16)
                .WithFlag(17, out transferCompleteIrqEnable, name: "Transfer complete interrupt enable")
                .WithFlag(18, out fifoThresholdInterruptEnable, name: "FIFO threshold interrupt enable")
                .WithFlag(19, out statusMatchInterruptEnable, name: "Status match interrupt enable")
                .WithTaggedFlag("Timeout interrupt enable", 20)
                .WithReservedBits(21, 1)
                .WithFlag(22, out pollingModeStopOnMatch, name: "Automatic status-polling mode stop")
                .WithEnumField(23, 1, out pollingMatchMode, name: "Polling match mode")
                .WithTag("Prescaler", 24, 8)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.DeviceConfiguration.Define(registers)
                .WithTaggedFlag("Clock mode (mode 0/mode 3)", 0)
                .WithReservedBits(1, 7)
                .WithTag("Chip select high time", 8, 3)
                .WithReservedBits(11, 5)
                .WithValueField(16, 5, out flashSize, name: "Flash memory size")
                .WithReservedBits(21, 11);

            Registers.Status.Define(registers)
                .WithFlag(0, FieldMode.Read, name: "Transmit error flag")
                .WithFlag(1, out transferComplete, FieldMode.Read, name: "Transfer complete flag")
                .WithFlag(2, out fifoThresholdReached, FieldMode.Read, name: "FIFO threshold flag")
                .WithFlag(3, out statusMatch, FieldMode.Read, name: "Status match flag")
                .WithFlag(4, FieldMode.Read, name: "Timeout flag")
                .WithFlag(5, FieldMode.Read, name: "Busy")
                .WithReservedBits(6, 2)
                .WithValueField(8, 6, FieldMode.Read,
                    valueProviderCallback: _ =>
                    {
                        switch(functionalMode.Value)
                        {
                        case ModeOfOperation.AutomaticStatusPolling:
                        case ModeOfOperation.MemoryMapped:
                            return 0;
                        case ModeOfOperation.IndirectRead:
                        case ModeOfOperation.IndirectWrite:
                            var count = transferFifo.Count;
                            return (count > MaximumFifoDepth) ? MaximumFifoDepth : (ulong)count;
                        default:
                            throw new InvalidOperationException($"Invalid mode: {functionalMode.Value}");
                        }
                    },
                    name: "FIFO level")
                .WithReservedBits(14, 18);

            Registers.FlagClear.Define(registers)
                .WithFlag(0, FieldMode.WriteOneToClear, name: "Transmit error flag clear")
                .WithFlag(1, FieldMode.WriteOneToClear, writeCallback: (_, value) => transferComplete.Value = false, name: "Transfer complete flag clear")
                .WithReservedBits(2, 1)
                .WithFlag(3, FieldMode.WriteOneToClear, writeCallback: (_, value) => statusMatch.Value = false, name: "Status match flag clear")
                .WithFlag(4, FieldMode.WriteOneToClear, name: "Timeout flag clear")
                .WithReservedBits(5, 27)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.DataLength.Define(registers)
                .WithValueField(0, 32, out dataSize, name: "Data length");

            Registers.CommunicationConfiguration.Define(registers)
                .WithValueField(0, 8, out instruction, name: "Instruction")
                .WithValueField(8, 2, writeCallback: (_, value) => skipInstruction = value == 0, name: "Instruction mode")
                .WithValueField(10, 2, writeCallback: (_, value) => skipAddress = value == 0, name: "Address mode")
                .WithValueField(12, 2, out addressSize, name: "Address size")
                .WithValueField(14, 2, writeCallback: (_, value) => skipAlternateBytes = value == 0, name: "Alternate byte mode")
                .WithValueField(16, 2, out alternateBytesSize, name: "Alternate byte size")
                .WithValueField(18, 5,
                    writeCallback: (_, val) => dummyCyclesConfigured = (int)val,
                    name: "Number of dummy cycles")
                .WithReservedBits(23, 1)
                .WithValueField(24, 2, writeCallback: (_, value) => skipData = value == 0, name: "Data mode")
                .WithEnumField(26, 2, out functionalMode,
                    writeCallback: (oldValue, value) =>
                    {
                        // Can't use `changeCallback` here, since it will trigger after the write callback, that is hooked to triggering the transfer
                        // this could cause the queue to loose all data immediately after the transfer happens, if there was a mode change directly before
                        if(oldValue != value)
                        {
                            ClearTransferFifo();
                            pollingTokenSource.Cancel();
                            pollingTokenSource = new CancellationTokenSource();
                            UpdateInterrupts();
                        }
                        // Only run the protocol validation blit when we
                        // TRANSITION into memory-mapped mode (edge, not
                        // level). Rewriting CCR while already in
                        // memory-mapped mode would re-trigger it on
                        // every write, and the bootloader touches CCR
                        // several times during init.
                        memoryMappedEntryPending =
                            (ModeOfOperation)value == ModeOfOperation.MemoryMapped
                            && oldValue != value;
                    },
                    name: "Functional mode")
                .WithFlag(28, out sendInstructionOnlyOnce, name: "Send instruction only once")
                .WithTaggedFlag("Free-running clock mode", 29)
                .WithTaggedFlag("DDR hold", 30)
                .WithTaggedFlag("DDR mode", 31)
                // This callback needs to be triggered last, after all other fields are populated and callbacks triggered.
                //
                // After the CCR is fully applied, if this CCR write
                // transitioned the peripheral INTO memory-mapped mode
                // (edge-triggered, not level — see the FMODE field
                // callback for the flag setup), drive the SPI protocol
                // for the first few bytes of flash and blit the result
                // back into MappedMemory. This surfaces controller-side
                // config bugs (like the missing mode-bits phase that hit
                // us on hardware) as corrupted CPU-visible code — see
                // ProtocolValidateMemoryMappedMode for the rationale.
                .WithWriteCallback((_, __) =>
                {
                    TriggerTransfer(TriggerTransferSource.Instruction);
                    // XIP fidelity is now served per-fetch by the [ConnectionRegion("xip")]
                    // reads (DriveMemoryMappedRead), which drive the SPI protocol against the
                    // flash on every CPU/data fetch from the 0x9000_0000 window. The old
                    // CCR-write blit into a separate MappedMemory is no longer needed.
                    if(memoryMappedEntryPending)
                    {
                        memoryMappedEntryPending = false;
                        // New memory-mapped session: re-arm SIOO so the first read
                        // sends the instruction and re-enters AX Continuous Read Mode.
                        memoryMappedInstructionSent = false;
                    }
                });

            Registers.Address.Define(registers)
                .WithValueField(0, 32, out address, name: "Address")
                .WithWriteCallback(writeCallback: (_, __) => TriggerTransfer(TriggerTransferSource.Address));

            Registers.AlternateByte.Define(registers)
                .WithValueField(0, 32, out alternateBytes, name: "Alternate bytes");

            Registers.Data.Define(registers)
                .WithValueField(0, 32,
                    writeCallback: (_, value) =>
                    {
                        WriteToDataRegister(value, 32);
                    },
                    valueProviderCallback: _ =>
                    {
                        return ReadFromDataRegister(32);
                    },
                    name: "Data bytes");

            Registers.PollingStatusMask.Define(registers)
                .WithValueField(0, 32, out pollingMask, name: "Polling status mask");

            Registers.PollingStatusMatch.Define(registers)
                .WithValueField(0, 32, out pollingReferenceMatch, name: "Polling status match");

            Registers.PollingInterval.Define(registers)
                .WithValueField(0, 16, out pollingInterval, name: "Polling interval")
                .WithReservedBits(16, 16);
        }

        private void ClearTransferFifo()
        {
            transferFifo.Clear();
            remainingBytesToTransfer = null;
        }

        // For clarity, sizes are given in number of bits
        private void CheckDataRegisterAccessSize(int size)
        {
            if(size % 8 != 0 || size / 8 > 4)
            {
                throw new ArgumentException($"Size {size} is not properly constrained, cannot access data register");
            }
        }

        private void WriteToDataRegister(ulong val, int size)
        {
            CheckDataRegisterAccessSize(size);
            if(functionalMode.Value != ModeOfOperation.IndirectWrite)
            {
                this.Log(LogLevel.Error, "Data register should only be written in {0} mode. This operation has no effect", ModeOfOperation.IndirectWrite);
                return;
            }
            lock(locker)
            {
                // The QUADSPI shifts the data register out LSB-first: byte 0
                // of the transfer is DR[7:0]. GetBytesFromValue defaults to
                // MSB-first (big-endian), so without `reverse: true` a 1-byte
                // write of 0x40 would enqueue [0x00,0x00,0x00,0x40] and the
                // transfer's front byte would be 0x00 — silently sending the
                // wrong byte (and reversing every multi-byte page program).
                // reverse: true gives LSB-first [0x40,0x00,0x00,0x00], so the
                // byte the firmware placed in DR[7:0] is what reaches the chip.
                // Mirrors ReadFromDataRegister, which unpacks little-endian.
                transferFifo.EnqueueRange(BitHelper.GetBytesFromValue(val, size / 8, reverse: true));
                UpdateInterrupts();
                TriggerTransfer(TriggerTransferSource.DataWrite);
            }
        }

        private uint ReadFromDataRegister(int size)
        {
            CheckDataRegisterAccessSize(size);
            if(!(functionalMode.Value == ModeOfOperation.IndirectRead || functionalMode.Value == ModeOfOperation.AutomaticStatusPolling))
            {
                this.Log(LogLevel.Error, "Data register should not be read in mode: {0}. Returning 0", functionalMode.Value);
                return 0;
            }
            byte[] response = new byte[4];
            lock(locker)
            {
                for(int i = 0; i < size / 8; ++i)
                {
                    if(transferFifo.TryDequeue(out var val))
                    {
                        // Push the value back into the queue immediately in AutomaticStatusPolling mode, so it can be read from Data register again
                        // It's guaranteed in `ReceiveData` that the queue won't grow beyond 4-byte limit
                        if(functionalMode.Value == ModeOfOperation.AutomaticStatusPolling)
                        {
                            transferFifo.Enqueue(val);
                        }
                    }
                    else
                    {
                        this.Log(LogLevel.Warning, "Read from empty transfer FIFO, containing less data than 32 bits. Filling with zeroes");
                    }
                    response[i] = val;
                }
                if(functionalMode.Value == ModeOfOperation.AutomaticStatusPolling)
                {
                    // In case of polling, if less than the full size of the queue is requested, pop and push the data back, so it's not corrupted
                    for(int i = size / 8; i < 4; ++i)
                    {
                        if(transferFifo.TryDequeue(out var val))
                        {
                            transferFifo.Enqueue(val);
                        }
                    }
                }
                UpdateInterrupts();
                TriggerTransfer(TriggerTransferSource.DataRead);
            }
            return BitHelper.ToUInt32(response, 0, 4, true);
        }

        private void UpdateInterrupts()
        {
            bool thresholdReached = functionalMode.Value == ModeOfOperation.IndirectRead && (transferFifo.Count >= (int)fifoThreshold.Value);
            thresholdReached |= functionalMode.Value == ModeOfOperation.IndirectWrite && ((MaximumFifoDepth - transferFifo.Count) >= (int)fifoThreshold.Value);
            fifoThresholdReached.Value = thresholdReached;

            bool status = (transferComplete.Value && transferCompleteIrqEnable.Value)
                          || (statusMatch.Value && statusMatchInterruptEnable.Value)
                          || (fifoThresholdReached.Value && fifoThresholdInterruptEnable.Value);

            this.Log(LogLevel.Noisy, "IRQ set to: {0}", status);
            IRQ.Set(status);
        }

        private void SplitToBytesAndSend(IValueRegisterField value, int size, string name)
        {
            var bytes = BitHelper.GetBytesFromValue(value.Value, size);
            this.Log(LogLevel.Debug, "Sending {0}: {1}", name, Misc.PrettyPrintCollectionHex(bytes));
            foreach(var part in bytes)
            {
                RegisteredPeripheral.Transmit(part);
            }
        }

        private void TriggerTransfer(TriggerTransferSource source)
        {
            if(!enabled.Value)
            {
                this.Log(LogLevel.Warning, "Trying to trigger SPI transfer, but the peripheral is disabled");
                return;
            }
            if(functionalMode.Value == ModeOfOperation.MemoryMapped)
            {
                this.Log(LogLevel.Debug, "The peripheral is in MemoryMapped mode, all transfers are ignored");
                return;
            }
            this.Log(LogLevel.Debug, "Trying to trigger SPI transfer, source: {0}, mode: {1}", source, functionalMode.Value);

            lock(locker)
            {
                // If more data is expected in the command sequence, delay the transfer
                // until the data is written into the relevant register
                if(!VerifyIfTransferShouldTrigger(source))
                {
                    return;
                }

                // If a transfer requires more data to complete, then skip immediately to the data phase
                if(remainingBytesToTransfer == null)
                {
                    if(!skipInstruction)
                    {
                        this.Log(LogLevel.Debug, "Sending command: 0x{0:X}", instruction.Value);
                        RegisteredPeripheral.Transmit((byte)instruction.Value);
                    }

                    if(!skipAddress)
                    {
                        SplitToBytesAndSend(address, (int)addressSize.Value + 1, "address");
                    }

                    if(!skipAlternateBytes)
                    {
                        SplitToBytesAndSend(alternateBytes, (int)alternateBytesSize.Value + 1, "alternate bytes");
                    }
                }

                if(!skipData)
                {
                    TransferData();
                    HandlePollingData();
                }
                else
                {
                    // Transfer is only complete here, if there was no data to send
                    // since this function is re-entrant otherwise
                    RegisteredPeripheral.FinishTransmission();
                    transferComplete.Value = true;
                }
                UpdateInterrupts();
            }
        }

        // --- Memory-mapped (XIP) protocol validation ---------------------
        //
        // The CPU reads XIP code from a plain `MappedMemory` at
        // 0x9000_0000 (declared in daisy_seed.repl as qspiFlashBacking),
        // because Renode's CPU translator refuses to fetch instructions
        // from a non-memory sysbus region. That means CPU AHB reads
        // never go through this peripheral, so a broken CCR wouldn't
        // naturally surface as corrupted code.
        //
        // We close the gap here. On every CCR write that lands the
        // controller in memory-mapped mode, `ProtocolValidateMemoryMappedMode`
        // drives one round of the CCR-configured SPI protocol against
        // the attached flash chip for the first `ProtocolValidationBytes`
        // bytes of flash, buffers the protocol-returned bytes, and blits
        // them back into the MappedMemory via
        // `IS25LP064A_Fixed.DirectWrite`. That way:
        //
        //   * Good config (e.g. 0xEB with proper mode-bits phase) →
        //     protocol returns the correct bytes → MappedMemory unchanged
        //     → CPU executes correctly.
        //   * Bad config (e.g. 0xEB with ABMODE=0) → protocol
        //     mis-parses phases → returns off-by-one/garbage bytes →
        //     MappedMemory gets overwritten → CPU crashes or diverges.
        //
        // The validation is bounded to the first N bytes (default 4 KiB)
        // rather than the full 8 MiB for perf; N is enough to contain the
        // vector table and the first few pages of startup code — anything
        // downstream that a broken config would corrupt still shows up
        // when the CPU tries to fetch from further in.

        // Small enough to run in milliseconds (each byte = one chip
        // Transmit + one DirectWrite), large enough to cover the ARM
        // vector table (0x100 bytes = SP + reset + system handlers +
        // first 43 external IRQ slots). If the CCR config mis-parses
        // phases, the corrupted SP / reset-vector alone hang the CPU
        // — no need to blit further to prove the bug.
        private const int ProtocolValidationBytes = 256;

        private void ProtocolValidateMemoryMappedMode()
        {
            var chip = RegisteredPeripheral as IS25LP064A_Fixed;
            if(chip == null)
            {
                return; // no compatible chip attached; skip
            }
            if(!enabled.Value || functionalMode.Value != ModeOfOperation.MemoryMapped)
            {
                return;
            }
            if(skipData)
            {
                this.Log(LogLevel.Warning,
                    "CCR entered memory-mapped mode with DMODE=0 (no data phase). XIP would return nothing on real HW.");
                return;
            }

            // One long protocol transaction reads the first N bytes.
            // Real hardware in AX Continuous Read Mode does the same
            // over many CE# cycles, but at the byte-serial level a
            // single-CE# read is protocol-equivalent (the chip's state
            // machine transitions Command → Address → ModeBits →
            // Dummies → Data → …Data), and much faster in Renode.
            var buffer = new byte[ProtocolValidationBytes];
            const long baseAddr = 0;

            if(!skipInstruction)
            {
                chip.Transmit((byte)instruction.Value);
            }
            if(!skipAddress)
            {
                var abytes = (int)addressSize.Value + 1;
                for(int i = abytes - 1; i >= 0; i--)
                {
                    chip.Transmit((byte)(baseAddr >> (i * 8)));
                }
            }
            if(!skipAlternateBytes)
            {
                var abBytes = (int)alternateBytesSize.Value + 1;
                for(int i = abBytes - 1; i >= 0; i--)
                {
                    chip.Transmit((byte)(alternateBytes.Value >> (i * 8)));
                }
            }
            var dummyBytesToSend = ComputeDummyBytes();
            for(int i = 0; i < dummyBytesToSend; i++)
            {
                chip.Transmit(0);
            }
            for(int i = 0; i < ProtocolValidationBytes; i++)
            {
                buffer[i] = chip.Transmit(0);
            }
            chip.FinishTransmission();

            // Blit the protocol-produced bytes back to MappedMemory. If
            // the CCR config is correct, this is a no-op (protocol
            // returned the same bytes that were already there). If it's
            // broken, the bytes are corrupted and the CPU will fault
            // when it tries to execute or read them.
            for(int i = 0; i < ProtocolValidationBytes; i++)
            {
                chip.DirectWrite(baseAddr + i, buffer[i]);
            }

            this.Log(LogLevel.Debug,
                "STM32H7_QuadSPI_Fixed: protocol validation blit complete ({0} bytes at 0x9000_0000+, dummy bytes sent: {1})",
                ProtocolValidationBytes, dummyBytesToSend);
        }

        // Datasheet-accurate mapping from DCYC (SCK cycles) to
        // byte-serial dummy-byte count, factoring in the current data
        // mode. In quad mode each SCK cycle transfers 4 bits per lane
        // (2 SCK per byte), in dual 4 SCK per byte, in single 8 SCK
        // per byte. `Ceiling` matches how the wire actually holds
        // silent time until the next full byte boundary.
        //
        // Upstream Renode uses the coarse `<=8 → 1, <=16 → 2, …` mapping
        // which is only correct for single-lane data. For 0xEB Fast
        // Read Quad I/O with DCYC=6 (IS25LP064A default per §8.24 read
        // parameters), the wire has 24 bits of dummy time = 3 bytes,
        // NOT 1. Our flash-chip model expects that same count for its
        // Dummies → Data phase transition to happen at the right
        // byte boundary.
        private int ComputeDummyBytes()
        {
            if(dummyCyclesConfigured == 0)
            {
                return 0;
            }
            // Data-mode lanes: DMODE 01=1, 10=2, 11=4. If data phase is
            // skipped or DMODE=0 (indirect), fall back to single-lane
            // accounting since there's no data phase whose bit-rate
            // would define byte alignment.
            int lanes = 1;
            if(!skipData)
            {
                lanes = ExtractLaneCount(Registers.CommunicationConfiguration, 24);
            }
            // Bits per byte-clock = lanes.
            // Bytes = ceil(cycles * lanes / 8).
            int bits = dummyCyclesConfigured * lanes;
            return (bits + 7) / 8;
        }

        private int ExtractLaneCount(Registers reg, int bitOffset)
        {
            var raw = (registers.Read((long)reg) >> bitOffset) & 0x3;
            return raw switch
            {
                0 => 0, // skipped
                1 => 1,
                2 => 2,
                3 => 4,
                _ => 1,
            };
        }

        // ================= XIP (memory-mapped) execute-in-place path =============
        //
        // The controller is registered a second time over the 0x9000_0000 window as
        // a BusMultiRegistration "xip" region (see stubs.robot). That range is flagged
        // executable-IO (via IExecutableInPlaceMemory), so every CPU instruction/data
        // fetch from it routes through the system bus into the [ConnectionRegion("xip")]
        // methods below. Each read replays the CCR-configured SPI protocol against the
        // attached IS25LP064A model, so a broken CCR (missing mode-bits phase, wrong
        // instruction, bad DCYC) returns shifted/garbage bytes and the CPU faults —
        // exactly the failure mode real silicon exhibits, and now covering the WHOLE
        // window per-fetch rather than a bounded blit at CCR-write time.

        // Replay the CCR-configured memory-mapped read protocol for `count` bytes at
        // flash address `flashAddr`. Falls back to the raw flash contents when the
        // controller isn't actively driving memory-mapped mode, so code loaded via
        // LoadELF still executes even without a bootloader configuring XIP first.
        private byte[] DriveMemoryMappedRead(long flashAddr, int count)
        {
            var result = new byte[count];
            var chip = RegisteredPeripheral as IS25LP064A_Fixed;
            if(chip == null)
            {
                return result;
            }
            if(!enabled.Value || functionalMode.Value != ModeOfOperation.MemoryMapped)
            {
                // Controller not serving XIP: return raw flash contents.
                for(int i = 0; i < count; i++)
                {
                    result[i] = chip.DirectRead(flashAddr + i);
                }
                return result;
            }
            if(!IsNcsPadRoutedToFlash())
            {
                // NCS pad is not muxed to QSPI (e.g. GPIOG was reset out of AF10
                // while executing from XIP): the flash chip select is not driven,
                // so the deselected chip floats the bus and the CPU reads 0xFF —
                // and faults on the garbage instruction. This is Bug D.
                this.Log(LogLevel.Warning,
                    "XIP fetch at 0x{0:X} while NCS pin ({1}) is NOT in the QSPI alternate "
                    + "function — flash deselected, returning 0xFF (would fault the CPU).",
                    flashAddr, NcsPin);
                for(int i = 0; i < count; i++)
                {
                    result[i] = 0xFF;
                }
                return result;
            }
            if(skipData)
            {
                // DMODE=0 in memory-mapped mode: no data phase; real HW returns nothing,
                // so leave the buffer zeroed and let the CPU fault on the bad code.
                this.Log(LogLevel.Warning,
                    "XIP read with DMODE=0 (no data phase) at 0x{0:X}; returning zeros.", flashAddr);
                return result;
            }
            lock(locker)
            {
                // Honour SIOO (Send Instruction Only Once): after the first
                // memory-mapped read the flash is in AX Continuous Read Mode and
                // expects NO command byte on subsequent reads (IS25LP064A_Fixed's
                // OnCommandByte treats the next byte as the first address byte).
                // Re-sending 0xEB here would be misparsed as an address — exactly
                // the real controller's behaviour when SIOO=1.
                if(!skipInstruction && !(sendInstructionOnlyOnce.Value && memoryMappedInstructionSent))
                {
                    chip.Transmit((byte)instruction.Value);
                }
                memoryMappedInstructionSent = true;
                if(!skipAddress)
                {
                    var abytes = (int)addressSize.Value + 1;
                    for(int i = abytes - 1; i >= 0; i--)
                    {
                        chip.Transmit((byte)(flashAddr >> (i * 8)));
                    }
                }
                if(!skipAlternateBytes)
                {
                    var abBytes = (int)alternateBytesSize.Value + 1;
                    for(int i = abBytes - 1; i >= 0; i--)
                    {
                        chip.Transmit((byte)(alternateBytes.Value >> (i * 8)));
                    }
                }
                var dummyBytes = ComputeDummyBytes();
                for(int i = 0; i < dummyBytes; i++)
                {
                    chip.Transmit(0);
                }
                for(int i = 0; i < count; i++)
                {
                    result[i] = chip.Transmit(0);
                }
                chip.FinishTransmission();
            }
            return result;
        }

        private void DirectWriteXip(long offset, byte[] data, int startingIndex, int count)
        {
            var chip = RegisteredPeripheral as IS25LP064A_Fixed;
            if(chip == null)
            {
                return;
            }
            for(int i = 0; i < count; i++)
            {
                chip.DirectWrite(offset + i, data[startingIndex + i]);
            }
        }

        // --- [ConnectionRegion("xip")] scalar accessors (the CPU-fetch path) -------
        // Reads drive the SPI protocol; writes (LoadELF, sysbus WriteX) store raw bytes
        // into the flash backing. FillAccessMethodsWithTaggedMethods requires matching
        // read+write pairs per width.

        [ConnectionRegion("xip")]
        public byte ReadByteXip(long offset)
        {
            return DriveMemoryMappedRead(offset, 1)[0];
        }

        [ConnectionRegion("xip")]
        public void WriteByteXip(long offset, byte value)
        {
            DirectWriteXip(offset, new[] { value }, 0, 1);
        }

        [ConnectionRegion("xip")]
        public ushort ReadWordXip(long offset)
        {
            var b = DriveMemoryMappedRead(offset, 2);
            return (ushort)(b[0] | (b[1] << 8));
        }

        [ConnectionRegion("xip")]
        public void WriteWordXip(long offset, ushort value)
        {
            DirectWriteXip(offset, new[] { (byte)value, (byte)(value >> 8) }, 0, 2);
        }

        [ConnectionRegion("xip")]
        public uint ReadDoubleWordXip(long offset)
        {
            var b = DriveMemoryMappedRead(offset, 4);
            return (uint)(b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24));
        }

        [ConnectionRegion("xip")]
        public void WriteDoubleWordXip(long offset, uint value)
        {
            DirectWriteXip(offset, new[] { (byte)value, (byte)(value >> 8), (byte)(value >> 16), (byte)(value >> 24) }, 0, 4);
        }

        // --- IPerConnectionRegionMultibyteAccess (block LoadELF/LoadBinary path) ----

        public byte[] ReadBytes(string connectionRegion, long offset, int count, IPeripheral context = null)
        {
            if(connectionRegion == XipRegionName)
            {
                return DriveMemoryMappedRead(offset, count);
            }
            return ReadBytes(offset, count, context);
        }

        public void WriteBytes(string connectionRegion, long offset, byte[] array, int startingIndex, int count, IPeripheral context = null)
        {
            if(connectionRegion == XipRegionName)
            {
                DirectWriteXip(offset, array, startingIndex, count);
                return;
            }
            WriteBytes(offset, array, startingIndex, count, context);
        }

        // --- IMemory-required members for the default (control-register) region -----

        public byte[] ReadBytes(long offset, int count, IPeripheral context = null)
        {
            var result = new byte[count];
            for(int i = 0; i < count; i++)
            {
                result[i] = ReadByte(offset + i);
            }
            return result;
        }

        public void WriteBytes(long offset, byte[] array, int startingIndex, int count, IPeripheral context = null)
        {
            for(int i = 0; i < count; i++)
            {
                WriteByte(offset + i, array[startingIndex + i]);
            }
        }

        public ulong ReadQuadWord(long offset)
        {
            return ReadDoubleWord(offset) | ((ulong)ReadDoubleWord(offset + 4) << 32);
        }

        public void WriteQuadWord(long offset, ulong value)
        {
            WriteDoubleWord(offset, (uint)value);
            WriteDoubleWord(offset + 4, (uint)(value >> 32));
        }

        private const string XipRegionName = "xip";

        private bool VerifyIfTransferShouldTrigger(TriggerTransferSource source)
        {
            // Check if a write to this register (identified by `source`), should result in data transfer
            // or if the peripheral is still waiting for more data (e.g. it got an instruction but it's missing data)
            switch(source)
            {
            case TriggerTransferSource.Instruction:
                if(skipInstruction && source == TriggerTransferSource.Instruction)
                {
                    // If the source is Instruction register, but instruction stage is disabled, the transfer shouldn't trigger
                    this.Log(LogLevel.Noisy, "Not transferring, {0} is disabled", nameof(TriggerTransferSource.Instruction));
                    return false;
                }
                if(skipAddress && !skipData)
                {
                    // Data without address - let the next stage determine if it's valid
                    goto case TriggerTransferSource.Address;
                }
                if(!skipAddress)
                {
                    this.Log(LogLevel.Noisy, "Address is missing, not transferring");
                    // Wait for address
                    return false;
                }
                goto case TriggerTransferSource.Address;
            case TriggerTransferSource.Address:
                if(skipAddress && source == TriggerTransferSource.Address)
                {
                    this.Log(LogLevel.Noisy, "Not transferring, {0} is disabled", nameof(TriggerTransferSource.Address));
                    return false;
                }
                if(!skipData && functionalMode.Value == ModeOfOperation.IndirectWrite)
                {
                    this.Log(LogLevel.Noisy, "Data is missing, not transferring");
                    // Wait for data
                    return false;
                }
                // It's safe to break here, instead of jumping to data, since it can be a terminal stage too
                break;
            case TriggerTransferSource.DataWrite:
                if(skipData && source == TriggerTransferSource.DataWrite)
                {
                    this.Log(LogLevel.Noisy, "Not transferring, {0} is disabled", nameof(TriggerTransferSource.DataWrite));
                    return false;
                }
                break;
            case TriggerTransferSource.DataRead:
                // Polling mode schedules transfers by its own
                if(functionalMode.Value == ModeOfOperation.AutomaticStatusPolling)
                {
                    return false;
                }
                // Only allow Data read to trigger transfer, if there already is a transfer ongoing
                // and there was not enough space in the transfer FIFO, so the transfer was suspended
                if(remainingBytesToTransfer == null)
                {
                    this.Log(LogLevel.Noisy, "Not transferring, {0} transfer is complete", nameof(TriggerTransferSource.DataRead));
                    return false;
                }
                break;
            default:
                this.ErrorLog("Trigger source {0} is unrecognized, not transferring", source);
                return false;
            }
            return true;
        }

        private bool DoesPolledDataMatch(uint polledUInt)
        {
            switch(pollingMatchMode.Value)
            {
            case PollingMatchMode.AllBits:
                // Since XOR returns 1 only on differing bits, zero means no change at all
                // AND with the mask, and if equals zero, both values match on all bits
                return ((polledUInt ^ pollingReferenceMatch.Value) & pollingMask.Value) == 0;
            case PollingMatchMode.AnyBit:
                // If the diff (AND mask) is equal to the mask bytes, that means there was no single bit matched
                return ((polledUInt ^ pollingReferenceMatch.Value) & pollingMask.Value) != pollingMask.Value;
            default:
                throw new InvalidOperationException($"Invalid match mode: {pollingMatchMode.Value}");
            }
        }

        private void HandlePollingData()
        {
            if(functionalMode.Value != ModeOfOperation.AutomaticStatusPolling)
            {
                return;
            }

            uint responseUInt = ReadFromDataRegister((int)(dataSize.Value + 1) * 8);
            bool matched = DoesPolledDataMatch(responseUInt);
            if(matched)
            {
                statusMatch.Value = true;
            }

            if(!matched || !pollingModeStopOnMatch.Value)
            {
                var nextEvent = pollingInterval.Value * PollingMultiplier;
                this.Log(LogLevel.Debug, "Polling not matched (got: 0x{0:X}, expected: 0x{1:X}, mask: 0x{2:X}), scheduling next polling in {3} microseconds",
                    responseUInt, pollingReferenceMatch.Value, pollingMask.Value, nextEvent);
                var token = pollingTokenSource.Token;
                Machine.ScheduleAction(TimeInterval.FromMicroseconds(nextEvent), _ =>
                {
                    lock(locker)
                    {
                        if(token.IsCancellationRequested)
                        {
                            return;
                        }
                        // Schedule a deferred polling action, if there was no match
                        transferFifo.Clear();
                        TransferData();
                        HandlePollingData();
                        UpdateInterrupts();
                    }
                }, $"{nameof(STM32H7_QuadSPI_Fixed)} Polling task");
            }
        }

        private void TransferData()
        {
            // Compute dummy-byte count from CCR live at transfer trigger
            // time. DCYC + DMODE both need to be current — earlier field
            // callbacks in the CCR write fire before DMODE is updated, so
            // computing at trigger time is the only way to see the final
            // config. Also datasheet-accurate: single/dual/quad lanes
            // change the SCK-cycles → byte-clocks ratio.
            var dummiesRemaining = ComputeDummyBytes();
            while(dummiesRemaining > 0)
            {
                RegisteredPeripheral.Transmit(0);
                dummiesRemaining -= 1;
            }
            if(skipData)
            {
                return;
            }
            if(functionalMode.Value == ModeOfOperation.AutomaticStatusPolling && dataSize.Value > 3)
            {
                // In Status polling only 4 bytes can be transferred at once, for the purpose of the comparison
                this.Log(LogLevel.Error, "DataSize cannot be more than 3 when in {0} mode. Automatically clamping the comparison length to 3",
                    nameof(ModeOfOperation.AutomaticStatusPolling));
                dataSize.Value = 3;
            }

            if(remainingBytesToTransfer == null)
            {
                if(dataSize.Value == uint.MaxValue)
                {
                    // Transfer all bytes, until the end of the flash memory
                    remainingBytesToTransfer = 2 << (int)(flashSize.Value + 1);
                }
                else
                {
                    remainingBytesToTransfer = (int)dataSize.Value + 1;
                }
            }

            byte d = 0;
            while((functionalMode.Value != ModeOfOperation.IndirectWrite || transferFifo.TryDequeue(out d))
                && remainingBytesToTransfer > 0 && transferFifo.Count <= MaximumFifoDepth)
            {
                --remainingBytesToTransfer;
                if(functionalMode.Value == ModeOfOperation.IndirectWrite)
                {
                    this.Log(LogLevel.Debug, "Sending byte: 0x{0:X}, bytes left: {1}", d, remainingBytesToTransfer);
                }
                var recv = RegisteredPeripheral.Transmit(d);
                if(functionalMode.Value != ModeOfOperation.IndirectWrite)
                {
                    this.Log(LogLevel.Debug, "Got byte: 0x{0:X}, bytes left: {1}", recv, remainingBytesToTransfer);
                    ReceiveData(recv);
                }
            }
            if(remainingBytesToTransfer == 0)
            {
                RegisteredPeripheral.FinishTransmission();
                transferComplete.Value = true;
                remainingBytesToTransfer = null;
                if(functionalMode.Value == ModeOfOperation.IndirectWrite)
                {
                    // Flush bytes left in the FIFO beyond DL+1 — e.g. a 32-bit DR
                    // write when DL < 4. On silicon the FIFO is flushed at
                    // transaction end; without this, the leftovers prepend to the
                    // NEXT indirect write and corrupt it. Real bug this masked:
                    // Set-Flash-Quad-Enable's WRSR writes a 32-bit 0x40 with DL=0,
                    // leaving 0x00,0x00,0x00, so a following Set-Read-Parameters
                    // programmed 0x00 instead of 0xF0 (wrong dummy-cycle config).
                    // (Read transfers keep the FIFO — it holds the received data.)
                    transferFifo.Clear();
                }
            }
        }

        private void ReceiveData(byte data)
        {
            if(functionalMode.Value == ModeOfOperation.IndirectWrite)
            {
                return;
            }
            transferFifo.Enqueue(data);
            if(functionalMode.Value == ModeOfOperation.AutomaticStatusPolling)
            {
                // Status polling implies that only the last 4 values obtained (uint32) make their way into the FIFO
                // So don't allow the queue to grow beyond this limit
                if(transferFifo.Count > 4)
                {
                    transferFifo.Dequeue();
                }
            }
        }

        private IFlagRegisterField transferCompleteIrqEnable;
        private IEnumRegisterField<PollingMatchMode> pollingMatchMode;

        private IEnumRegisterField<ModeOfOperation> functionalMode;
        // Set in the FMODE field write callback iff this CCR write is
        // transitioning INTO memory-mapped mode. Consumed at the end of
        // the CCR write callback by ProtocolValidateMemoryMappedMode.
        // Prevents re-blitting on subsequent CCR writes that stay in
        // memory-mapped mode (which the bootloader does at least once
        // when it comes back out of exit_memory_mapped for reflashing).
        private bool memoryMappedEntryPending;
        private bool memoryMappedInstructionSent;
        private int? remainingBytesToTransfer;
        // Snapshot of the raw DCYC field (0..31). Used to derive
        // byte-serial dummy counts via `ComputeDummyBytes` which
        // factors in the current DMODE.
        private int dummyCyclesConfigured;
        private bool skipData = true;
        private bool skipAlternateBytes = true;
        private bool skipAddress = true;

        private bool skipInstruction = true;
        private IFlagRegisterField sendInstructionOnlyOnce;
        private IValueRegisterField pollingInterval;
        private IValueRegisterField pollingReferenceMatch;

        private IValueRegisterField pollingMask;
        // Number of bytes in flash = 2 ** (flashSize + 1)
        private IValueRegisterField flashSize;
        private IValueRegisterField alternateBytes;
        private IValueRegisterField alternateBytesSize;
        private IValueRegisterField address;
        private IValueRegisterField dataSize;
        private IValueRegisterField instruction;
        private IValueRegisterField fifoThreshold;

        // Sizes specified here are always less by one than the real transfer size (that's how the HW handles the registers)
        // e.g. size 0 means that 1 byte is to be transferred
        private IFlagRegisterField enabled;
        private IFlagRegisterField pollingModeStopOnMatch;
        private IFlagRegisterField fifoThresholdReached;
        private IFlagRegisterField statusMatch;

        private IFlagRegisterField transferComplete;
        private IFlagRegisterField fifoThresholdInterruptEnable;
        private IFlagRegisterField statusMatchInterruptEnable;
        private IValueRegisterField addressSize;

        private CancellationTokenSource pollingTokenSource = new CancellationTokenSource();

        private readonly Queue<byte> transferFifo = new Queue<byte>();
        private readonly object locker = new object();
        private readonly IMachine ownerMachine;

        private readonly DoubleWordRegisterCollection registers;

        private const int MaximumFifoDepth = 32;

        private enum TriggerTransferSource
        {
            Instruction,
            Address,
            DataWrite,
            DataRead
        }

        private enum PollingMatchMode
        {
            AllBits = 0b0, // AND
            AnyBit = 0b1, // OR
        }

        private enum ModeOfOperation
        {
            IndirectWrite = 0b00,
            IndirectRead = 0b01,
            AutomaticStatusPolling = 0b10,
            // Note, that we can't map a flash memory to address range here
            // this has to be a property of the flash itself (e.g. GenericSpiFlash: underlyingMemory)
            MemoryMapped = 0b11,
        }

        private enum Registers
        {
            Control = 0x00,
            DeviceConfiguration = 0x04,
            Status = 0x08,
            FlagClear = 0x0C,
            DataLength = 0x10,
            CommunicationConfiguration = 0x14,
            Address = 0x18,
            AlternateByte = 0x1C,
            Data = 0x20,
            PollingStatusMask = 0x24,
            PollingStatusMatch = 0x28,
            PollingInterval = 0x2C,
            LowPowerTimeout = 0x30,
        }
    }
}