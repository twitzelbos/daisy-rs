//
// IS25LP064A_Fixed — Renode model for the ISSI IS25LP064A 64Mb (8MiB)
// QSPI NOR flash. Datasheet: reference/IS25LP064A datasheet.pdf (Rev A18).
//
// Copyright (c) 2026 daisy-rs contributors
// MIT License (same as Renode).
//
// Why this exists
// ---------------
// Renode's stock `SPI.GenericSpiFlash` recognises 0xEB (Fast Read Quad
// I/O) as just "read with 1 dummy byte". Per the IS25LP064A datasheet
// §8.7 (and Figure 8.8), the actual sequence is:
//
//     [8 clks IO0]  command (0xEB)
//     [6 clks x4]   3-byte address
//     [2 clks x4]   8-bit mode bits (REQUIRED)
//     [6 clks x4]   6 dummy cycles
//     [n clks x4]   continuous data
//
// The mode-bits phase is functional, not decorative:
//   * If mode bits M[7:4] == 1010b (0xAx), the device enters AX
//     Continuous Read Mode. Subsequent 0xEB transactions (in a new CE#
//     cycle) SKIP the command decode phase — the device starts
//     directly with address bits.
//   * If M[7:4] != 1010b, AX mode is exited and the next 0xEB
//     transaction must include the command byte.
//
// GenericSpiFlash doesn't model any of this. That gap is what let our
// bootloader's `enter_memory_mapped` config (which omitted the
// alternate-bytes phase entirely) pass every Renode test while silently
// failing on hardware.
//
// This model implements the datasheet's actual state machine, so future
// changes to our QSPI controller config get validated against the same
// protocol the real chip enforces. It's not exhaustively cycle-accurate
// — Renode's SPI backend is byte-serial — but it's accurate at the
// byte-boundary level, which is what byte-serial firmware (via the
// STM32H7 QUADSPI's per-phase byte splits) actually drives.
//
// Storage
// -------
// The chip's backing bytes live in a caller-supplied `MappedMemory`
// (declared in daisy_seed.repl as `qspiFlashBacking @ sysbus
// 0x9000_0000`). That address doubles as the CPU-visible XIP window
// so instruction fetches at 0x9000_0000 hit the same bytes the chip
// serves through the SPI protocol. `DirectRead`/`DirectWrite` are
// exposed for STM32H7_QuadSPI_Fixed's protocol-validation blit path.
//
// Command coverage
// ----------------
// See `OnCommandByte` for the full list. All commands the libDaisy
// bootloader / app code exercise are implemented, plus the security-
// relevant DP/RDPD and SFDP-read paths. Anything unrecognised logs a
// warning and no-ops for the rest of the transaction, matching how
// silicon typically ignores unknown opcodes.

using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Memory;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.SPI
{
    public sealed class IS25LP064A_Fixed : ISPIPeripheral, IGPIOReceiver
    {
        public IS25LP064A_Fixed(MappedMemory underlyingMemory)
        {
            if(underlyingMemory.Size != FlashSize)
            {
                throw new Antmicro.Renode.Exceptions.ConstructionException(
                    $"IS25LP064A_Fixed: underlyingMemory must be exactly {FlashSize} bytes; got {underlyingMemory.Size}");
            }
            this.underlyingMemory = underlyingMemory;
            PowerOnReset();
        }

        // --- Direct byte access ---------------------------------------------

        // Used by STM32H7_QuadSPI_Fixed's protocol-validation blit path to
        // overwrite MappedMemory with what the SPI protocol would return
        // for the current CCR config.
        public byte DirectRead(long offset)
        {
            return underlyingMemory.ReadByte(offset & (FlashSize - 1));
        }

        public void DirectWrite(long offset, byte value)
        {
            underlyingMemory.WriteByte(offset & (FlashSize - 1), value);
        }

        // --- ISPIPeripheral ---------------------------------------------

        public byte Transmit(byte data)
        {
            byte reply = 0xFF;
            if(deepPowerDown)
            {
                // Datasheet §8.28: In DP mode, only RDPD (0xAB) is honoured.
                // We handle the wake path in OnCommandByte too; here we just
                // ignore any other traffic and reply 0xFF.
                if(phase == Phase.Command && (data == 0xAB || data == 0x00))
                {
                    return OnCommandByte(data);
                }
                this.Log(LogLevel.Debug, "IS25LP064A: byte 0x{0:X2} ignored in Deep Power Down", data);
                return 0xFF;
            }
            switch(phase)
            {
            case Phase.Command:
                reply = OnCommandByte(data);
                break;
            case Phase.Address:
                reply = OnAddressByte(data);
                break;
            case Phase.ModeBits:
                reply = OnModeBitsByte(data);
                break;
            case Phase.Dummies:
                reply = OnDummyByte(data);
                break;
            case Phase.Data:
                reply = OnDataByte(data);
                break;
            }
            return reply;
        }

        public void FinishTransmission()
        {
            this.Log(LogLevel.Noisy, "IS25LP064A: CE# deasserted, phase={0}, addr=0x{1:X6}, byteCount={2}",
                phase, address, byteCount);

            // On CE# deassertion, latch mode-bits behaviour and reset phase.
            // A read that never got past address bytes doesn't affect mode
            // state (undefined per datasheet, but we treat it as no-op).
            if(currentOp == Op.ReadQuadIO && phase != Phase.Address && phase != Phase.Command)
            {
                if((pendingModeBits & 0xF0) == 0xA0)
                {
                    if(!axContinuousReadMode)
                    {
                        this.Log(LogLevel.Debug, "IS25LP064A: entering AX Continuous Read Mode (mode bits 0x{0:X2})", pendingModeBits);
                    }
                    axContinuousReadMode = true;
                }
                else
                {
                    if(axContinuousReadMode)
                    {
                        this.Log(LogLevel.Debug, "IS25LP064A: exiting AX Continuous Read Mode (mode bits 0x{0:X2})", pendingModeBits);
                    }
                    axContinuousReadMode = false;
                }
            }

            // Complete any pending erase/program. Real chip performs these
            // asynchronously; we do them synchronously.
            if(currentOp == Op.SectorErase4K && addressBytesReceived == AddressBytes)
            {
                EraseRange(address & ~(SectorSize4K - 1), SectorSize4K, "4KiB sector");
            }
            if(currentOp == Op.BlockErase32K && addressBytesReceived == AddressBytes)
            {
                EraseRange(address & ~(BlockSize32K - 1), BlockSize32K, "32KiB block");
            }
            if(currentOp == Op.BlockErase64K && addressBytesReceived == AddressBytes)
            {
                EraseRange(address & ~(BlockSize64K - 1), BlockSize64K, "64KiB block");
            }
            if(currentOp == Op.ChipErase)
            {
                EraseRange(0, FlashSize, "chip erase");
            }
            if((currentOp == Op.PageProgram || currentOp == Op.QuadPageProgram) && programBytesWritten > 0)
            {
                this.Log(LogLevel.Debug, "IS25LP064A: wrote {0} bytes starting at 0x{1:X6}",
                    programBytesWritten, address);
            }
            if(currentOp == Op.EnterDeepPowerDown)
            {
                deepPowerDown = true;
                this.Log(LogLevel.Debug, "IS25LP064A: entered Deep Power Down");
            }

            // WEL clears on completion of any write/erase op.
            switch(currentOp)
            {
            case Op.PageProgram:
            case Op.QuadPageProgram:
            case Op.SectorErase4K:
            case Op.BlockErase32K:
            case Op.BlockErase64K:
            case Op.ChipErase:
            case Op.WriteStatusRegister:
            case Op.WriteFunctionRegister:
                // WEL auto-clears on completion of any write/erase (datasheet
                // Table 6.2). Keep the WEL status bit (bit 1) in sync with the
                // writeEnableLatch flag — WREN/WRDI set both, so completion
                // must clear both, else RDSR keeps reporting a stale WEL=1.
                writeEnableLatch = false;
                statusRegister &= 0xFD;
                break;
            }

            // RSTEN/RST must be back-to-back per §8.31. Any other command
            // between them clears the enable.
            if(currentOp != Op.ResetEnable && currentOp != Op.Reset)
            {
                resetEnabled = false;
            }

            phase = Phase.Command;
            currentOp = Op.None;
            addressBytesReceived = 0;
            dummyBytesRemaining = 0;
            byteCount = 0;
            programBytesWritten = 0;
        }

        // Renode system reset. Models an STM32 (MCU) reset, which does NOT
        // reach the flash chip — the QSPI NOR is a SEPARATE POWER DOMAIN. So a
        // warm reboot aborts any in-flight transaction but PRESERVES the chip's
        // latched state: AX Continuous Read Mode, the Read Parameters
        // (dummy-cycle) register, the QE bit, Deep Power Down, WEL. This is the
        // exact hardware behaviour that wedged our bootloader: after a warm
        // reset the flash stays in continuous mode and ignores every
        // instruction-led command until the firmware sends a continuous-exit
        // frame. Only a power cycle (PowerCycle) or the flash software-reset
        // sequence (RSTEN 0x66 + RST 0x99) clears the latched state.
        public void Reset()
        {
            ResetTransactionState();
            this.Log(LogLevel.Debug,
                "IS25LP064A: MCU reset — transaction aborted, chip NVM/mode state PRESERVED "
                + "(AX-continuous={0}, readParams=0x{1:X2}, QE={2}). A warm MCU reset does not "
                + "reset the flash; use PowerCycle() to model unplugging the board.",
                axContinuousReadMode, readParameters, (statusRegister & 0x40) != 0);
        }

        // Physically power-cycling the board — the flash returns to its
        // power-on defaults. On hardware this is the only reliable way to clear
        // a wedged AX-continuous state, so tests use it to model unplugging the
        // Daisy. Exposed publicly so a `.robot` can invoke it via `machine` API.
        public void PowerCycle()
        {
            PowerOnReset();
            this.Log(LogLevel.Debug, "IS25LP064A: power-on reset — all latched state cleared");
        }

        // Full power-on reset: transaction state AND every latched chip state.
        // Invoked by the constructor, by PowerCycle(), and by the in-band flash
        // software reset (0x66 then 0x99), which DOES fully reset the chip.
        private void PowerOnReset()
        {
            ResetTransactionState();
            axContinuousReadMode = false;
            writeEnableLatch = false;
            statusRegister = 0x00;
            functionRegister = 0x00;
            // Datasheet Table 6.7 Read Parameters register. The dummy-cycle
            // selector is P4:P3 = bits[4:3]; the volatile power-on default
            // (Table 6.7 "Default") is P7:P5=111 (max drive strength) with
            // P4:P0=0 → 0xE0, i.e. P4:P3=00 → 6 dummy cycles for 0xEB
            // (Table 6.10, and those 6 INCLUDE the 2 AX/mode-bit cycles).
            // A bootloader that drives the STM32 QUADSPI with an 8-bit
            // alternate-byte phase (2 cycles) plus DCYC=6 clocks 8 pre-data
            // cycles, so it MUST issue 0xC0 Set Read Parameters to raise the
            // flash to 8 cycles (0xF0) — otherwise every XIP fetch shifts by
            // one byte. See DummyBytesFor0xEBFromReadParameters.
            readParameters = 0xE0;
            deepPowerDown = false;
            qpiMode = false;
        }

        // Per-transaction shift/phase state only. An MCU reset deasserts CE#,
        // which aborts any command mid-flight but leaves latched NVM/mode state.
        private void ResetTransactionState()
        {
            phase = Phase.Command;
            currentOp = Op.None;
            address = 0;
            addressBytesReceived = 0;
            dummyBytesRemaining = 0;
            byteCount = 0;
            programBytesWritten = 0;
            pendingModeBits = 0;
            quadReadWithoutQE = false;
            resetEnabled = false;
        }

        // --- IGPIOReceiver (chip select handled by the QSPI controller) ---

        public void OnGPIO(int number, bool value)
        {
            // Renode's convention: pin 0 = CE#, active-low.
            if(number == 0 && value)
            {
                this.Log(LogLevel.Noisy, "IS25LP064A: chip select deasserted via GPIO");
                FinishTransmission();
            }
        }

        // --- Phase handlers ---------------------------------------------

        private byte OnCommandByte(byte data)
        {
            // If we're in AX Continuous Read Mode, subsequent 0xEB reads
            // skip the command byte per datasheet §8.7. In that case the
            // first byte of the transaction is actually the first address
            // byte. We detect this by staying in AX mode across CE# cycles
            // and having the controller drive with SIOO=1 (skips inst).
            if(axContinuousReadMode)
            {
                // Real chip: no command clocks, first thing on the wire is
                // the address. In Renode's byte-serial world, the QSPI
                // controller with SIOO=1 skips Transmit(cmd) on the second
                // and later transactions, so what we receive here IS the
                // first address byte for a continuation ReadQuadIO.
                this.Log(LogLevel.Noisy, "IS25LP064A: AX-continuation read (no command byte)");
                currentOp = Op.ReadQuadIO;
                phase = Phase.Address;
                addressBytesReceived = 0;
                address = 0;
                byteCount = 0;
                quadReadWithoutQE = (statusRegister & 0x40) == 0;
                // A continuation read is a full 0xEB read minus the command
                // clocks: it still has the mode-bits + dummy phases, so the
                // dummy counter must be re-armed exactly as the initial 0xEB
                // command does (case 0xEB below). Without this it stays 0 from
                // the previous transaction and the data phase starts early,
                // shifting every returned byte. (Only reachable via per-fetch
                // XIP reads; the old single-transaction blit never hit it.)
                dummyBytesRemaining = DummyBytesFor0xEBFromReadParameters();
                return OnAddressByte(data);
            }

            // Wake from Deep Power Down: only 0xAB (RDPD) is honoured (also
            // NOP 0x00 to keep the state machine tidy).
            if(deepPowerDown)
            {
                if(data == 0xAB)
                {
                    deepPowerDown = false;
                    currentOp = Op.ReleaseDeepPowerDown;
                    phase = Phase.Data;
                    this.Log(LogLevel.Debug, "IS25LP064A: exiting Deep Power Down (RDPD)");
                    return 0xFF;
                }
                if(data == 0x00)
                {
                    phase = Phase.Command;
                    return 0xFF;
                }
                this.Log(LogLevel.Warning, "IS25LP064A: command 0x{0:X2} ignored in Deep Power Down", data);
                phase = Phase.Command;
                return 0xFF;
            }

            currentOp = Op.None;
            addressBytesReceived = 0;
            address = 0;

            switch(data)
            {
            case 0x00: // NOP
                phase = Phase.Command;
                break;
            case 0x03: // NORD — Normal Read
                currentOp = Op.Read;
                phase = Phase.Address;
                dummyBytesRemaining = 0;
                break;
            case 0x0B: // FRD — Fast Read
                currentOp = Op.FastRead;
                phase = Phase.Address;
                dummyBytesRemaining = 1; // 1 dummy byte after address
                break;
            case 0x3B: // FRDIO_D — Fast Read Dual Output (1-1-2)
                currentOp = Op.FastRead;
                phase = Phase.Address;
                dummyBytesRemaining = 1;
                break;
            case 0xBB: // FRDIO — Fast Read Dual I/O
                currentOp = Op.FastRead;
                phase = Phase.Address;
                dummyBytesRemaining = 1;
                break;
            case 0x6B: // FRQO — Fast Read Quad Output (1-1-4)
                currentOp = Op.FastRead;
                phase = Phase.Address;
                dummyBytesRemaining = 1;
                break;
            case 0xEB: // FRQIO — Fast Read Quad I/O (§8.7)
                currentOp = Op.ReadQuadIO;
                phase = Phase.Address;
                dummyBytesRemaining = DummyBytesFor0xEBFromReadParameters();
                // §8.7: QE (status bit 6) must be 1 before a quad read. On
                // real silicon a 0xEB with QE clear is not serviced as a quad
                // command; we flag it so the Data phase returns 0xFF.
                quadReadWithoutQE = (statusRegister & 0x40) == 0;
                if(quadReadWithoutQE)
                {
                    this.Log(LogLevel.Warning,
                        "IS25LP064A: 0xEB Fast Read Quad I/O issued with QE (status bit 6) clear — "
                        + "datasheet §8.7 requires QE=1; returning 0xFF.");
                }
                break;
            case 0x02: // PP — Page Program
                if(!RequireWriteEnable("Page Program")) return 0xFF;
                currentOp = Op.PageProgram;
                phase = Phase.Address;
                break;
            case 0x32: // PPQ — Quad Input Page Program (1-1-4)
            case 0x38: // PPQI — Quad Input Page Program (1-4-4)
                if(!RequireWriteEnable("Quad Page Program")) return 0xFF;
                currentOp = Op.QuadPageProgram;
                phase = Phase.Address;
                break;
            case 0x20: // SER — Sector Erase 4KiB
            case 0xD7:
                if(!RequireWriteEnable("Sector Erase 4KiB")) return 0xFF;
                currentOp = Op.SectorErase4K;
                phase = Phase.Address;
                break;
            case 0x52: // BER32 — Block Erase 32KiB
                if(!RequireWriteEnable("Block Erase 32KiB")) return 0xFF;
                currentOp = Op.BlockErase32K;
                phase = Phase.Address;
                break;
            case 0xD8: // BER64 — Block Erase 64KiB
                if(!RequireWriteEnable("Block Erase 64KiB")) return 0xFF;
                currentOp = Op.BlockErase64K;
                phase = Phase.Address;
                break;
            case 0xC7: // CER — Chip Erase
            case 0x60:
                if(!RequireWriteEnable("Chip Erase")) return 0xFF;
                currentOp = Op.ChipErase;
                phase = Phase.Command; // no address/data; execute on FinishTransmission
                break;
            case 0x05: // RDSR — Read Status Register
                currentOp = Op.ReadStatusRegister;
                phase = Phase.Data;
                break;
            case 0x01: // WRSR — Write Status Register
                if(!RequireWriteEnable("Write Status Register")) return 0xFF;
                currentOp = Op.WriteStatusRegister;
                phase = Phase.Data;
                break;
            case 0x48: // RDFR — Read Function Register (§8.13)
                currentOp = Op.ReadFunctionRegister;
                phase = Phase.Data;
                break;
            case 0x42: // WRFR — Write Function Register (§8.14)
                if(!RequireWriteEnable("Write Function Register")) return 0xFF;
                currentOp = Op.WriteFunctionRegister;
                phase = Phase.Data;
                break;
            case 0x06: // WREN — Write Enable
                writeEnableLatch = true;
                statusRegister |= 0x02;
                phase = Phase.Command;
                break;
            case 0x04: // WRDI — Write Disable
                writeEnableLatch = false;
                statusRegister &= 0xFD;
                phase = Phase.Command;
                break;
            case 0x66: // RSTEN — Reset Enable
                currentOp = Op.ResetEnable;
                resetEnabled = true;
                phase = Phase.Command;
                break;
            case 0x99: // RST — Reset Memory
                currentOp = Op.Reset;
                if(resetEnabled)
                {
                    // The in-band software reset DOES fully reset the chip
                    // (unlike an MCU reset) — clear all latched state.
                    PowerOnReset();
                    this.Log(LogLevel.Debug, "IS25LP064A: software reset (0x66/0x99) — full chip reset");
                }
                resetEnabled = false;
                phase = Phase.Command;
                break;
            case 0x9F: // RDJDID — Read JEDEC ID
            case 0xAF:
                currentOp = Op.ReadJEDECID;
                phase = Phase.Data;
                break;
            case 0xC0: // SRP — Set Read Parameters (§8.24)
                currentOp = Op.SetReadParameters;
                phase = Phase.Data;
                break;
            case 0x35: // QPIEN — Enter QPI mode (§8.25)
                qpiMode = true;
                this.Log(LogLevel.Debug, "IS25LP064A: QPI mode enabled");
                phase = Phase.Command;
                break;
            case 0xF5: // QPIDI — Exit QPI mode (§8.26)
                qpiMode = false;
                this.Log(LogLevel.Debug, "IS25LP064A: QPI mode disabled");
                phase = Phase.Command;
                break;
            case 0xB9: // DP — Deep Power Down (§8.28)
                currentOp = Op.EnterDeepPowerDown;
                phase = Phase.Command; // effect latches on FinishTransmission
                break;
            case 0xAB: // RDPD — Release Deep Power Down / Read Device ID (§8.29)
                currentOp = Op.ReleaseDeepPowerDown;
                phase = Phase.Data;
                break;
            case 0x5A: // RDSFDP — Read Serial Flash Discoverable Parameters
                currentOp = Op.ReadSFDP;
                phase = Phase.Address;
                dummyBytesRemaining = 1;
                break;
            case 0x90: // MFID — Read Manufacturer/Device ID
                currentOp = Op.ReadManufacturerDeviceID;
                phase = Phase.Address;
                break;
            default:
                this.Log(LogLevel.Warning, "IS25LP064A: unsupported command 0x{0:X2}", data);
                currentOp = Op.None;
                phase = Phase.Command;
                break;
            }
            return 0xFF;
        }

        private bool RequireWriteEnable(string opName)
        {
            if(!writeEnableLatch)
            {
                this.Log(LogLevel.Warning, "IS25LP064A: {0} ignored — WEL is clear", opName);
                currentOp = Op.None;
                phase = Phase.Command;
                return false;
            }
            return true;
        }

        private byte OnAddressByte(byte data)
        {
            address = (address << 8) | data;
            addressBytesReceived++;
            if(addressBytesReceived >= AddressBytes)
            {
                address &= FlashSize - 1;
                switch(currentOp)
                {
                case Op.Read:
                    phase = Phase.Data;
                    break;
                case Op.FastRead:
                    phase = Phase.Dummies;
                    break;
                case Op.ReadSFDP:
                    phase = Phase.Dummies;
                    break;
                case Op.ReadQuadIO:
                    // §8.7 sequence: after address bytes, expect the
                    // mode-bits byte before dummies begin.
                    phase = Phase.ModeBits;
                    break;
                case Op.ReadManufacturerDeviceID:
                    // §8.19: A2-A1 don't care, A0 selects manufacturer(0)/device(1)
                    phase = Phase.Data;
                    break;
                case Op.PageProgram:
                case Op.QuadPageProgram:
                case Op.SectorErase4K:
                case Op.BlockErase32K:
                case Op.BlockErase64K:
                    phase = Phase.Data;
                    break;
                }
            }
            return 0xFF;
        }

        private byte OnModeBitsByte(byte data)
        {
            pendingModeBits = data;
            this.Log(LogLevel.Noisy, "IS25LP064A: received mode bits 0x{0:X2} (M[7:4]=0x{1:X})", data, (data >> 4) & 0xF);
            phase = Phase.Dummies;
            return 0xFF;
        }

        private byte OnDummyByte(byte data)
        {
            if(dummyBytesRemaining > 0)
            {
                dummyBytesRemaining--;
                if(dummyBytesRemaining == 0)
                {
                    phase = Phase.Data;
                }
            }
            else
            {
                phase = Phase.Data;
            }
            return 0xFF;
        }

        private byte OnDataByte(byte data)
        {
            switch(currentOp)
            {
            case Op.Read:
            case Op.FastRead:
                {
                    var offset = (long)((address + byteCount) & (FlashSize - 1));
                    byteCount++;
                    return underlyingMemory.ReadByte(offset);
                }
            case Op.ReadQuadIO:
                {
                    if(quadReadWithoutQE)
                    {
                        // §8.7: quad read with QE clear is not honoured.
                        byteCount++;
                        return 0xFF;
                    }
                    var offset = (long)((address + byteCount) & (FlashSize - 1));
                    byteCount++;
                    return underlyingMemory.ReadByte(offset);
                }
            case Op.ReadSFDP:
                {
                    // 8 dummy cycles + SFDP data. Our model keeps a small
                    // in-memory SFDP parameter table (§9). Reads wrap.
                    var sfdp = SFDPBytes();
                    var idx = (int)((address + byteCount) % (uint)sfdp.Length);
                    byteCount++;
                    return sfdp[idx];
                }
            case Op.PageProgram:
            case Op.QuadPageProgram:
                {
                    // Programs are page-aligned; wrap within the page.
                    var pageBase = address & ~(PageSize - 1);
                    var pageOffset = (address & (PageSize - 1)) + byteCount;
                    var target = pageBase + (pageOffset & (PageSize - 1));
                    // Program can only clear bits (1 → 0); AND with existing.
                    var existing = underlyingMemory.ReadByte(target);
                    underlyingMemory.WriteByte(target, (byte)(existing & data));
                    byteCount++;
                    programBytesWritten++;
                    return 0xFF;
                }
            case Op.ReadStatusRegister:
                return statusRegister;
            case Op.WriteStatusRegister:
                // Datasheet Table 6.2 Note 1: WRSR must not modify WIP (bit
                // 0, read-only progress flag) or WEL (bit 1, controlled only
                // by WREN/WRDI and auto-cleared on write completion). Only
                // SRWD (bit 7), QE (bit 6) and the block-protect bits (5:2)
                // are writable — mask them in, preserve WIP/WEL.
                statusRegister = (byte)((statusRegister & 0x03) | (data & 0xFC));
                byteCount++;
                return 0xFF;
            case Op.ReadFunctionRegister:
                return functionRegister;
            case Op.WriteFunctionRegister:
                // Only OTP bits (bit 4 IRL_INV, bits 5-7 IRL0-IRL2) and TB
                // are writable; we accept the whole byte for simplicity.
                functionRegister = data;
                byteCount++;
                return 0xFF;
            case Op.ReadJEDECID:
                switch(byteCount++)
                {
                case 0: return 0x9D;       // Manufacturer ID (ISSI)
                case 1: return 0x60;       // Memory Type
                case 2: return 0x17;       // Capacity Code (8 MiB)
                default: return 0xFF;
                }
            case Op.ReadManufacturerDeviceID:
                // §8.19: A0=0 → Mfr, A0=1 → Device ID (Table 8.4)
                switch(byteCount++)
                {
                case 0: return (byte)((address & 1) == 0 ? 0x9D : 0x16);
                case 1: return (byte)((address & 1) == 0 ? 0x16 : 0x9D);
                default:
                    return (byte)(((byteCount & 1) == 0) ? 0x9D : 0x16);
                }
            case Op.ReleaseDeepPowerDown:
                // §8.29: after 3 dummy address bytes (which we don't require
                // through this path — real controller would issue address+dummy),
                // returns device ID byte 0x16 continuously.
                byteCount++;
                return 0x16;
            case Op.SetReadParameters:
                if(byteCount == 0)
                {
                    readParameters = data;
                    this.Log(LogLevel.Debug, "IS25LP064A: Set Read Parameters = 0x{0:X2}", data);
                }
                byteCount++;
                return 0xFF;
            }
            return 0xFF;
        }

        private void EraseRange(long start, long length, string what)
        {
            for(long i = 0; i < length; i++)
            {
                underlyingMemory.WriteByte((start + i) & (FlashSize - 1), 0xFF);
            }
            this.Log(LogLevel.Debug, "IS25LP064A: erased {0} at 0x{1:X6}", what, start);
        }

        // Minimal SFDP parameter table. First 16 bytes is the SFDP header,
        // then a Parameter Header pointing to a JEDEC Basic Flash Parameter
        // table. This is the shortest table that satisfies drivers that
        // probe SFDP for basic geometry.
        private static byte[] SFDPBytes()
        {
            // Layout per JESD216B. We build a static single-header response.
            var result = new byte[0x100];
            for(int i = 0; i < result.Length; i++) result[i] = 0xFF;
            // SFDP header: signature 'SFDP' (little-endian ASCII "PDFS" is
            // the on-wire order? — datasheet §9 shows bytes 0..3 = 'S','F','D','P').
            result[0x00] = (byte)'S';
            result[0x01] = (byte)'F';
            result[0x02] = (byte)'D';
            result[0x03] = (byte)'P';
            // Minor rev / major rev / NPH (0 = 1 param header) / access protocol
            result[0x04] = 0x06;  // minor rev
            result[0x05] = 0x01;  // major rev
            result[0x06] = 0x00;  // NPH = 0 (1 parameter header)
            result[0x07] = 0xFF;  // reserved / access protocol
            // Parameter Header 0: ID LSB=0x00 (JEDEC Basic), minor rev, major rev, length in DW, pointer
            result[0x08] = 0x00;  // ID LSB (JEDEC basic)
            result[0x09] = 0x06;  // minor rev
            result[0x0A] = 0x01;  // major rev
            result[0x0B] = 0x14;  // length in DWORDs (20 for JESD216B)
            result[0x0C] = 0x30;  // pointer LSB → 0x30
            result[0x0D] = 0x00;
            result[0x0E] = 0x00;
            result[0x0F] = 0xFF;  // ID MSB
            // Parameter table at 0x30: DWORD 1 (density etc) — 8 MiB = 64 Mb
            // JEDEC formula: density = 2^N - 1 bits, for 64 Mb → 0x03FF_FFFF
            result[0x30] = 0x00;  // DW1 byte 0 — 4kB erase + fast read info
            result[0x31] = 0x00;
            result[0x32] = 0x00;
            result[0x33] = 0xFF;
            result[0x34] = 0xFF;  // DW2 byte 0 — density LSB
            result[0x35] = 0xFF;
            result[0x36] = 0xFF;
            result[0x37] = 0x03;  // density MSB = 0x03FF_FFFF = 64 Mb
            return result;
        }

        // --- Types / constants ------------------------------------------

        private enum Phase { Command, Address, ModeBits, Dummies, Data }
        private enum Op {
            None,
            Read, FastRead, ReadQuadIO,
            PageProgram, QuadPageProgram,
            SectorErase4K, BlockErase32K, BlockErase64K, ChipErase,
            ReadStatusRegister, WriteStatusRegister,
            ReadFunctionRegister, WriteFunctionRegister,
            ReadJEDECID, ReadManufacturerDeviceID,
            SetReadParameters,
            EnterDeepPowerDown, ReleaseDeepPowerDown,
            ReadSFDP,
            ResetEnable, Reset,
        }

        // Datasheet-accurate dummy-byte accounting for 0xEB Fast Read Quad
        // I/O, driven by the current Read Parameters register (Table 6.7 /
        // Table 6.10). Bit layout of readParameters (Table 6.7):
        //
        //   [7:5] ODS2:0  output driver strength
        //   [4:3] P4:P3   Dummy-cycle selector
        //   [2]   P2      Wrap enable
        //   [1:0] P1:P0   Burst length
        //
        // Table 6.10, Fast Read Quad I/O (EBh) column gives the TOTAL clock
        // cycles between the address phase and data, which per Table 6.10
        // Note 3 / §8.7 INCLUDE the 2 AX/mode-bit cycles:
        //
        //   P4:P3 = 00 → 6      01 → 4      10 → 8      11 → 10
        //
        // We model the AX/mode byte as its own ModeBits phase (1 byte = 8
        // bits = 2 quad SCK cycles), so the Dummies phase must represent
        // only the cycles AFTER the mode byte: (total - 2). Quad mode packs
        // 4 bits per SCK cycle, so bytes = ceil((total-2) * 4 / 8).
        //
        //   default 0xE0 (P4:P3=00) → 6 total → 4 after mode → 2 dummy bytes
        //   libDaisy  0xF0 (P4:P3=10) → 8 total → 6 after mode → 3 dummy bytes
        //
        // The STM32H7_QuadSPI_Fixed controller sends 1 alternate byte plus
        // ComputeDummyBytes(DCYC) dummy bytes. With libDaisy's config
        // (ABSIZE=8-bit + DCYC=6 → 1 alt + 3 dummy = 4 pre-data bytes) the
        // chip's 1 mode + 3 dummy = 4 pre-data bytes align and XIP reads are
        // byte-exact. If a bootloader leaves the flash at the 6-cycle default
        // while still driving DCYC=6, the chip yields only 1 + 2 = 3 pre-data
        // bytes, so data starts one byte early and every XIP fetch shifts —
        // exactly the real-silicon failure this model now reproduces.
        private int DummyBytesFor0xEBFromReadParameters()
        {
            var cyclesIndex = (readParameters >> 3) & 0x3; // P4:P3, Table 6.7
            int totalCycles = cyclesIndex switch           // Table 6.10, EBh
            {
                0 => 6,
                1 => 4,
                2 => 8,
                3 => 10,
                _ => 6,
            };
            // Subtract the 2 AX/mode-bit cycles modelled as a separate
            // ModeBits byte, then convert quad SCK cycles → byte-serial bytes
            // (4 bits/cycle), matching the controller's ComputeDummyBytes.
            int afterModeCycles = totalCycles - 2;
            if(afterModeCycles < 0)
            {
                afterModeCycles = 0;
            }
            return (afterModeCycles * 4 + 7) / 8;
        }

        private readonly MappedMemory underlyingMemory;

        private const uint FlashSize = 8u * 1024 * 1024;
        private const uint SectorSize4K = 4 * 1024;
        private const uint BlockSize32K = 32 * 1024;
        private const uint BlockSize64K = 64 * 1024;
        private const uint PageSize = 256;
        private const int AddressBytes = 3;

        private Phase phase;
        private Op currentOp;
        private uint address;
        private int addressBytesReceived;
        private int dummyBytesRemaining;
        private uint byteCount;

        // 0xEB / AX Continuous Read Mode state
        private byte pendingModeBits;
        private bool axContinuousReadMode;
        // Set when a 0xEB quad read is issued with QE (status bit 6) clear;
        // the Data phase then returns 0xFF per datasheet §8.7.
        private bool quadReadWithoutQE;

        // Write / erase state
        private bool writeEnableLatch;
        private uint programBytesWritten;

        // Status register (§6.1)
        //   bit 0: WIP  — always 0 in this model (writes synchronous)
        //   bit 1: WEL
        //   bit 6: QE
        //   others: block-protect / SRP
        private byte statusRegister;

        // Function register (§6.2) — read latency, TB, IRL bits.
        private byte functionRegister;

        // Reset Enable sequence (§8.31): RSTEN (0x66) then RST (0x99)
        // must arrive back-to-back to be effective.
        private bool resetEnabled;

        // Read Parameters register (§8.24) — controls dummy cycle count.
        private byte readParameters;

        // Deep Power Down state (§8.28/§8.29).
        private bool deepPowerDown;

        // QPI mode (§8.25/§8.26) — informational; controller drives quad
        // lanes independently.
        private bool qpiMode;
    }
}
