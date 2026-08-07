//
// STM32H7_FMC_SDRAM — Renode model for the STM32H7 FMC SDRAM controller.
//
// RM0433 §22 (FMC), errata ES0392 §2.6.6/§2.6.7. Copyright (c) 2026 daisy-rs
// contributors. MIT License (same as Renode).
//
// Why this exists
// ---------------
// Renode's stock STM32H7 platform models external SDRAM as a plain, always-
// live Memory.MappedMemory at 0xC000_0000, and only TAGS the FMC control
// registers at 0x5200_4000 (they do nothing). So firmware can read/write
// SDRAM whether or not it ever ran the FMC init sequence — a missing or
// broken bring-up passes silently in sim, exactly the kind of bug the daisy
// bootloader's XIP config hid on the QSPI side.
//
// This model closes that gap for SDRAM:
//   * It owns the FMC control registers (SDCR1/2, SDTR1/2, SDCMR, SDRTR,
//     SDSR) at 0x5200_4000.
//   * It owns the 64 MiB data window at 0xC000_0000 (ConnectionRegion
//     "sdram") and leaves it UNUSABLE until the JEDEC power-up command
//     sequence completes: Clock-Configuration-Enable (MODE=001) → Precharge
//     All (010) → Auto-Refresh (011) → Load-Mode-Register (100). Reads before
//     that return 0x00 and writes are dropped (both logged) — so a test that
//     accesses SDRAM without initialising it fails, and daisy_bsp::sdram::init
//     can finally be VALIDATED in sim rather than only smoke-run.
//   * SDSR reports "not busy" so the firmware's status polls terminate.
//
// It is not cycle-accurate (no real refresh timing / self-refresh modelling),
// but it is faithful at the "did you run the init sequence" level, which is
// what functional firmware tests need.
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.MTD
{
    public sealed class STM32H7_FMC_SDRAM :
        IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>,
        IPerConnectionRegionMultibyteAccess, IKnownSize
    {
        public STM32H7_FMC_SDRAM(long sdramSize = 0x4000000)
        {
            if(sdramSize <= 0 || (sdramSize & (sdramSize - 1)) != 0)
            {
                throw new Antmicro.Renode.Exceptions.ConstructionException(
                    $"STM32H7_FMC_SDRAM: sdramSize must be a positive power of two; got 0x{sdramSize:X}");
            }
            this.sdramSize = sdramSize;
            backing = new byte[sdramSize];
            RegistersCollection = new DoubleWordRegisterCollection(this);
            DefineRegisters();
            Reset();
        }

        public void Reset()
        {
            RegistersCollection.Reset();
            clockEnabled = false;
            prechargedAll = false;
            autoRefreshed = false;
            initialized = false;
            Array.Clear(backing, 0, backing.Length);
        }

        // The primary registration is the FMC control-register block.
        public long Size => 0x1000;
        public DoubleWordRegisterCollection RegistersCollection { get; }

        public uint ReadDoubleWord(long offset) => RegistersCollection.Read(offset);
        public void WriteDoubleWord(long offset, uint value) => RegistersCollection.Write(offset, value);

        // --- FMC control registers (offsets from 0x5200_4000, RM0433 §22.7) ---
        private void DefineRegisters()
        {
            // SDCR1/SDCR2, SDTR1/SDTR2 — captured for inspection; behaviour is
            // gated on the command sequence, not on these fields.
            ((Registers)Registers.SDCR1).Define(this).WithValueField(0, 32, out sdcr1, name: "SDCR1");
            ((Registers)Registers.SDCR2).Define(this).WithValueField(0, 32, name: "SDCR2");
            ((Registers)Registers.SDTR1).Define(this).WithValueField(0, 32, out sdtr1, name: "SDTR1");
            ((Registers)Registers.SDTR2).Define(this).WithValueField(0, 32, name: "SDTR2");

            // SDCMR — Command Mode. MODE[2:0] drives the init state machine.
            ((Registers)Registers.SDCMR).Define(this)
                .WithValueField(0, 3, FieldMode.Write, writeCallback: (_, mode) => OnCommand((uint)mode), name: "MODE")
                .WithFlag(3, FieldMode.Write, name: "CTB2")
                .WithFlag(4, FieldMode.Write, name: "CTB1")
                .WithValueField(5, 4, FieldMode.Write, name: "NRFS")
                .WithValueField(9, 13, out modeRegisterDefinition, FieldMode.Write, name: "MRD")
                .WithReservedBits(22, 10);

            // SDRTR — Refresh Timer. COUNT[13:1].
            ((Registers)Registers.SDRTR).Define(this)
                .WithFlag(0, FieldMode.Write, name: "CRE")
                .WithValueField(1, 13, out refreshCount, name: "COUNT")
                .WithFlag(14, name: "REIE")
                .WithReservedBits(15, 17);

            // SDSR — Status. BUSY (bit 5) always reads 0 (ready); MODES/RE = 0.
            ((Registers)Registers.SDSR).Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => false, name: "RE")
                .WithValueField(1, 2, FieldMode.Read, valueProviderCallback: _ => 0, name: "MODES1")
                .WithValueField(3, 2, FieldMode.Read, valueProviderCallback: _ => 0, name: "MODES2")
                .WithFlag(5, FieldMode.Read, valueProviderCallback: _ => false, name: "BUSY")
                .WithReservedBits(6, 26);
        }

        private void OnCommand(uint mode)
        {
            switch(mode)
            {
            case 0: // Normal mode — no-op
                break;
            case 1: // Clock Configuration Enable
                clockEnabled = true;
                this.Log(LogLevel.Debug, "FMC SDRAM: clock configuration enabled");
                break;
            case 2: // Precharge All
                if(!clockEnabled)
                {
                    this.Log(LogLevel.Warning, "FMC SDRAM: Precharge-All before clock enable");
                }
                prechargedAll = true;
                break;
            case 3: // Auto-Refresh
                if(!prechargedAll)
                {
                    this.Log(LogLevel.Warning, "FMC SDRAM: Auto-Refresh before Precharge-All");
                }
                autoRefreshed = true;
                break;
            case 4: // Load Mode Register — final step of the JEDEC bring-up
                if(clockEnabled && prechargedAll && autoRefreshed)
                {
                    initialized = true;
                    this.Log(LogLevel.Info,
                        "FMC SDRAM: initialised (MRD=0x{0:X}); 0x{1:X8}..0x{2:X8} now usable.",
                        modeRegisterDefinition.Value, DataBase, DataBase + (ulong)sdramSize - 1);
                }
                else
                {
                    this.Log(LogLevel.Warning,
                        "FMC SDRAM: Load-Mode-Register out of sequence (clk={0} pall={1} refresh={2}); SDRAM stays unusable.",
                        clockEnabled, prechargedAll, autoRefreshed);
                }
                break;
            default:
                this.Log(LogLevel.Warning, "FMC SDRAM: unknown SDCMR MODE 0x{0:X}", mode);
                break;
            }
        }

        // --- SDRAM data window (ConnectionRegion "sdram", base 0xC000_0000) ---
        // Gated on `initialized`: before the FMC init sequence completes the
        // device is not driving the bus, so reads return 0x00 and writes are
        // dropped (real HW is undefined / the bus stalls — 0x00 + a warning is
        // the sim-friendly, test-catchable equivalent).

        private bool CheckUsable(long offset, string what)
        {
            if(!initialized)
            {
                this.Log(LogLevel.Warning,
                    "FMC SDRAM: {0} at 0x{1:X8} before init sequence completed — SDRAM not usable.",
                    what, DataBase + (ulong)offset);
                return false;
            }
            return true;
        }

        [ConnectionRegion("sdram")]
        public byte ReadByteSdram(long offset)
        {
            if(!CheckUsable(offset, "read"))
            {
                return 0;
            }
            return backing[offset & (sdramSize - 1)];
        }

        [ConnectionRegion("sdram")]
        public void WriteByteSdram(long offset, byte value)
        {
            if(!CheckUsable(offset, "write"))
            {
                return;
            }
            backing[offset & (sdramSize - 1)] = value;
        }

        [ConnectionRegion("sdram")]
        public ushort ReadWordSdram(long offset)
        {
            return (ushort)(ReadByteSdram(offset) | (ReadByteSdram(offset + 1) << 8));
        }

        [ConnectionRegion("sdram")]
        public void WriteWordSdram(long offset, ushort value)
        {
            WriteByteSdram(offset, (byte)value);
            WriteByteSdram(offset + 1, (byte)(value >> 8));
        }

        [ConnectionRegion("sdram")]
        public uint ReadDoubleWordSdram(long offset)
        {
            return (uint)(ReadByteSdram(offset)
                | (ReadByteSdram(offset + 1) << 8)
                | (ReadByteSdram(offset + 2) << 16)
                | (ReadByteSdram(offset + 3) << 24));
        }

        [ConnectionRegion("sdram")]
        public void WriteDoubleWordSdram(long offset, uint value)
        {
            WriteByteSdram(offset, (byte)value);
            WriteByteSdram(offset + 1, (byte)(value >> 8));
            WriteByteSdram(offset + 2, (byte)(value >> 16));
            WriteByteSdram(offset + 3, (byte)(value >> 24));
        }

        // --- IPerConnectionRegionMultibyteAccess (block / DMA path) ---
        public byte[] ReadBytes(string connectionRegion, long offset, int count, IPeripheral context = null)
        {
            var result = new byte[count];
            if(connectionRegion == SdramRegionName && CheckUsable(offset, "block read"))
            {
                for(int i = 0; i < count; i++)
                {
                    result[i] = backing[(offset + i) & (sdramSize - 1)];
                }
            }
            return result;
        }

        public void WriteBytes(string connectionRegion, long offset, byte[] array, int startingIndex, int count, IPeripheral context = null)
        {
            if(connectionRegion == SdramRegionName && CheckUsable(offset, "block write"))
            {
                for(int i = 0; i < count; i++)
                {
                    backing[(offset + i) & (sdramSize - 1)] = array[startingIndex + i];
                }
            }
        }

        // --- state / constants ---
        private const ulong DataBase = 0xC0000000;
        private const string SdramRegionName = "sdram";

        private readonly long sdramSize;
        private readonly byte[] backing;

        private IValueRegisterField sdcr1;
        private IValueRegisterField sdtr1;
        private IValueRegisterField modeRegisterDefinition;
        private IValueRegisterField refreshCount;

        private bool clockEnabled;
        private bool prechargedAll;
        private bool autoRefreshed;
        private bool initialized;

        private enum Registers : long
        {
            SDCR1 = 0x140,
            SDCR2 = 0x144,
            SDTR1 = 0x148,
            SDTR2 = 0x14C,
            SDCMR = 0x150,
            SDRTR = 0x154,
            SDSR = 0x158,
        }
    }
}
