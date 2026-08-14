//
// GapGuard — detects accesses to the STM32H7's unmapped inter-region RAM gaps.
//
// Copyright (c) 2026 daisy-rs contributors. MIT License.
//
// Why this exists
// ---------------
// The on-chip RAM regions are NOT contiguous: DTCM (0x2000_0000, 128 KiB),
// AXI SRAM (0x2400_0000, 512 KiB) and D2 SRAM (0x3000_0000, 288 KiB) have
// unmapped holes between them. On silicon, accessing a hole is a bus fault.
// Stock Renode instead has the sysbus silently return 0 for unmapped reads and
// swallow unmapped writes (SystemBus.ReportNonExistingRead/Write) — so firmware
// that strays into a gap "passes" in sim but faults on hardware.
//
// That is exactly the bug behind the WM8731 DI path: a `#[link_section=".sram_d2"]`
// NOLOAD block `INSERT AFTER .bss` dragged cortex-m-rt's `__ebss` to 0x3000_0600,
// so the startup `.bss` zeroing loop ran from DTCM across the DTCM→AXI gap and
// faulted on unmapped memory before `main`. It passed the Renode boot robots
// because Renode backed the gap.
//
// Registered at the START of each gap (see `Provision Gap Guards` in stubs.robot),
// this COUNTS every access (rather than throwing a CpuAbortException, which hangs
// Renode's `RunFor` in this build) and logs an error. A robot then asserts the
// count is 0 for a healthy app and non-zero for the gap-exerciser — the same
// count-the-violations pattern as CacheCoherencyChecker. Pairs with the
// comprehensive host-side `daisy check-elf` symbol invariant.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // Same interface set as CacheCoherencyChecker (IDoubleWord + IByte + IKnownSize).
    public sealed class GapGuard : IDoubleWordPeripheral, IBytePeripheral, IKnownSize
    {
        public GapGuard(IMachine machine, ulong size)
        {
            Size = (long)size;
        }

        public long Size { get; }

        // Number of accesses seen — read from a robot: `<guardName> Accesses`.
        public ulong Accesses { get; private set; }

        public void Reset()
        {
            Accesses = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            Record(offset, false);
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            Record(offset, true);
        }

        public byte ReadByte(long offset)
        {
            Record(offset, false);
            return 0;
        }

        public void WriteByte(long offset, byte value)
        {
            Record(offset, true);
        }

        private void Record(long offset, bool write)
        {
            Accesses++;
            this.Log(LogLevel.Error,
                "{0} to an UNMAPPED RAM gap (guard offset 0x{1:X}) — a bus fault on silicon. " +
                "Most likely a startup .bss/.data init loop crossing a RAM-region boundary " +
                "(e.g. __ebss dragged into a foreign region).",
                write ? "Write" : "Read", offset);
        }
    }
}
