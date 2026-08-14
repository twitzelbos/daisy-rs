//
// GapGuard — faults on access to the STM32H7's unmapped inter-region RAM gaps.
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
// that strays into a gap "passes" in sim but locks up on hardware.
//
// That is exactly the bug behind the WM8731 DI path: a `#[link_section=".sram_d2"]`
// NOLOAD block `INSERT AFTER .bss` dragged cortex-m-rt's `__ebss` to 0x3000_0600,
// so the startup `.bss` zeroing loop ran from DTCM across the DTCM→AXI gap and
// hung the M7 before `main`. It passed the Renode boot robots because Renode
// backed the gap.
//
// Registered over the two gaps (see `Provision Gap Guards` in stubs.robot), this
// turns any access into a CPU abort, so such a loop locks up in sim just like
// silicon — a red CI check. Pairs with the host-side `daisy check-elf` invariant.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public sealed class GapGuard : IDoubleWordPeripheral, IWordPeripheral, IBytePeripheral, IKnownSize
    {
        public GapGuard(IMachine machine, ulong size)
        {
            Size = (long)size;
        }

        public long Size { get; }

        public void Reset()
        {
        }

        public uint ReadDoubleWord(long offset)
        {
            Abort(offset, false);
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            Abort(offset, true);
        }

        public ushort ReadWord(long offset)
        {
            Abort(offset, false);
            return 0;
        }

        public void WriteWord(long offset, ushort value)
        {
            Abort(offset, true);
        }

        public byte ReadByte(long offset)
        {
            Abort(offset, false);
            return 0;
        }

        public void WriteByte(long offset, byte value)
        {
            Abort(offset, true);
        }

        private void Abort(long offset, bool write)
        {
            this.Log(LogLevel.Error,
                "{0} to an UNMAPPED RAM gap (guard offset 0x{1:X}) — a bus fault on silicon. " +
                "Most likely a startup .bss/.data init loop crossing a RAM-region boundary " +
                "(e.g. __ebss dragged into a foreign region). Aborting the CPU to match hardware.",
                write ? "Write" : "Read", offset);
            throw new CpuAbortException("access to unmapped RAM gap (bus-faults on hardware)");
        }
    }
}
