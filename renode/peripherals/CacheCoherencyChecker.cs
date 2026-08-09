//
// CacheCoherencyChecker — a FUNCTIONAL Cortex-M7 D-cache/DMA coherency checker
// for Renode.
//
// Copyright (c) 2026 daisy-rs contributors. MIT License (same as Renode).
//
// Why this exists
// ---------------
// Renode's Cortex-M is a functional translator with NO cache model: cacheability
// attributes and the SCB cache-maintenance ops (DCIMVAC/DCCMVAC/DCCIMVAC/...) have
// no effect, so a DMA/CPU-shared buffer with wrong cache attributes — or a missing
// clean/invalidate around a DMA — PASSES in sim and CORRUPTS on silicon. That is
// the exact class of bug that already bit this project on real hardware (the
// "QSPI/DMA buffer corruption"). See docs/renode-fidelity.md §1.
//
// This does not add cycle accuracy or a real cache; it is a *checker*. It overlays
// a RAM region (the DMA/shared buffer), watches every access, classifies the master
// (CPU via TryGetCurrentCPU vs a foreign/DMA master), models the coherency-relevant
// per-line state, and FLAGS the two silent-failure cases:
//
//   * CPU reads a line a foreign master wrote after the CPU cached it, with no
//     intervening invalidate  → CPU would read stale cached data on HW.
//   * A foreign master reads a line the CPU wrote (dirty in cache) but never
//     cleaned → the DMA reads stale backing memory on HW.
//
// The M7 D-cache is NOT coherent with DMA (no ACE), so firmware must either place
// DMA buffers in non-cacheable memory or clean/invalidate by address. This model
// reflects those RM/PM0253 semantics.
//
// Fidelity boundary (documented, not faked)
// -----------------------------------------
// - Cacheability is ASSUMED for the overlaid region (place the checker only over a
//   region firmware treats as cacheable + write-back). We do not re-derive the MPU
//   attribute map; DC-enable (CCR.DC) is honoured when the CCR watchpoint is wired.
// - Set/way maintenance ops (DCISW/DCCSW/DCCISW) are treated as operating on ALL
//   lines (the boot-time full clean/invalidate loop), not decoded per set/way.
// - Backing memory is kept coherent in sim (both masters see real data); we only
//   TRACK the state that determines whether HW would have returned stale data.
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class CacheCoherencyChecker : IDoubleWordPeripheral, IBytePeripheral, IKnownSize
    {
        public CacheCoherencyChecker(IMachine machine, ulong size, ulong baseAddress, uint lineBytes = 32)
        {
            this.machine = machine;
            this.lineBytes = lineBytes;
            this.baseAddress = baseAddress;
            data = new byte[size];
            lines = new Line[(size + lineBytes - 1) / lineBytes];
            Reset();
            // The SCB cache-maintenance registers are inert in Renode's NVIC, so we
            // observe the CPU's writes to them via watchpoints (the written value is
            // the target address for the MVA ops). CCR.DC tracks cache-enable.
            var bus = machine.GetSystemBus(this);
            bus.AddWatchpointHook(DcimvacAddr, SysbusAccessWidth.DoubleWord, Access.Write, OnInvalidate);
            bus.AddWatchpointHook(DccimvacAddr, SysbusAccessWidth.DoubleWord, Access.Write, OnCleanInvalidate);
            bus.AddWatchpointHook(DccmvacAddr, SysbusAccessWidth.DoubleWord, Access.Write, OnClean);
            bus.AddWatchpointHook(DciswAddr, SysbusAccessWidth.DoubleWord, Access.Write, OnAllInvalidate);
            bus.AddWatchpointHook(DccswAddr, SysbusAccessWidth.DoubleWord, Access.Write, OnAllClean);
            bus.AddWatchpointHook(DcciswAddr, SysbusAccessWidth.DoubleWord, Access.Write, OnAllCleanInvalidate);
            bus.AddWatchpointHook(CcrAddr, SysbusAccessWidth.DoubleWord, Access.Write, OnCcr);
        }

        public long Size => data.Length;

        // Violation counters — read from a robot test / monitor to assert behaviour.
        public int StaleReadViolations { get; private set; }
        public int DirtyDmaReadViolations { get; private set; }
        public int Violations => StaleReadViolations + DirtyDmaReadViolations;

        public void Reset()
        {
            Array.Clear(data, 0, data.Length);
            for(var i = 0; i < lines.Length; i++)
            {
                lines[i] = default(Line);
            }
            StaleReadViolations = 0;
            DirtyDmaReadViolations = 0;
            dcacheEnabled = true; // libDaisy pre_init enables the D-cache
        }

        public uint ReadDoubleWord(long offset)
        {
            OnAccess(offset, write: false);
            return BitConverter.ToUInt32(data, (int)offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            OnAccess(offset, write: true);
            var bytes = BitConverter.GetBytes(value);
            Array.Copy(bytes, 0, data, (int)offset, 4);
        }

        public byte ReadByte(long offset)
        {
            OnAccess(offset, write: false);
            return data[offset];
        }

        public void WriteByte(long offset, byte value)
        {
            OnAccess(offset, write: true);
            data[offset] = value;
        }

        // --- Coherency state machine ------------------------------------------
        private void OnAccess(long offset, bool write)
        {
            if(!dcacheEnabled || offset < 0 || offset >= data.Length)
            {
                return;
            }
            var li = offset / lineBytes;
            var cpu = machine.SystemBus.TryGetCurrentCPU(out var _);

            if(write)
            {
                if(cpu)
                {
                    // CPU write → the CPU's cache holds newer data than backing
                    // (write-back). Now cached + dirty.
                    lines[li].CachedByCpu = true;
                    lines[li].Dirty = true;
                    lines[li].Stale = false;
                }
                else
                {
                    // Foreign (DMA) write → backing updated; a CPU-cached copy is
                    // now stale (HW would still return the old cached value).
                    if(lines[li].CachedByCpu)
                    {
                        lines[li].Stale = true;
                    }
                }
            }
            else // read
            {
                if(cpu)
                {
                    if(lines[li].Stale)
                    {
                        StaleReadViolations++;
                        this.Log(LogLevel.Error,
                            "CACHE COHERENCY VIOLATION: CPU read of STALE line @ 0x{0:X} — a DMA/foreign master wrote this cacheable buffer after the CPU cached it, and firmware did not invalidate. On silicon the CPU reads STALE cached data.",
                            offset);
                        lines[li].Stale = false; // report once per staleness
                    }
                    lines[li].CachedByCpu = true; // CPU now holds a clean cached copy
                }
                else
                {
                    // Foreign (DMA) read of a CPU-dirty line → reads stale backing.
                    if(lines[li].Dirty)
                    {
                        DirtyDmaReadViolations++;
                        this.Log(LogLevel.Error,
                            "CACHE COHERENCY VIOLATION: DMA/foreign read of DIRTY line @ 0x{0:X} — the CPU wrote this cacheable buffer (write-back) and firmware did not clean it. On silicon the DMA reads STALE backing memory.",
                            offset);
                    }
                }
            }
        }

        // --- SCB cache-maintenance interception (via watchpoints) -------------
        // The MVA ops carry the target address in the written value; the set/way
        // ops (used by the boot-time full clean/invalidate loop) touch all lines.
        private void OnInvalidate(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value) => AtAddr(value, InvalidateLine);
        private void OnClean(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value) => AtAddr(value, CleanLine);
        private void OnCleanInvalidate(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value) => AtAddr(value, CleanInvalidateLine);
        private void OnAllInvalidate(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value) => ForAll(InvalidateLine);
        private void OnAllClean(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value) => ForAll(CleanLine);
        private void OnAllCleanInvalidate(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value) => ForAll(CleanInvalidateLine);

        private void OnCcr(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value)
        {
            dcacheEnabled = (value & (1u << 16)) != 0; // CCR.DC
        }

        private void InvalidateLine(int i)
        {
            lines[i].CachedByCpu = false;
            lines[i].Stale = false;
        }

        private void CleanLine(int i)
        {
            lines[i].Dirty = false;
        }

        private void CleanInvalidateLine(int i)
        {
            InvalidateLine(i);
            CleanLine(i);
        }

        // Map an absolute maintenance target address to our overlaid line (if any).
        private void AtAddr(ulong addr, Action<int> op)
        {
            if(addr < baseAddress || addr >= baseAddress + (ulong)data.Length)
            {
                return;
            }
            op((int)((addr - baseAddress) / lineBytes));
        }

        private void ForAll(Action<int> op)
        {
            for(var i = 0; i < lines.Length; i++)
            {
                op(i);
            }
        }

        // The overlaid region's base address (needed to map MVA maintenance ops);
        // set from the `baseAddress` constructor parameter — must equal the
        // registration address. Exposed read-only for monitor introspection.
        public ulong BaseAddress => baseAddress;

        private struct Line
        {
            public bool CachedByCpu;
            public bool Dirty;
            public bool Stale;
        }

        private readonly IMachine machine;
        private readonly byte[] data;
        private readonly Line[] lines;
        private readonly uint lineBytes;
        private ulong baseAddress;
        private bool dcacheEnabled;

        // SCB cache-maintenance + control register addresses (Cortex-M7, PM0253).
        private const ulong CcrAddr = 0xE000ED14;      // CCR (DC = bit 16)
        private const ulong DciswAddr = 0xE000EF60;    // DCISW  (invalidate by set/way)
        private const ulong DcimvacAddr = 0xE000EF5C;  // DCIMVAC (invalidate by MVA)
        private const ulong DccmvacAddr = 0xE000EF68;  // DCCMVAC (clean by MVA)
        private const ulong DccswAddr = 0xE000EF6C;    // DCCSW  (clean by set/way)
        private const ulong DccimvacAddr = 0xE000EF70; // DCCIMVAC (clean+invalidate by MVA)
        private const ulong DcciswAddr = 0xE000EF74;   // DCCISW (clean+invalidate by set/way)
    }
}
