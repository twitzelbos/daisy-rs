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
// Instruction-cache coverage (optional, when execBase/execSize are set)
// --------------------------------------------------------------------
// The M7 I-cache is likewise NOT coherent with data writes: code written into a
// region and then executed needs D-cache clean to PoU (DCCMVAU) + DSB + I-cache
// invalidate (ICIALLU/ICIMVAU) + DSB + ISB. Renode is actually *too correct* here
// — tlib invalidates its translated-block cache on a write to executed memory, so
// a MISSING ICIALLU runs fine in sim and fetches stale instructions on silicon:
// the same silent-failure shape. Over a small executable region (real memory the
// CPU runs from), the checker watches code writes (a modification), instruction
// execution (a per-PC CPU hook), and the I-cache maintenance ops, and flags an
// instruction fetch of code that was either written-but-not-cleaned-to-PoU or
// modified-after-being-cached-without-ICIALLU. Trigger conditions this guards:
// RAM-executed functions, overlay/dynamic loaders, or a bootloader that programs
// flash/RAM and jumps in the same session without a reset.
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
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class CacheCoherencyChecker : IDoubleWordPeripheral, IBytePeripheral, IKnownSize
    {
        public CacheCoherencyChecker(IMachine machine, ulong size, ulong baseAddress, uint lineBytes = 32,
            ulong execBase = 0, ulong execSize = 0)
        {
            this.machine = machine;
            this.lineBytes = lineBytes;
            this.baseAddress = baseAddress;
            this.execBase = execBase;
            data = new byte[size];
            lines = new Line[(size + lineBytes - 1) / lineBytes];
            execLines = new ExecLine[execSize > 0 ? (execSize + lineBytes - 1) / lineBytes : 0];
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

            // I-cache coverage: only wired when an executable region is configured.
            if(execLines.Length > 0)
            {
                bus.AddWatchpointHook(IcialluAddr, SysbusAccessWidth.DoubleWord, Access.Write, OnIcacheInvalidateAll);
                bus.AddWatchpointHook(IcimvauAddr, SysbusAccessWidth.DoubleWord, Access.Write, OnIcacheInvalidateByAddr);
                bus.AddWatchpointHook(DccmvauAddr, SysbusAccessWidth.DoubleWord, Access.Write, OnCleanToPou);
                // The code region is real memory the CPU executes from, so writes to
                // it don't pass through this peripheral: watch each line's word for a
                // modification, and hook each line's PC for an instruction fetch.
                var cpu = machine.SystemBus.GetCPUs().OfType<ICPUWithHooks>().FirstOrDefault();
                for(var i = 0; i < execLines.Length; i++)
                {
                    var addr = execBase + (ulong)i * lineBytes;
                    bus.AddWatchpointHook(addr, SysbusAccessWidth.DoubleWord, Access.Write, OnCodeWrite);
                    if(cpu != null)
                    {
                        cpu.AddHook(addr, OnExecuteCode);
                    }
                }
                if(cpu == null)
                {
                    this.Log(LogLevel.Warning,
                        "CacheCoherencyChecker: no ICPUWithHooks found; I-cache execution hooks not installed (I-cache checks disabled).");
                }
            }
        }

        public long Size => data.Length;

        // Violation counters — read from a robot test / monitor to assert behaviour.
        public int StaleReadViolations { get; private set; }
        public int DirtyDmaReadViolations { get; private set; }
        public int IStaleFetchViolations { get; private set; }
        public int Violations => StaleReadViolations + DirtyDmaReadViolations + IStaleFetchViolations;

        public void Reset()
        {
            Array.Clear(data, 0, data.Length);
            for(var i = 0; i < lines.Length; i++)
            {
                lines[i] = default(Line);
            }
            for(var i = 0; i < execLines.Length; i++)
            {
                execLines[i] = default(ExecLine);
            }
            StaleReadViolations = 0;
            DirtyDmaReadViolations = 0;
            IStaleFetchViolations = 0;
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
        private void OnAllClean(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value)
        {
            ForAll(CleanLine);
            ForAllExec(i => execLines[i].Dirty = false); // full D-clean reaches PoU too
        }
        private void OnAllCleanInvalidate(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value)
        {
            ForAll(CleanInvalidateLine);
            ForAllExec(i => execLines[i].Dirty = false);
        }

        private void OnCcr(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value)
        {
            dcacheEnabled = (value & (1u << 16)) != 0; // CCR.DC
        }

        // --- I-cache coherency state machine ----------------------------------
        // A code line becomes "I-cached" when executed; a write to it (CPU or DMA
        // loading code) leaves the D-cache dirty and, if already I-cached, makes
        // the I-cache copy stale. An instruction fetch of a line that is D-dirty
        // (never cleaned to PoU) or I-stale (never invalidated) is the bug.
        private void OnCodeWrite(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value)
        {
            var i = ExecIndex(a);
            if(i < 0)
            {
                return;
            }
            execLines[i].Dirty = true;            // new code sits in the D-cache, not yet at PoU
            if(execLines[i].ICached)
            {
                execLines[i].IStale = true;       // the I-cache still holds the old instructions
            }
        }

        private void OnExecuteCode(ICpuSupportingGdb cpu, ulong address)
        {
            var i = ExecIndex(address);
            if(i < 0)
            {
                return;
            }
            if(execLines[i].Dirty)
            {
                IStaleFetchViolations++;
                this.Log(LogLevel.Error,
                    "CACHE COHERENCY VIOLATION: instruction fetch @ 0x{0:X} of code the CPU wrote but did not clean to PoU — needs DCCMVAU + DSB before ICIALLU. On silicon the I-cache/prefetch refills from stale memory (the new instructions are stuck in the D-cache).",
                    address);
                execLines[i].Dirty = false;       // report once
            }
            else if(execLines[i].IStale)
            {
                IStaleFetchViolations++;
                this.Log(LogLevel.Error,
                    "CACHE COHERENCY VIOLATION: instruction fetch @ 0x{0:X} of code modified after it was cached in the I-cache, with no ICIALLU/ICIMVAU. On silicon the I-cache serves STALE instructions.",
                    address);
            }
            execLines[i].ICached = true;          // the fetch (re)fills the I-cache line
            execLines[i].IStale = false;
        }

        private void OnIcacheInvalidateAll(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value)
            => ForAllExec(InvalidateICacheLine);
        private void OnIcacheInvalidateByAddr(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value)
        {
            var i = ExecIndex(value);
            if(i >= 0)
            {
                InvalidateICacheLine(i);
            }
        }

        // DCCMVAU cleans the D-cache to the point of unification (what the I-cache
        // refills from). It clears code-line dirtiness AND any overlaid data line.
        private void OnCleanToPou(ICpuSupportingGdb cpu, ulong a, SysbusAccessWidth w, ulong value)
        {
            var i = ExecIndex(value);
            if(i >= 0)
            {
                execLines[i].Dirty = false;
            }
            AtAddr(value, CleanLine);
        }

        private void InvalidateICacheLine(int i)
        {
            execLines[i].ICached = false;
            execLines[i].IStale = false;
        }

        private int ExecIndex(ulong addr)
        {
            if(execLines.Length == 0 || addr < execBase)
            {
                return -1;
            }
            var off = addr - execBase;
            if(off >= (ulong)execLines.Length * lineBytes)
            {
                return -1;
            }
            return (int)(off / lineBytes);
        }

        private void ForAllExec(Action<int> op)
        {
            for(var i = 0; i < execLines.Length; i++)
            {
                op(i);
            }
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

        // Per code line: Dirty = written but not cleaned to PoU; ICached = fetched
        // into the I-cache; IStale = modified since being I-cached, no invalidate.
        private struct ExecLine
        {
            public bool Dirty;
            public bool ICached;
            public bool IStale;
        }

        private readonly IMachine machine;
        private readonly byte[] data;
        private readonly Line[] lines;
        private readonly ExecLine[] execLines;
        private readonly uint lineBytes;
        private ulong baseAddress;
        private readonly ulong execBase;
        private bool dcacheEnabled;

        // SCB cache-maintenance + control register addresses (Cortex-M7, PM0253).
        private const ulong CcrAddr = 0xE000ED14;      // CCR (DC = bit 16)
        private const ulong IcialluAddr = 0xE000EF50;  // ICIALLU (invalidate all I-cache)
        private const ulong IcimvauAddr = 0xE000EF58;  // ICIMVAU (invalidate I-cache by MVA)
        private const ulong DcimvacAddr = 0xE000EF5C;  // DCIMVAC (invalidate D by MVA)
        private const ulong DciswAddr = 0xE000EF60;    // DCISW  (invalidate D by set/way)
        private const ulong DccmvauAddr = 0xE000EF64;  // DCCMVAU (clean D to PoU by MVA)
        private const ulong DccmvacAddr = 0xE000EF68;  // DCCMVAC (clean D to PoC by MVA)
        private const ulong DccswAddr = 0xE000EF6C;    // DCCSW  (clean D by set/way)
        private const ulong DccimvacAddr = 0xE000EF70; // DCCIMVAC (clean+invalidate D by MVA)
        private const ulong DcciswAddr = 0xE000EF74;   // DCCISW (clean+invalidate D by set/way)
    }
}
