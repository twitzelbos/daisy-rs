//
// STM32H7_DWT_Clocked — the Cortex-M7 Data Watchpoint and Trace (DWT) unit,
// with a CYCCNT tick rate driven at runtime by the RCC clock-tree model.
//
// Copyright (c) 2026 daisy-rs contributors. MIT License (same as Renode).
//
// Authority
// ---------
// DWT is an ARM core peripheral, NOT an ST peripheral — RM0433 does not describe
// its registers. The reference is the ARMv7-M Architecture Reference Manual
// (DDI 0403E, §C1.8 "The DWT") and the Cortex-M7 TRM (DDI 0489). Register map at
// 0xE0001000.
//
// Why this exists
// ---------------
// Renode's stock Miscellaneous.DWT takes its frequency as a fixed construction
// parameter and models only CYCCNTENA + CYCCNT (NUMCOMP is an unset tag; TRCENA
// is ignored). The daisy platform previously hard-pinned DWT @ 400 MHz to match
// the firmware's OWN guess (CYCLES_PER_MS = 400_000) — circular, so a clock-config
// bug would be invisible. This model:
//   * exposes a settable Frequency so STM32H7_RCC_Clocked drives the CYCCNT rate
//     from the ACTUAL computed sys_ck (see feedback_renode_timing_fidelity);
//   * implements the DWT register behaviour the ARM ARM describes — CYCCNT gated
//     on BOTH CYCCNTENA and DEMCR.TRCENA (§C1.8.1), the DWT_CTRL capability
//     fields (NUMCOMP=4, NO* = 0 for the fully-featured M7), the profiling
//     counters, PCSR, comparators, and the software-lock registers.
//
// Fidelity boundary (documented, not faked)
// -----------------------------------------
// Renode's functional core does not expose the micro-architectural events the
// profiling counters measure (CPI stalls, exception-entry overhead, sleep
// cycles, LSU stalls, folded instructions). CPICNT/EXCCNT/SLEEPCNT/LSUCNT/
// FOLDCNT therefore hold their written value and do NOT auto-count — they are
// register-accurate but event-inert. PCSR reads 0xFFFFFFFF (the "PC sample not
// available" encoding). DWT comparators store their programmed values but do not
// themselves generate watchpoints (Renode has its own watchpoint mechanism);
// enabling one logs a warning.
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class STM32H7_DWT_Clocked : BasicDoubleWordPeripheral, IKnownSize
    {
        public STM32H7_DWT_Clocked(IMachine machine, uint frequency) : base(machine)
        {
            this.machine = machine;
            // 32-bit free-running counter: wraps at 2^32 (ARM ARM §C1.8.1).
            cycleCounter = new LimitTimer(machine.ClockSource, frequency, this, "CYCCNT",
                limit: CyccntLimit, direction: Direction.Ascending,
                workMode: WorkMode.Periodic, eventEnabled: false, autoUpdate: true);
            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            cycleCounter.Reset();
            cyccntena = false;
            cycleCounter.Enabled = false;
        }

        public long Size => 0x1000;

        // Driven by the RCC when the system clock is (re)configured. Updating the
        // LimitTimer frequency rescales how CYCCNT advances per unit of virtual
        // time, so DWT-based firmware delays reflect the real sys_ck.
        public uint Frequency
        {
            get => (uint)cycleCounter.Frequency;
            set
            {
                if(value == 0)
                {
                    return;
                }
                if(cycleCounter.Frequency != value)
                {
                    this.Log(LogLevel.Debug, "DWT CYCCNT rate set to {0} Hz by RCC", value);
                    cycleCounter.Frequency = value;
                }
            }
        }

        private void DefineRegisters()
        {
            // DWT_CTRL (§C1.8.7). Low bits are R/W control; bits [31:24] are the
            // read-only capability fields — NUMCOMP=4 and NO* = 0 for the H7's
            // fully-featured DWT (cycle + profiling counters, ext trigger, trace
            // all present). The cortex-m crate reads NUMCOMP/NOCYCCNT/NOPRFCNT.
            Registers.Control.Define(this)
                .WithFlag(0, name: "CYCCNTENA",
                    writeCallback: (_, v) => { cyccntena = v; UpdateCycleCounterEnable(); },
                    valueProviderCallback: _ => cyccntena)
                .WithValueField(1, 4, name: "POSTPRESET")
                .WithValueField(5, 4, name: "POSTINIT")
                .WithFlag(9, name: "CYCTAP")
                .WithValueField(10, 2, name: "SYNCTAP")
                .WithFlag(12, name: "PCSAMPLENA")
                .WithReservedBits(13, 3)
                .WithFlag(16, name: "EXCTRCENA")
                .WithFlag(17, name: "CPIEVTENA")
                .WithFlag(18, name: "EXCEVTENA")
                .WithFlag(19, name: "SLEEPEVTENA")
                .WithFlag(20, name: "LSUEVTENA")
                .WithFlag(21, name: "FOLDEVTENA")
                .WithFlag(22, name: "CYCEVTENA")
                .WithReservedBits(23, 1)
                .WithFlag(24, FieldMode.Read, valueProviderCallback: _ => false, name: "NOPRFCNT")
                .WithFlag(25, FieldMode.Read, valueProviderCallback: _ => false, name: "NOCYCCNT")
                .WithFlag(26, FieldMode.Read, valueProviderCallback: _ => false, name: "NOEXTTRIG")
                .WithFlag(27, FieldMode.Read, valueProviderCallback: _ => false, name: "NOTRCPKT")
                .WithValueField(28, 4, FieldMode.Read, valueProviderCallback: _ => NumComp, name: "NUMCOMP");

            // DWT_CYCCNT (§C1.8.8): 32-bit cycle counter. Counts at the processor
            // clock ONLY while CYCCNTENA and DEMCR.TRCENA are both set; software
            // may read or write it.
            Registers.CycleCounter.Define(this)
                .WithValueField(0, 32, name: "CYCCNT",
                    writeCallback: (_, v) => cycleCounter.Value = v,
                    valueProviderCallback: _ =>
                    {
                        // TRCENA may have changed via DEMCR (a different block);
                        // re-evaluate the gate, then sync so the read is current.
                        UpdateCycleCounterEnable();
                        if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
                        {
                            cpu.SyncTime();
                        }
                        return (uint)cycleCounter.Value;
                    });

            // Profiling counters (§C1.8.9-13). Register-accurate but event-inert:
            // the functional core produces none of these micro-arch events, so
            // they hold their written value (documented above). 8-bit each.
            Registers.CpiCounter.Define(this).WithValueField(0, 8, name: "CPICNT").WithReservedBits(8, 24);
            Registers.ExceptionCounter.Define(this).WithValueField(0, 8, name: "EXCCNT").WithReservedBits(8, 24);
            Registers.SleepCounter.Define(this).WithValueField(0, 8, name: "SLEEPCNT").WithReservedBits(8, 24);
            Registers.LsuCounter.Define(this).WithValueField(0, 8, name: "LSUCNT").WithReservedBits(8, 24);
            Registers.FoldCounter.Define(this).WithValueField(0, 8, name: "FOLDCNT").WithReservedBits(8, 24);

            // DWT_PCSR (§C1.8.14): PC sample. Not modelled in a functional core →
            // the architectural "sample not available" value.
            Registers.ProgramCounterSample.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => PcSampleUnavailable, name: "PCSR");

            // Comparators (§C1.8.15-17): COMPn/MASKn/FUNCTIONn, stride 0x10.
            // Stored for register completeness; watchpoint matching is not driven
            // from here (Renode's own mechanism handles that) — enabling one warns.
            for(var i = 0; i < NumComp; i++)
            {
                var index = i;
                RegAt((long)Registers.Comparator0 + index * ComparatorStride)
                    .WithValueField(0, 32, name: $"COMP{index}");
                RegAt((long)Registers.Mask0 + index * ComparatorStride)
                    .WithValueField(0, 5, name: $"MASK{index}").WithReservedBits(5, 27);
                RegAt((long)Registers.Function0 + index * ComparatorStride)
                    .WithValueField(0, 4, name: $"FUNCTION{index}",
                        writeCallback: (_, v) =>
                        {
                            if(v != 0)
                            {
                                this.Log(LogLevel.Warning,
                                    "DWT comparator {0} enabled (FUNCTION=0x{1:X}), but DWT watchpoints are not modelled", index, v);
                            }
                        })
                    .WithReservedBits(4, 28);
            }

            // Software lock (§C1.8.5-6). SLI=0 in LSR: no software lock is
            // implemented, so the DWT is always accessible and LAR writes are
            // no-ops (the cortex-m crate unlocks defensively via LAR).
            Registers.LockAccess.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "LAR");
            Registers.LockStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => 0, name: "LSR");
        }

        private DoubleWordRegister RegAt(long offset)
        {
            var register = new DoubleWordRegister(this);
            RegistersCollection.AddRegister(offset, register);
            return register;
        }

        private void UpdateCycleCounterEnable()
        {
            // ARM ARM §C1.8.1: CYCCNT counts only while DWT_CTRL.CYCCNTENA AND
            // DEMCR.TRCENA are both set. DEMCR lives in the SCB block (0xE000EDFC),
            // outside the DWT window, so read it from the bus (Renode's NVIC
            // models it). DEMCR writes don't reach this peripheral, so the gate is
            // re-evaluated on every DWT access (CTRL write, CYCCNT read) — which
            // matches how firmware brings DWT up: set DEMCR.TRCENA, THEN write
            // DWT_CTRL.CYCCNTENA (that write re-evaluates with TRCENA already set).
            // Defensive: if DEMCR can't be read, fall back to CYCCNTENA alone
            // rather than freezing the counter.
            var trcena = true;
            try
            {
                trcena = (machine.GetSystemBus(this).ReadDoubleWord(DemcrAddress) & TrcenaMask) != 0;
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Debug, "Could not read DEMCR for TRCENA gate: {0}", e.Message);
            }
            cycleCounter.Enabled = cyccntena && trcena;
        }

        private readonly IMachine machine;
        private readonly LimitTimer cycleCounter;
        private bool cyccntena;

        private const uint NumComp = 4;                 // Cortex-M7: 4 comparators
        private const ulong CyccntLimit = 1UL << 32;    // CYCCNT wraps at 2^32
        private const ulong DemcrAddress = 0xE000EDFC;  // SCB DEMCR (TRCENA @ bit 24)
        private const uint TrcenaMask = 1u << 24;
        private const uint PcSampleUnavailable = 0xFFFFFFFF;
        private const long ComparatorStride = 0x10;

        private enum Registers : long
        {
            Control = 0x000,
            CycleCounter = 0x004,
            CpiCounter = 0x008,
            ExceptionCounter = 0x00C,
            SleepCounter = 0x010,
            LsuCounter = 0x014,
            FoldCounter = 0x018,
            ProgramCounterSample = 0x01C,
            Comparator0 = 0x020,
            Mask0 = 0x024,
            Function0 = 0x028,
            LockAccess = 0xFB0,
            LockStatus = 0xFB4,
        }
    }
}
