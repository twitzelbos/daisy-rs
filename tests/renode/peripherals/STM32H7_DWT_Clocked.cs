//
// STM32H7_DWT_Clocked — a DWT whose CYCCNT tick rate can be driven at
// runtime by the RCC clock-tree model, instead of being hard-pinned to a
// fixed frequency in the platform file.
//
// Copyright (c) 2026 daisy-rs contributors. MIT License (same as Renode).
//
// Why this exists
// ---------------
// Renode's stock Miscellaneous.DWT takes its frequency as a construction
// parameter and never changes it. The daisy platform therefore hard-codes
// DWT @ 400 MHz to match the firmware's OWN assumption (CYCLES_PER_MS =
// 400_000) — a circular setup where the sim can never disagree with the
// firmware about the clock, so a clock-config bug (HAL not landing on
// 400 MHz) would be invisible. This variant exposes a settable `Frequency`
// so STM32H7_RCC_Clocked can drive the CYCCNT rate from the ACTUAL computed
// sys_ck, breaking the circularity. See feedback_renode_timing_fidelity.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class STM32H7_DWT_Clocked : BasicDoubleWordPeripheral, IKnownSize
    {
        public STM32H7_DWT_Clocked(IMachine machine, uint frequency) : base(machine)
        {
            this.machine = machine;
            cycleCounter = new LimitTimer(machine.ClockSource, frequency, this, "CycleCounter", direction: Direction.Ascending);
            CreateRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            cycleCounter.Reset();
        }

        public long Size => 0x1000;

        // Driven by the RCC when the system clock is (re)configured. Updating
        // the LimitTimer frequency rescales how CYCCNT advances per unit of
        // virtual time, so DWT-based firmware delays reflect the real sys_ck.
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

        private void CreateRegisters()
        {
            Registers.Control.Define(this)
                .WithFlag(0, writeCallback: (_, val) => cycleCounter.Enabled = val,
                    valueProviderCallback: _ => cycleCounter.Enabled, name: "CYCCNTENA")
                .WithReservedBits(1, 31);
            Registers.CycleCounter.Define(this)
                .WithValueField(0, 32, writeCallback: (_, val) => cycleCounter.Value = val,
                    valueProviderCallback: _ =>
                    {
                        if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
                        {
                            cpu.SyncTime();
                        }
                        return (uint)cycleCounter.Value;
                    }, name: "CYCCNT");
        }

        private readonly IMachine machine;
        private readonly LimitTimer cycleCounter;

        private enum Registers : long
        {
            Control = 0x0,
            CycleCounter = 0x4,
        }
    }
}
