//
// STM32H7_PWR — Renode model for the STM32H7 Power control block.
//
// RM0433 §7.5. Copyright (c) 2026 daisy-rs contributors. MIT License.
//
// Why this exists
// ---------------
// The stock daisy PWR is a Python stub (and the base H743 platform only TAGS
// PWR). This C# model belongs to the H750 platform and models the pieces the
// stm32h7xx-hal freeze() path and daisy-boot depend on, with real VOS-level
// behaviour instead of blindly asserting the ready bits:
//   * CR1.DBP (bit 8) — disable backup-domain write protection (sticky R/W).
//   * D3CR.VOS[15:14] — voltage scaling select; VOSRDY (bit 13) becomes set
//     once a valid scale (Scale 1/2/3) is programmed. VOS0 (480 MHz) is Scale 1
//     plus the SYSCFG overdrive, which is outside PWR.
//   * CSR1.ACTVOS[15:14] mirrors the currently-active scale and ACTVOSRDY
//     (bit 13) tracks VOSRDY — both polled by pwr.freeze().
//   * CR3.LDOEN/SCUEN (bits 1/2) and USB33DEN (bit 24) → USB33RDY (bit 26),
//     polled by daisy-boot before enabling OTG_HS.
//
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public sealed class STM32H7_PWR : IDoubleWordPeripheral, IKnownSize
    {
        public STM32H7_PWR(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public void Reset()
        {
            regs.Clear();
            // D3CR reset: VOSRDY reflects the reset scale. Reset VOS = Scale 3
            // (01) per RM0433 §7.5.9; leave VOSRDY clear until firmware selects.
            regs[(long)Reg.D3CR] = 0x1u << 14; // VOS = Scale 3 default
        }

        public long Size => 0x400;

        public uint ReadDoubleWord(long offset)
        {
            var v = regs.TryGetValue(offset, out var stored) ? stored : 0u;
            switch((Reg)offset)
            {
            case Reg.D3CR:
            {
                // VOSRDY (bit 13) set whenever a valid scale (non-zero) is set.
                var vos = (v >> 14) & 0x3;
                if(vos != 0)
                {
                    v |= 1u << 13;
                }
                break;
            }
            case Reg.CSR1:
            {
                // ACTVOS[15:14] mirrors D3CR.VOS, ACTVOSRDY (bit13) tracks VOSRDY.
                var vos = (Get(Reg.D3CR) >> 14) & 0x3;
                v = (v & ~(0x3u << 14)) | (vos << 14);
                if(vos != 0)
                {
                    v |= 1u << 13;
                }
                break;
            }
            case Reg.CR3:
                // USB33DEN (bit24) → USB33RDY (bit26).
                if((v & (1u << 24)) != 0)
                {
                    v |= 1u << 26;
                }
                break;
            }
            return v;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            regs[offset] = value;
            if((Reg)offset == Reg.D3CR)
            {
                this.Log(LogLevel.Debug, "PWR: VOS set to scale {0}", (value >> 14) & 0x3);
            }
        }

        private uint Get(Reg r) => regs.TryGetValue((long)r, out var v) ? v : 0u;

        private readonly IMachine machine;
        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();

        private enum Reg : long
        {
            CR1 = 0x00,
            CSR1 = 0x04,
            CR2 = 0x08,
            CR3 = 0x0C,
            CPUCR = 0x10,
            D3CR = 0x18,
        }
    }
}
