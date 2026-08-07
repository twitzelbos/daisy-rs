//
// STM32H7_ADC — Renode model for the STM32H7 ADC (v4, with LDO + calibration).
//
// RM0433 §25. Copyright (c) 2026 daisy-rs contributors. MIT License.
//
// Why this exists
// ---------------
// The base H743 platform maps ADC1/2 (0x40022000) to Renode's `STM32F0_ADC`,
// an earlier-generation model whose register map does NOT match the H7. The H7
// ADC bring-up is a multi-step handshake the stm32h7xx-hal drives in order:
//
//   power_up   : CR.DEEPPWD=0, CR.ADVREGEN=1, delay, poll ISR.LDORDY  (bit 12)
//   calibrate  : CR.ADCAL=1, poll until CR.ADCAL self-clears          (bit 31)
//   enable     : CR.ADEN=1,  poll ISR.ADRDY                           (bit 0)
//   read       : CR.ADSTART=1, poll ISR.EOC, read DR                  (bit 2 / DR)
//
// The F0 model models none of those bits, so every one of those polls spins
// forever — the ADC bring-up hangs in sim regardless of firmware correctness
// (this is why daisy-hothouse's ADC init could only be validated on silicon).
//
// Fidelity notes (all offsets/bits/reset-values verified against the RM0433
// register description via the stm32h7 PAC SVD, not just inferred from the HAL):
//   * CR reset value is 0 (RM0433 §25.4.25) — DEEPPWD is NOT set out of reset;
//     the ADC is "powered down" only in the sense that ADVREGEN=0 (LDO off).
//   * LDORDY (ISR bit 12) is read-only hardware status: set while ADVREGEN=1
//     and not in DEEPPWD. (The HAL polls it as a raw `isr & 0x1000` mask.)
//   * ADRDY (ISR bit 0) is set by hardware when the ADC is enabled (ADEN 0->1),
//     cleared by software write-1 (W1C), and cleared on disable — NOT auto-
//     derived, so a driver that clears then re-reads it sees the RM behaviour.
//   * A regular conversion completes when ADSTART is set: EOC (+EOS). ADSTART
//     self-clears only in single mode (CFGR.CONT=0); it stays set when CONT=1
//     (RM0433 §25.4.19) — a manual detail the Daisy's single reads never hit.
//   * Reading DR clears EOC (RM0433 §25.4.26). Starting a conversion while a
//     prior result is still unread (EOC set) raises OVR (overrun).
//   * ADCAL/ADSTP/JADSTP/ADDIS/LINCALRDYW1..6 are self-clearing command bits.
//
// Only the regular-conversion path the Daisy uses is modelled (single reads on
// ADC1, one channel per conversion via SQR1.SQ1). Injected/watchdog/DMA and the
// ADC2/common (0x100/0x300) blocks are accept-and-store. Analog values are
// injected via SetChannelValue to stand in for the Hothouse pots.
//
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Analog
{
    public sealed class STM32H7_ADC : IDoubleWordPeripheral, IKnownSize
    {
        public STM32H7_ADC(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public void Reset()
        {
            // RM0433 §25.4.25: ADC_CR reset value is 0 (ADVREGEN=0 → LDO off,
            // DEEPPWD=0). All status flags clear.
            regs.Clear();
        }

        public long Size => 0x400;

        // Test hook — inject the digital result a channel converts to (right-
        // aligned, up to 16-bit). Stands in for a pot wiper voltage. From the
        // monitor: `adc1 SetChannelValue 3 45000`.
        public void SetChannelValue(uint channel, uint value)
        {
            channelValues[channel] = value & 0xFFFF;
            this.Log(LogLevel.Debug, "ADC: channel {0} value set to 0x{1:X}", channel, value & 0xFFFF);
        }

        public uint ReadDoubleWord(long offset)
        {
            switch((Reg)offset)
            {
            case Reg.ISR:
                return ComputeIsr();
            case Reg.CR:
                // Self-clearing command bits complete instantly, so they must
                // read back 0 for the HAL's `while cr.<bit>().bit_is_set()`
                // polls to terminate: ADCAL (calibration), ADSTP/JADSTP (stop),
                // ADDIS (disable), and the six LINCALRDYW linearity-cal bits.
                return Get(Reg.CR) & ~(ADCAL | ADSTP | JADSTP | ADDIS | LINCALRDYW_ALL);
            case Reg.DR:
            {
                // Regular data register: digital result of the SQR1.SQ1 channel.
                // Reading it clears EOC (RM0433 §25.4.26).
                var channel = (Get(Reg.SQR1) >> SQ1_POS) & 0x1F;
                channelValues.TryGetValue(channel, out var value);
                regs[(long)Reg.ISR] = Get(Reg.ISR) & ~EOC;
                return value;
            }
            default:
                return Get((Reg)offset);
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch((Reg)offset)
            {
            case Reg.ISR:
                // Status flags (ADRDY/EOC/EOS/OVR/…) are write-1-to-clear.
                regs[(long)Reg.ISR] = Get(Reg.ISR) & ~value;
                return;
            case Reg.CR:
            {
                var prev = Get(Reg.CR);
                regs[(long)Reg.CR] = value;
                if((value & ADEN) != 0 && (prev & ADEN) == 0)
                {
                    // Enable → hardware raises ADRDY once ready.
                    regs[(long)Reg.ISR] = Get(Reg.ISR) | ADRDY;
                    this.Log(LogLevel.Debug, "ADC: enabled → ADRDY");
                }
                if((value & ADDIS) != 0)
                {
                    // Disable → ADEN and ADRDY clear.
                    regs[(long)Reg.CR] = Get(Reg.CR) & ~ADEN;
                    regs[(long)Reg.ISR] = Get(Reg.ISR) & ~ADRDY;
                }
                if((value & ADSTART) != 0)
                {
                    // Start a regular conversion. A prior unread result (EOC
                    // still set) is an overrun.
                    if((Get(Reg.ISR) & EOC) != 0)
                    {
                        regs[(long)Reg.ISR] = Get(Reg.ISR) | OVR;
                    }
                    regs[(long)Reg.ISR] = Get(Reg.ISR) | EOC | EOS;
                    // Single mode (CONT=0) self-clears ADSTART at end-of-sequence;
                    // continuous mode leaves it set.
                    if((Get(Reg.CFGR) & CONT) == 0)
                    {
                        regs[(long)Reg.CR] = Get(Reg.CR) & ~ADSTART;
                    }
                }
                if((value & ADCAL) != 0)
                {
                    this.Log(LogLevel.Debug, "ADC: calibration requested (completes instantly)");
                }
                if((value & ADVREGEN) != 0 && (prev & ADVREGEN) == 0)
                {
                    this.Log(LogLevel.Debug, "ADC: voltage regulator enabled → LDORDY");
                }
                return;
            }
            default:
                regs[offset] = value;
                return;
            }
        }

        private uint ComputeIsr()
        {
            var isr = Get(Reg.ISR); // stored: ADRDY, EOC, EOS, OVR (set above)
            var cr = Get(Reg.CR);
            // LDORDY is read-only hardware status: the internal LDO is ready
            // whenever the regulator is enabled and not in deep-power-down.
            if((cr & ADVREGEN) != 0 && (cr & DEEPPWD) == 0)
            {
                isr |= LDORDY;
            }
            else
            {
                isr &= ~LDORDY;
            }
            return isr;
        }

        private uint Get(Reg r) => regs.TryGetValue((long)r, out var v) ? v : 0u;

        private readonly IMachine machine;
        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();
        private readonly Dictionary<uint, uint> channelValues = new Dictionary<uint, uint>();

        // ISR bits (RM0433 §25.4.23)
        private const uint ADRDY = 1u << 0;
        private const uint EOC = 1u << 2;
        private const uint EOS = 1u << 3;
        private const uint OVR = 1u << 4;
        private const uint LDORDY = 1u << 12;

        // CR bits (RM0433 §25.4.25)
        private const uint ADEN = 1u << 0;
        private const uint ADDIS = 1u << 1;
        private const uint ADSTART = 1u << 2;
        private const uint ADSTP = 1u << 4;
        private const uint JADSTP = 1u << 5;
        private const uint LINCALRDYW_ALL = 0x3Fu << 22; // LINCALRDYW1..6 (bits 22-27)
        private const uint ADVREGEN = 1u << 28;
        private const uint DEEPPWD = 1u << 29;
        private const uint ADCAL = 1u << 31;

        // CFGR (RM0433 §25.4.20): CONT = bit 13.
        private const uint CONT = 1u << 13;

        // SQR1 (RM0433 §25.4.24): SQ1 = bits 10:6.
        private const int SQ1_POS = 6;

        private enum Reg : long
        {
            ISR = 0x00,
            IER = 0x04,
            CR = 0x08,
            CFGR = 0x0C,
            CFGR2 = 0x10,
            SMPR1 = 0x14,
            SMPR2 = 0x18,
            PCSEL = 0x1C,
            SQR1 = 0x30,
            SQR2 = 0x34,
            SQR3 = 0x38,
            SQR4 = 0x3C,
            DR = 0x40,
        }
    }
}
