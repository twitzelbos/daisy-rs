//
// STM32H7_RCC_Clocked — an RCC model that COMPUTES the clock tree from the
// register contents the firmware programs (RM0433 §8.7), rather than mirroring
// enable→ready bits and pretending the frequency is whatever the firmware
// assumed. Copyright (c) 2026 daisy-rs contributors. MIT License.
//
// Why this exists
// ---------------
// The stock daisy RCC is a Python stub that only mirrors enable→ready bits (so
// the stm32h7xx-hal freeze() sequence terminates) and computes NO frequency.
// The DWT was then hard-pinned to 400 MHz to match the firmware's own guess — a
// circular setup in which the sim can never disagree with the firmware, so a
// mis-configured PLL is invisible. That blind spot hid a real bug: the SDRAM
// PLL2 config wrote PLLCFGR bits at the wrong offsets, yet every sim test still
// passed because nothing in sim consumed PLL2's frequency.
//
// What this models (faithful to RM0433 §8.7)
// ------------------------------------------
//   * All THREE PLLs (PLL1/2/3), each producing P/Q/R outputs. Every output is
//     gated on its PLLxON (CR) AND its DIVxyEN (PLLCFGR) enable, exactly like
//     silicon — an output that isn't enabled reads 0, so tests can assert that
//     e.g. DIVR2EN is what makes PLL2R appear.
//   * DIVMx prescalers (PLLCKSELR), DIVN/P/Q/R (PLLxDIVR), and the fractional
//     divider FRACN (PLLxFRACR) — the last gated on PLLxFRACEN (PLLCFGR).
//   * sys_ck: CFGR.SW source select + D1CFGR.D1CPRE prescaler, → DWT.
//   * Kernel-clock muxes we actually use: FMC (D1CCIPR.FMCSEL) and SAI1
//     (D2CCIP1R.SAI1SEL). These are what validate the SDRAM/SAI clock paths.
//   * RM0433 §8.7.13 range checks: warns when PLLxRGE doesn't match the VCO
//     input band, or PLLxVCOSEL doesn't match the VCO output band. These fire on
//     genuinely out-of-spec configs (silicon may tolerate them; a faithful model
//     says so out loud instead of silently computing a frequency anyway).
//
// STATED LIMITATIONS (not silent — read here before trusting a path)
// ------------------------------------------------------------------
//   * per_ck (CKPERSEL) and I2S_CKIN are NOT modelled: an FMC/SAI mux set to
//     per_ck / external returns 0 and logs a warning. The Daisy uses PLL2R (FMC)
//     and PLL3P (SAI1), which ARE modelled. Extend here if a per_ck path is used.
//   * Only the FMC + SAI1 kernel muxes are derived; other DxCCIPx consumers
//     (SPI, USART, ADC, …) are not — add them the same way when a test needs one.
//   * PLL lock is instantaneous (PLLxON→PLLxRDY same cycle); no lock latency.
//
using System.Collections.Generic;
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public sealed class STM32H7_RCC_Clocked : IDoubleWordPeripheral, IKnownSize
    {
        public STM32H7_RCC_Clocked(IMachine machine, long hseFrequency = 16000000)
        {
            this.machine = machine;
            this.hseFrequency = hseFrequency;
            Reset();
        }

        public void Reset()
        {
            regs.Clear();
            // CR reset value (RM0433 §8.7.2): HSION|HSIRDY|HSIDIVF set.
            regs[(long)Reg.CR] = (1u << 0) | (1u << 2) | (1u << 5);
            SystemClockFrequency = HsiCk(); // pre-PLL: runs off HSI
            Pll1PClock = Pll1QClock = Pll1RClock = 0;
            Pll2PClock = Pll2QClock = Pll2RClock = 0;
            Pll3PClock = Pll3QClock = Pll3RClock = 0;
            FmcKernelClock = 0;
            Sai1KernelClock = 0;
            lastLoggedSignature = null;
        }

        public long Size => 0x400;

        // --- Derived frequencies (Hz), readable from tests / monitor ---
        public ulong SystemClockFrequency { get; private set; }
        public ulong Pll1PClock { get; private set; }
        public ulong Pll1QClock { get; private set; }
        public ulong Pll1RClock { get; private set; }
        public ulong Pll2PClock { get; private set; }
        public ulong Pll2QClock { get; private set; }
        public ulong Pll2RClock { get; private set; }
        public ulong Pll3PClock { get; private set; }
        public ulong Pll3QClock { get; private set; }
        public ulong Pll3RClock { get; private set; }
        public ulong FmcKernelClock { get; private set; }
        public ulong Sai1KernelClock { get; private set; }

        public uint ReadDoubleWord(long offset)
        {
            var v = regs.TryGetValue(offset, out var stored) ? stored : 0u;
            switch((Reg)offset)
            {
            case Reg.CR:
                // Mirror every enable→ready pair the HAL polls (RM0433 §8.7.2).
                if((v & (1u << 0)) != 0) v |= 1u << 2;   // HSION → HSIRDY
                if((v & (1u << 7)) != 0) v |= 1u << 8;   // CSION → CSIRDY
                if((v & (1u << 12)) != 0) v |= 1u << 13; // HSI48ON → HSI48RDY
                if((v & (1u << 16)) != 0) v |= 1u << 17; // HSEON → HSERDY
                if((v & (1u << 24)) != 0) v |= 1u << 25; // PLL1ON → PLL1RDY
                if((v & (1u << 26)) != 0) v |= 1u << 27; // PLL2ON → PLL2RDY
                if((v & (1u << 28)) != 0) v |= 1u << 29; // PLL3ON → PLL3RDY
                break;
            case Reg.CFGR:
                // SWS[5:3] follows SW[2:0] (the clock switch completes).
                v = (v & ~(0x7u << 3)) | ((v & 0x7u) << 3);
                break;
            case Reg.BDCR:
                // LSEON (bit 0) → LSERDY (bit 1).
                if((v & 0x1) != 0) v |= 1u << 1;
                break;
            case Reg.CSR:
                // LSION (bit 0) → LSIRDY (bit 1). The HAL's freeze() polls
                // LSIRDY on the boot critical path even when no user code
                // enables the LSI — WITHOUT this the bootloader spins forever.
                if((v & 0x1) != 0) v |= 1u << 1;
                break;
            }
            return v;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            regs[offset] = value;
            // Any write that can affect the clock tree triggers a recompute.
            switch((Reg)offset)
            {
            case Reg.CR:
            case Reg.CFGR:
            case Reg.D1CFGR:
            case Reg.PLLCKSELR:
            case Reg.PLLCFGR:
            case Reg.PLL1DIVR:
            case Reg.PLL1FRACR:
            case Reg.PLL2DIVR:
            case Reg.PLL2FRACR:
            case Reg.PLL3DIVR:
            case Reg.PLL3FRACR:
            case Reg.D1CCIPR:
            case Reg.D2CCIP1R:
                RecomputeClockTree();
                break;
            }
        }

        private uint Get(Reg r) => regs.TryGetValue((long)r, out var v) ? v : 0u;

        // HSI kernel clock after HSIDIV (CR[4:3]): 64 MHz >> HSIDIV.
        private ulong HsiCk() => 64000000UL >> (int)((Get(Reg.CR) >> 3) & 0x3);

        private void RecomputeClockTree()
        {
            var cr = Get(Reg.CR);
            var cfgr = Get(Reg.PLLCFGR);
            var pllckselr = Get(Reg.PLLCKSELR);

            // PLL reference clock (RM0433 §8.7.11 PLLCKSELR.PLLSRC).
            ulong pllRef = (pllckselr & 0x3) switch
            {
                0 => HsiCk(),             // HSI (after HSIDIV)
                1 => 4000000UL,           // CSI
                2 => (ulong)hseFrequency, // HSE
                _ => 0UL,
            };

            var divm1 = (pllckselr >> 4) & 0x3F;
            var divm2 = (pllckselr >> 12) & 0x3F;
            var divm3 = (pllckselr >> 20) & 0x3F;

            // PLL1 (on=CR.24, fracen=CFGR.0, vcosel=CFGR.1, rge=CFGR[3:2], p/q/rEn=CFGR.16/17/18)
            ComputePll("PLL1", pllRef, divm1, (cr & (1u << 24)) != 0,
                Get(Reg.PLL1DIVR), Get(Reg.PLL1FRACR), (cfgr & (1u << 0)) != 0,
                (cfgr >> 1) & 1, (cfgr >> 2) & 0x3,
                (cfgr >> 16) & 1, (cfgr >> 17) & 1, (cfgr >> 18) & 1,
                out var pll1P, out var pll1Q, out var pll1R);
            // PLL2 (on=CR.26, fracen=CFGR.4, vcosel=CFGR.5, rge=CFGR[7:6], p/q/rEn=CFGR.19/20/21)
            ComputePll("PLL2", pllRef, divm2, (cr & (1u << 26)) != 0,
                Get(Reg.PLL2DIVR), Get(Reg.PLL2FRACR), (cfgr & (1u << 4)) != 0,
                (cfgr >> 5) & 1, (cfgr >> 6) & 0x3,
                (cfgr >> 19) & 1, (cfgr >> 20) & 1, (cfgr >> 21) & 1,
                out var pll2P, out var pll2Q, out var pll2R);
            // PLL3 (on=CR.28, fracen=CFGR.8, vcosel=CFGR.9, rge=CFGR[11:10], p/q/rEn=CFGR.22/23/24)
            ComputePll("PLL3", pllRef, divm3, (cr & (1u << 28)) != 0,
                Get(Reg.PLL3DIVR), Get(Reg.PLL3FRACR), (cfgr & (1u << 8)) != 0,
                (cfgr >> 9) & 1, (cfgr >> 10) & 0x3,
                (cfgr >> 22) & 1, (cfgr >> 23) & 1, (cfgr >> 24) & 1,
                out var pll3P, out var pll3Q, out var pll3R);

            Pll1PClock = pll1P; Pll1QClock = pll1Q; Pll1RClock = pll1R;
            Pll2PClock = pll2P; Pll2QClock = pll2Q; Pll2RClock = pll2R;
            Pll3PClock = pll3P; Pll3QClock = pll3Q; Pll3RClock = pll3R;

            // --- sys_ck: CFGR.SW source + D1CFGR.D1CPRE prescaler (RM0433 §8.7.6/§8.7.7) ---
            ulong sysSrc = (Get(Reg.CFGR) & 0x7) switch
            {
                0 => HsiCk(),
                1 => 4000000UL,
                2 => (ulong)hseFrequency,
                3 => pll1P,
                _ => HsiCk(),
            };
            var newSys = sysSrc / Prescale((Get(Reg.D1CFGR) >> 8) & 0xF); // D1CPRE[11:8]

            // --- FMC kernel clock (RM0433 D1CCIPR.FMCSEL[1:0]) ---
            var hclk3 = newSys / Prescale(Get(Reg.D1CFGR) & 0xF); // HPRE[3:0]
            FmcKernelClock = (Get(Reg.D1CCIPR) & 0x3) switch
            {
                0 => hclk3,   // rcc_hclk3
                1 => pll1Q,   // pll1_q_ck
                2 => pll2R,   // pll2_r_ck  ← Daisy SDRAM
                _ => Unmodelled("FMC", "per_ck"),
            };

            // --- SAI1 kernel clock (RM0433 D2CCIP1R.SAI1SEL[2:0]) ---
            Sai1KernelClock = (Get(Reg.D2CCIP1R) & 0x7) switch
            {
                0 => pll1Q,   // pll1_q_ck
                1 => pll2P,   // pll2_p_ck
                2 => pll3P,   // pll3_p_ck  ← Daisy SAI
                3 => Unmodelled("SAI1", "I2S_CKIN"),
                4 => Unmodelled("SAI1", "per_ck"),
                _ => 0UL,
            };

            if(newSys != 0 && newSys != SystemClockFrequency)
            {
                SystemClockFrequency = newSys;
                DriveDwt(newSys);
            }

            // Log whenever ANY derived frequency changes (not only sys_ck), so a
            // test that configures PLL2/PLL3 alone can assert its kernel clock.
            var sig = string.Format(
                "sys_ck = {0} pll1p = {1} pll1q = {2} pll1r = {3} pll2p = {4} pll2q = {5} pll2r = {6} "
                + "pll3p = {7} pll3q = {8} pll3r = {9} fmc_ker = {10} sai1_ker = {11}",
                SystemClockFrequency, Pll1PClock, Pll1QClock, Pll1RClock,
                Pll2PClock, Pll2QClock, Pll2RClock, Pll3PClock, Pll3QClock, Pll3RClock,
                FmcKernelClock, Sai1KernelClock);
            if(sig != lastLoggedSignature)
            {
                lastLoggedSignature = sig;
                this.Log(LogLevel.Info, "RCC clocks: {0}", sig);
            }
        }

        // Compute one PLL's P/Q/R outputs (RM0433 §8.7.13/§8.7.14). Each output is
        // gated on the PLL being ON and its divider-enable bit; FRACN is gated on
        // FRACEN — matching silicon, so a test sees 0 for anything not enabled.
        private void ComputePll(string name, ulong refClk, uint divm, bool on,
            uint divrReg, uint fracr, bool fracen, uint vcosel, uint rge,
            uint pEn, uint qEn, uint rEn,
            out ulong p, out ulong q, out ulong r)
        {
            p = q = r = 0;
            if(!on || divm == 0 || refClk == 0)
            {
                return;
            }
            ulong vcoIn = refClk / divm;
            uint divn = (divrReg & 0x1FF) + 1;        // DIVN[8:0] + 1
            uint divp = ((divrReg >> 9) & 0x7F) + 1;  // DIVP[15:9] + 1
            uint divq = ((divrReg >> 16) & 0x7F) + 1; // DIVQ[22:16] + 1
            uint divr = ((divrReg >> 24) & 0x7F) + 1; // DIVR[30:24] + 1
            double frac = fracen ? ((fracr >> 3) & 0x1FFF) / 8192.0 : 0.0; // FRACN[15:3]
            double vco = vcoIn * (divn + frac);

            if(pEn != 0) p = (ulong)(vco / divp);
            if(qEn != 0) q = (ulong)(vco / divq);
            if(rEn != 0) r = (ulong)(vco / divr);

            ValidateRanges(name, vcoIn, vco, rge, vcosel);
        }

        // RM0433 §8.7.13: PLLxRGE selects the VCO INPUT band, PLLxVCOSEL the VCO
        // OUTPUT band. A faithful model warns when the programmed band doesn't
        // contain the actual frequency (silicon may still lock; we say so).
        private void ValidateRanges(string name, ulong vcoIn, double vco, uint rge, uint vcosel)
        {
            var (rgeLo, rgeHi) = rge switch
            {
                0 => (1000000UL, 2000000UL),
                1 => (2000000UL, 4000000UL),
                2 => (4000000UL, 8000000UL),
                _ => (8000000UL, 16000000UL),
            };
            if(vcoIn < rgeLo || vcoIn > rgeHi)
            {
                this.Log(LogLevel.Warning,
                    "{0}: VCO input {1} Hz outside PLLxRGE band [{2}..{3}] Hz (RM0433 §8.7.13)",
                    name, vcoIn, rgeLo, rgeHi);
            }
            // VCOSEL: 0 = wide (192–836 MHz), 1 = medium (150–420 MHz).
            var (vcoLo, vcoHi) = vcosel == 0 ? (192000000.0, 836000000.0) : (150000000.0, 420000000.0);
            if(vco < vcoLo || vco > vcoHi)
            {
                this.Log(LogLevel.Warning,
                    "{0}: VCO output {1} Hz outside PLLxVCOSEL band [{2}..{3}] Hz (RM0433 §8.7.13)",
                    name, (ulong)vco, (ulong)vcoLo, (ulong)vcoHi);
            }
        }

        private ulong Unmodelled(string consumer, string source)
        {
            this.Log(LogLevel.Warning,
                "{0} kernel mux = {1}, which this model does not compute (returns 0). See header LIMITATIONS.",
                consumer, source);
            return 0UL;
        }

        // RM0433 prescaler encoding (D1CPRE / HPRE): high bit clear → div1, else 1<<((n&7)+1).
        private static uint Prescale(uint field)
            => (field & 0x8) == 0 ? 1u : (1u << (int)((field & 0x7) + 1));

        private void DriveDwt(ulong sysCk)
        {
            if(dwt == null)
            {
                dwt = machine.GetPeripheralsOfType<STM32H7_DWT_Clocked>().FirstOrDefault();
            }
            if(dwt != null)
            {
                dwt.Frequency = (uint)sysCk;
            }
        }

        private readonly IMachine machine;
        private readonly long hseFrequency;
        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();
        private STM32H7_DWT_Clocked dwt;
        private string lastLoggedSignature;

        private enum Reg : long
        {
            CR = 0x00,
            CFGR = 0x10,
            D1CFGR = 0x18,
            PLLCKSELR = 0x28,
            PLLCFGR = 0x2C,
            PLL1DIVR = 0x30,
            PLL1FRACR = 0x34,
            PLL2DIVR = 0x38,
            PLL2FRACR = 0x3C,
            PLL3DIVR = 0x40,
            PLL3FRACR = 0x44,
            D1CCIPR = 0x4C,
            D2CCIP1R = 0x50,
            BDCR = 0x70,
            CSR = 0x74,
        }
    }
}
