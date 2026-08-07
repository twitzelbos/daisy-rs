//
// STM32H7_OTG — Renode model for the STM32H7 USB OTG (Synopsys DWC_OTG) core,
// DEVICE mode only (the Daisy's on-board micro-USB is OTG2 @ 0x40080000 used as
// a USB device). Host mode is deliberately out of scope.
//
// RM0433 §59. Copyright (c) 2026 daisy-rs contributors. MIT License.
//
// Why this exists
// ---------------
// The base platform TAGS the OTG regions, so every register reads 0 — and the
// synopsys-usb-otg core-init spins forever on GRSTCTL.AHBIDL / CSRST and
// GINTSTS.CMOD (verified against vendor/synopsys-usb-otg/src/bus.rs). This
// model implements the OTG register behaviour RM0433 §59 describes so the HAL's
// UsbBus init completes and, across the later phases, device enumeration and
// transfers can be exercised.
//
// PHASE 1 — global core block (§59.15.1). Reset (CSRST + RX/TX FIFO flush,
// AHB-idle), the GINTSTS/GINTMSK/GAHBCFG interrupt mechanism (with the NVIC
// line and a RaiseEvent stimulus hook), current-mode (CMOD = device), core ID,
// and FIFO-size/config registers (stored). Device registers, the RX/TX FIFO
// packet path and enumeration are added in later phases; until then those
// offsets accept-and-store so init still completes.
//
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.USB
{
    public sealed class STM32H7_OTG : IDoubleWordPeripheral, IKnownSize
    {
        public STM32H7_OTG(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public GPIO IRQ { get; } = new GPIO();

        public long Size => 0x40000; // whole OTG2 window (core + FIFOs), per the tag

        public void Reset()
        {
            regs.Clear();
            UpdateIrq();
        }

        // Test/stimulus hook: raise OTG interrupt-status bits, exactly as a
        // hardware event (bus reset, enum-done, RX-FIFO-non-empty, …) would.
        // The later host-stimulus phases drive enumeration through this.
        public void RaiseEvent(uint gintstsBits)
        {
            regs[(long)Reg.GINTSTS] = Get(Reg.GINTSTS) | gintstsBits;
            UpdateIrq();
        }

        // Test/stimulus hook: raise an endpoint interrupt (DIEPINTx / DOEPINTx),
        // as a completed transfer / received SETUP would. Feeds the DAINT →
        // GINTSTS.IEPINT/OEPINT aggregation.
        public void RaiseEndpointInterrupt(bool inEndpoint, uint endpoint, uint bits)
        {
            var addr = (inEndpoint ? DIEP_BASE : DOEP_BASE) + endpoint * EP_STRIDE + EP_INT_OFFSET;
            regs[addr] = GetRaw(addr) | bits;
            UpdateIrq();
        }

        public uint ReadDoubleWord(long offset)
        {
            // Per-endpoint interrupt registers are write-1-to-clear status; the
            // IN endpoints' TX-FIFO-status always reports space free.
            if(InRange(offset, DIEP_BASE, DIEP_BASE + NumEndpoints * EP_STRIDE)
                && (offset - DIEP_BASE) % EP_STRIDE == EP_TXFSTS_OFFSET)
            {
                return 0x0000_0100; // DTXFSTSx: FIFO space available (words)
            }

            switch((Reg)offset)
            {
            case Reg.GRSTCTL:
                // Self-clearing command bits read back 0 (the init loops poll
                // `while CSRST/RXFFLSH/TXFFLSH == 1`); AHB master is always idle
                // in a functional (non-cycle-accurate) model.
                return (Get(Reg.GRSTCTL) & ~(CSRST | RXFFLSH | TXFFLSH)) | AHBIDL;
            case Reg.GINTSTS:
                return ComputeGintsts();
            case Reg.CID:
                // Synopsys core ID — must land in the H7 family arm of
                // synopsys-usb-otg's core_id match (USB_OTG_CORE_ID_310A).
                return CoreId;
            case Reg.GNPTXSTS:
                // Non-periodic TX FIFO always reports space free so the HAL's
                // FIFO-space checks never stall (refined with real FIFO depth in
                // the transfer phase).
                return 0x0008_0100;
            case Reg.DSTS:
                // Read-only device status: enumerated speed = Full (internal FS
                // PHY), not suspended. Preserve any stored suspend bit.
                return (Get(Reg.DSTS) & 0x1) | DSTS_FS;
            case Reg.DAINT:
                // Read-only: which endpoints have a (masked) pending interrupt.
                return ComputeDaint();
            default:
                return Get((Reg)offset);
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            // Per-endpoint DIEPINTx/DOEPINTx are write-1-to-clear; clearing them
            // can drop DAINT → GINTSTS.IEPINT/OEPINT, so refresh the IRQ.
            if((InRange(offset, DIEP_BASE, DIEP_BASE + NumEndpoints * EP_STRIDE)
                    && (offset - DIEP_BASE) % EP_STRIDE == EP_INT_OFFSET)
                || (InRange(offset, DOEP_BASE, DOEP_BASE + NumEndpoints * EP_STRIDE)
                    && (offset - DOEP_BASE) % EP_STRIDE == EP_INT_OFFSET))
            {
                regs[offset] = Get((Reg)offset) & ~value;
                UpdateIrq();
                return;
            }

            switch((Reg)offset)
            {
            case Reg.GINTSTS:
                // Interrupt status flags are write-1-to-clear (§59.15.3).
                regs[(long)Reg.GINTSTS] = Get(Reg.GINTSTS) & ~value;
                UpdateIrq();
                return;
            case Reg.DCTL:
                regs[offset] = value;
                this.Log(LogLevel.Debug, "OTG: DCTL soft-disconnect = {0}", (value & SDIS) != 0);
                return;
            case Reg.DAINTMSK:
            case Reg.DIEPMSK:
            case Reg.DOEPMSK:
                regs[offset] = value;
                UpdateIrq(); // masks feed the DAINT → GINTSTS aggregation
                return;
            case Reg.GOTGINT:
                regs[(long)Reg.GOTGINT] = Get(Reg.GOTGINT) & ~value; // W1C
                return;
            case Reg.GRSTCTL:
                regs[offset] = value;
                if((value & CSRST) != 0)
                {
                    // Core soft reset returns the core to its default state
                    // (§59.15.2). Later phases also clear device/endpoint state.
                    this.Log(LogLevel.Debug, "OTG: core soft reset (CSRST)");
                }
                // CSRST/RXFFLSH/TXFFLSH are self-clearing (handled on read).
                return;
            case Reg.GAHBCFG:
            case Reg.GINTMSK:
                regs[offset] = value;
                UpdateIrq();
                return;
            default:
                regs[offset] = value;
                return;
            }
        }

        private void UpdateIrq()
        {
            // §59.15: the OTG interrupt line asserts when an unmasked status bit
            // is pending AND the global interrupt is enabled (GAHBCFG.GINTMSK).
            var pending = (ComputeGintsts() & Get(Reg.GINTMSK)) != 0;
            var globalEnabled = (Get(Reg.GAHBCFG) & GINTMSK_GLOBAL) != 0;
            IRQ.Set(pending && globalEnabled);
        }

        private uint ComputeGintsts()
        {
            var gintsts = Get(Reg.GINTSTS) & ~CMOD; // CMOD = 0 (device mode)
            // GINTSTS.IEPINT/OEPINT aggregate the per-endpoint interrupts through
            // DAINT masked by DAINTMSK (§59.15.3 — the interrupt tree).
            var daintMasked = ComputeDaint() & Get(Reg.DAINTMSK);
            if((daintMasked & 0x0000_FFFFu) != 0)
            {
                gintsts |= IEPINT;
            }
            if((daintMasked & 0xFFFF_0000u) != 0)
            {
                gintsts |= OEPINT;
            }
            return gintsts;
        }

        private uint ComputeDaint()
        {
            // DAINT bit x (IN) / bit 16+x (OUT) is set when endpoint x's
            // DIEPINTx / DOEPINTx has a bit set that DIEPMSK / DOEPMSK enables.
            uint daint = 0;
            var diepmsk = Get(Reg.DIEPMSK);
            var doepmsk = Get(Reg.DOEPMSK);
            for(int ep = 0; ep < NumEndpoints; ep++)
            {
                if((GetRaw(DIEP_BASE + ep * EP_STRIDE + EP_INT_OFFSET) & diepmsk) != 0)
                {
                    daint |= 1u << ep;
                }
                if((GetRaw(DOEP_BASE + ep * EP_STRIDE + EP_INT_OFFSET) & doepmsk) != 0)
                {
                    daint |= 1u << (16 + ep);
                }
            }
            return daint;
        }

        private uint GetRaw(long offset) => regs.TryGetValue(offset, out var v) ? v : 0u;

        private uint Get(Reg r) => regs.TryGetValue((long)r, out var v) ? v : 0u;

        private readonly IMachine machine;
        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();

        // Synopsys core ID for the STM32H7 OTG (ST stm32h7xx_ll_usb.h
        // USB_OTG_CORE_ID_310A); synopsys-usb-otg branches on this.
        private const uint CoreId = 0x4F54_310A;

        // GRSTCTL bits (§59.15.2)
        private const uint CSRST = 1u << 0;
        private const uint RXFFLSH = 1u << 4;
        private const uint TXFFLSH = 1u << 5;
        private const uint AHBIDL = 1u << 31;

        // GINTSTS bits (§59.15.3)
        private const uint CMOD = 1u << 0;
        private const uint SOF = 1u << 3;
        private const uint RXFLVL = 1u << 4;
        private const uint USBSUSP = 1u << 11;
        private const uint USBRST = 1u << 12;
        private const uint ENUMDNE = 1u << 13;
        private const uint IEPINT = 1u << 18;
        private const uint OEPINT = 1u << 19;
        private const uint WKUPINT = 1u << 31;

        // GAHBCFG.GINTMSK — global interrupt enable (bit 0, §59.15.1).
        private const uint GINTMSK_GLOBAL = 1u << 0;

        // --- Device registers (§59.15.x, from the OTG device base 0x800) ----
        // DSTS.ENUMSPD = 0b11: Full speed using the internal FS PHY (the Daisy's
        // on-board USB). SUSPSTS = 0. Read-only status.
        private const uint DSTS_FS = 0b11u << 1;
        private const uint SDIS = 1u << 1; // DCTL soft-disconnect

        // Per-endpoint interrupt registers live in 0x900..0x9FF (IN) and
        // 0xB00..0xBFF (OUT), 0x20 apart. Within each endpoint block: CTL +0x00,
        // INT +0x08, TSIZ +0x10, (IN) TXFSTS +0x18.
        private const long DIEP_BASE = 0x900;
        private const long DOEP_BASE = 0xB00;
        private const long EP_STRIDE = 0x20;
        private const long EP_INT_OFFSET = 0x08;
        private const long EP_TXFSTS_OFFSET = 0x18;
        private const int NumEndpoints = 9; // H7 OTG_FS: up to 9 bidirectional EPs

        private static bool InRange(long offset, long lo, long hi) => offset >= lo && offset < hi;

        private enum Reg : long
        {
            GOTGCTL = 0x000,
            GOTGINT = 0x004,
            GAHBCFG = 0x008,
            GUSBCFG = 0x00C,
            GRSTCTL = 0x010,
            GINTSTS = 0x014,
            GINTMSK = 0x018,
            GRXSTSR = 0x01C,
            GRXSTSP = 0x020,
            GRXFSIZ = 0x024,
            DIEPTXF0 = 0x028,
            GNPTXSTS = 0x02C,
            GCCFG = 0x038,
            CID = 0x03C,
            GLPMCFG = 0x054,
            DCFG = 0x800,
            DCTL = 0x804,
            DSTS = 0x808,
            DIEPMSK = 0x810,
            DOEPMSK = 0x814,
            DAINT = 0x818,
            DAINTMSK = 0x81C,
            DIEPEMPMSK = 0x834,
            PCGCCTL = 0xE00,
        }
    }
}
