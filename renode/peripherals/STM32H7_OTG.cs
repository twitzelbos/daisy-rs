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
// UsbBus init completes, and device enumeration and transfers can be exercised.
//
// PHASE 1 — global core block (§59.15.1). Reset (CSRST + RX/TX FIFO flush,
// AHB-idle), the GINTSTS/GINTMSK/GAHBCFG interrupt mechanism (with the NVIC
// line and a RaiseEvent stimulus hook), current-mode (CMOD = device), core ID,
// and FIFO-size/config registers (stored).
//
// PHASE 2 — device register block (§59.15.x). Device address (DCFG.DAD),
// soft-disconnect (DCTL.SDIS), enumerated-speed status (DSTS.ENUMSPD), and the
// endpoint interrupt tree DIEPINTx/DOEPINTx → DAINT → GINTSTS.IEPINT/OEPINT.
//
// PHASE 3 — the packet path (§59.15.6 device receive + §transmit). The device
// RxFIFO status-pop protocol: a received SETUP/OUT packet pushes a status word
// (readable via GRXSTSR peek / GRXSTSP pop) plus its data (read word-by-word
// from the DFIFO at 0x1000), and asserts GINTSTS.RXFLVL until drained — exactly
// what synopsys-usb-otg's poll() walks (GRXSTSR → GRXSTSP → fill_from_fifo). The
// IN path: arming an endpoint (DIEPCTL.EPENA) and writing the packet to its TX
// FIFO completes the transfer (DIEPINTx.XFRC), captured for inspection. Host
// stimulus hooks (ReceiveSetup / ReceiveOut) inject packets as the bus would.
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
            rxStatusQueue.Clear();
            rxDataQueue.Clear();
            inExpected.Clear();
            inWritten.Clear();
            inCapture.Clear();
            UpdateIrq();
        }

        // Test/stimulus hook: raise OTG interrupt-status bits, exactly as a
        // hardware event (bus reset, enum-done, …) would. The host-stimulus
        // enumeration test drives USBRST/ENUMDNE through this.
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

        // Host stimulus: deliver an 8-byte SETUP packet to EP0 (the 8 bytes are
        // packed little-endian into two words). Pushes the RxFIFO framing a real
        // SETUP transaction produces: a SETUP-data-received status (PKTSTS=0x06,
        // BCNT=8) + the data, then a SETUP-complete status (PKTSTS=0x04).
        public void ReceiveSetup(uint low, uint high)
        {
            var data = new byte[8];
            for(int i = 0; i < 4; i++)
            {
                data[i] = (byte)(low >> (8 * i));
                data[4 + i] = (byte)(high >> (8 * i));
            }
            EnqueueRx(0, PKTSTS_SETUP_DATA, data, 8);
        }

        // Host stimulus: deliver an OUT data packet (0..8 bytes) to `endpoint`,
        // e.g. a control status-stage ZLP (length 0). Pushes an OUT-received
        // status (PKTSTS=0x02) + data, then an OUT-complete status (PKTSTS=0x03).
        public void ReceiveOut(uint endpoint, uint low, uint high, uint length)
        {
            var data = new byte[length];
            for(uint i = 0; i < length; i++)
            {
                var w = i < 4 ? low : high;
                data[i] = (byte)(w >> (int)(8 * (i % 4)));
            }
            EnqueueRx(endpoint, PKTSTS_OUT_DATA, data, length);
        }

        // Inspect the last IN packet the device wrote to an endpoint's TX FIFO
        // (so a test can verify, e.g., the device descriptor it sent).
        public uint InPacketLength(uint endpoint)
            => inCapture.TryGetValue(endpoint, out var l) ? (uint)l.Count : 0u;

        public uint InPacketByte(uint endpoint, uint index)
            => inCapture.TryGetValue(endpoint, out var l) && index < l.Count ? l[(int)index] : 0u;

        public uint ReadDoubleWord(long offset)
        {
            // DFIFO region (0x1000+): reading any endpoint FIFO pops one word off
            // the single device RxFIFO (synopsys reads fifo(0) = 0x1000).
            if(offset >= FIFO_BASE)
            {
                return ReadRxFifo();
            }

            // Per-endpoint IN TX-FIFO-status always reports space free.
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
            case Reg.GRXSTSR:
                // Peek the top RxFIFO status word without popping it (§59.15.6).
                return rxStatusQueue.Count > 0 ? rxStatusQueue.Peek() : 0u;
            case Reg.GRXSTSP:
                // Pop the top RxFIFO status word; RXFLVL deasserts when drained.
                if(rxStatusQueue.Count == 0)
                {
                    return 0u;
                }
                var popped = rxStatusQueue.Dequeue();
                UpdateIrq();
                return popped;
            case Reg.CID:
                // Synopsys core ID — must land in the H7 family arm of
                // synopsys-usb-otg's core_id match (USB_OTG_CORE_ID_310A).
                return CoreId;
            case Reg.GNPTXSTS:
                // Non-periodic TX FIFO always reports space free so the HAL's
                // FIFO-space checks never stall.
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
            // DFIFO region (0x1000+): writing endpoint N's TX FIFO pushes IN data
            // toward the host; a full packet completes the transfer (XFRC).
            if(offset >= FIFO_BASE)
            {
                WriteTxFifo((uint)((offset - FIFO_BASE) / FIFO_STRIDE), value);
                return;
            }

            // Arming an IN endpoint: DIEPCTL.EPENA starts the transfer whose size
            // was programmed in DIEPTSIZ.XFRSIZ. A zero-length packet completes at
            // once (no FIFO write follows); otherwise it completes once the data
            // words land in the TX FIFO.
            if(InRange(offset, DIEP_BASE, DIEP_BASE + NumEndpoints * EP_STRIDE)
                && (offset - DIEP_BASE) % EP_STRIDE == EP_CTL_OFFSET)
            {
                regs[offset] = value;
                if((value & EPENA) != 0)
                {
                    ArmInEndpoint((uint)((offset - DIEP_BASE) / EP_STRIDE));
                }
                return;
            }

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
                // Interrupt status flags are write-1-to-clear (§59.15.3). RXFLVL
                // is read-only (reflects the FIFO) and ignores the write.
                regs[(long)Reg.GINTSTS] = Get(Reg.GINTSTS) & ~(value & ~RXFLVL);
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
                    // (§59.15.2), including the FIFOs and endpoint accounting.
                    this.Log(LogLevel.Debug, "OTG: core soft reset (CSRST)");
                    rxStatusQueue.Clear();
                    rxDataQueue.Clear();
                    inExpected.Clear();
                    inWritten.Clear();
                    inCapture.Clear();
                }
                if((value & RXFFLSH) != 0)
                {
                    rxStatusQueue.Clear();
                    rxDataQueue.Clear();
                }
                // CSRST/RXFFLSH/TXFFLSH are self-clearing (handled on read).
                UpdateIrq();
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

        // --- RX FIFO: device receive status-pop protocol (§59.15.6) ---------

        private void EnqueueRx(uint ep, uint pktsts, byte[] data, uint byteCount)
        {
            // Status word for the data packet, then its bytes padded to a word
            // boundary (fill_from_fifo reads ceil(BCNT/4) words), then the
            // transaction-complete status word — the sequence a real transfer
            // leaves in the FIFO.
            rxStatusQueue.Enqueue(PackRxStatus(ep, byteCount, pktsts));
            foreach(var b in data)
            {
                rxDataQueue.Enqueue(b);
            }
            for(uint pad = (4 - (byteCount & 3)) & 3; pad > 0; pad--)
            {
                rxDataQueue.Enqueue(0);
            }
            var done = pktsts == PKTSTS_SETUP_DATA ? PKTSTS_SETUP_DONE : PKTSTS_OUT_DONE;
            rxStatusQueue.Enqueue(PackRxStatus(ep, 0, done));
            UpdateIrq();
        }

        // GRXSTSP/GRXSTSR layout (§device): EPNUM[3:0], BCNT[14:4], DPID[16:15],
        // PKTSTS[20:17]. DPID is left 0 (DATA0); the driver ignores it.
        private static uint PackRxStatus(uint ep, uint bcnt, uint pktsts)
            => (ep & 0xF) | ((bcnt & 0x7FF) << 4) | ((pktsts & 0xF) << 17);

        private uint ReadRxFifo()
        {
            uint w = 0;
            for(int i = 0; i < 4; i++)
            {
                if(rxDataQueue.Count > 0)
                {
                    w |= (uint)rxDataQueue.Dequeue() << (8 * i);
                }
            }
            return w;
        }

        // --- IN endpoint transmit path --------------------------------------

        private void ArmInEndpoint(uint ep)
        {
            var xfrsiz = (int)(GetRaw(DIEP_BASE + ep * EP_STRIDE + EP_TSIZ_OFFSET) & 0x7FFFF);
            inExpected[ep] = xfrsiz;
            inWritten[ep] = 0;
            inCapture[ep] = new List<byte>();
            if(xfrsiz == 0)
            {
                CompleteIn(ep); // zero-length packet: completes with no FIFO write
            }
        }

        private void WriteTxFifo(uint ep, uint word)
        {
            if(!inCapture.TryGetValue(ep, out var buf))
            {
                buf = new List<byte>();
                inCapture[ep] = buf;
            }
            var expected = inExpected.TryGetValue(ep, out var e) ? e : 0;
            for(int i = 0; i < 4 && buf.Count < expected; i++)
            {
                buf.Add((byte)(word >> (8 * i)));
            }
            inWritten[ep] = inWritten.TryGetValue(ep, out var w) ? w + 4 : 4;
            if(inWritten[ep] >= expected)
            {
                CompleteIn(ep);
            }
        }

        private void CompleteIn(uint ep)
        {
            // Transfer-complete: raise DIEPINTx.XFRC, and mirror the hardware
            // side effects — DIEPTSIZ.PKTCNT decremented to 0, EPENA cleared.
            var intAddr = DIEP_BASE + ep * EP_STRIDE + EP_INT_OFFSET;
            regs[intAddr] = GetRaw(intAddr) | XFRC;
            var tsizAddr = DIEP_BASE + ep * EP_STRIDE + EP_TSIZ_OFFSET;
            regs[tsizAddr] = GetRaw(tsizAddr) & ~PKTCNT_MASK;
            var ctlAddr = DIEP_BASE + ep * EP_STRIDE + EP_CTL_OFFSET;
            regs[ctlAddr] = GetRaw(ctlAddr) & ~EPENA;
            UpdateIrq();
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
            // CMOD = 0 (device); RXFLVL reflects the RxFIFO (both read-only here).
            var gintsts = Get(Reg.GINTSTS) & ~CMOD & ~RXFLVL;
            if(rxStatusQueue.Count > 0)
            {
                gintsts |= RXFLVL;
            }
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

        // RxFIFO (device receive): the queue of status words + the byte stream
        // read back through the DFIFO. Per-IN-endpoint transfer accounting +
        // captured IN data.
        private readonly Queue<uint> rxStatusQueue = new Queue<uint>();
        private readonly Queue<byte> rxDataQueue = new Queue<byte>();
        private readonly Dictionary<uint, int> inExpected = new Dictionary<uint, int>();
        private readonly Dictionary<uint, int> inWritten = new Dictionary<uint, int>();
        private readonly Dictionary<uint, List<byte>> inCapture = new Dictionary<uint, List<byte>>();

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

        // Per-endpoint registers live in 0x900..0x9FF (IN) and 0xB00..0xBFF
        // (OUT), 0x20 apart. Within each endpoint block: CTL +0x00, INT +0x08,
        // TSIZ +0x10, (IN) TXFSTS +0x18.
        private const long DIEP_BASE = 0x900;
        private const long DOEP_BASE = 0xB00;
        private const long EP_STRIDE = 0x20;
        private const long EP_CTL_OFFSET = 0x00;
        private const long EP_INT_OFFSET = 0x08;
        private const long EP_TSIZ_OFFSET = 0x10;
        private const long EP_TXFSTS_OFFSET = 0x18;
        private const int NumEndpoints = 9; // H7 OTG_FS: up to 9 bidirectional EPs

        private const uint EPENA = 1u << 31;      // DIEPCTL enable
        private const uint XFRC = 1u << 0;        // DIEPINT/DOEPINT transfer complete
        private const uint PKTCNT_MASK = 0x1FF8_0000u; // DIEPTSIZ.PKTCNT [28:19]

        // DFIFO push/pop windows: channel N at 0x1000 + N*0x1000 (§59.15).
        private const long FIFO_BASE = 0x1000;
        private const long FIFO_STRIDE = 0x1000;

        // RxFIFO PKTSTS values (device mode, GRXSTSP §59.15.6).
        private const uint PKTSTS_OUT_DATA = 0x02;   // OUT data packet received
        private const uint PKTSTS_OUT_DONE = 0x03;   // OUT transfer completed
        private const uint PKTSTS_SETUP_DONE = 0x04; // SETUP transaction completed
        private const uint PKTSTS_SETUP_DATA = 0x06; // SETUP data packet received

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
