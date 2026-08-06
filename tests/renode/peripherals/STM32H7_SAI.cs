//
// STM32H7_SAI — Renode model for the STM32H7 Serial Audio Interface (SAI1).
//
// RM0433 §51. Copyright (c) 2026 daisy-rs contributors. MIT License.
//
// Why this exists
// ---------------
// The stock STM32H7 platform only TAGS SAI1 (0x40015800) — no model — so the
// SAI → DMAMUX → DMA audio-request chain never fires in sim, and firmware
// double-buffered audio (daisy-audio, following the HAL sai_dma_passthru
// example) can't run. This model implements the register subset a driver
// touches (per sub-block A/B: CR1, CR2, SR, CLRFR, DR) and, crucially, DRIVES
// DMA: while an audio block is enabled with DMAEN set, a LimitTimer pulses the
// block's DMA-request line every frame, exactly like a real codec clocking
// samples. That advances the (extended) DMA over virtual time so
// `emulation RunFor` streams audio.
//
// Data source = internal TX→RX loopback. On this Daisy revision block A is RX
// and block B is TX; each TX write to B's data register is fed straight into
// A's RX FIFO, so a firmware/loopback test sees whatever it plays come back as
// capture. (A real codec is a wire in the sim's eyes.) Request-line numbers
// (RM0433 DMAMUX1 mapping): SAI1_A = 87, SAI1_B = 88.
//
// NOT cycle-accurate: the frame rate is a sim-friendly constant chosen so
// RunFor stays fast — same spirit as the DWT's fixed frequency. It models the
// PROTOCOL (SAIEN/DMAEN gating, DMA request cadence, data movement, FIFO
// level / FREQ status), not real 48 kHz timing.
//
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.Sound
{
    public sealed class STM32H7_SAI : IDoubleWordPeripheral, IKnownSize
    {
        public STM32H7_SAI(IMachine machine)
        {
            DmaRequestA = new GPIO();
            DmaRequestB = new GPIO();
            loopback = new Queue<uint>();
            // Frame pacer: pulses the DMA request lines FrameRateHz times per
            // virtual second while audio is running. Reduced rate keeps RunFor
            // fast; it is a modelling choice, not the real 48 kHz.
            frameTimer = new LimitTimer(machine.ClockSource, FrameRateHz, this, "saiFrame", limit: 1, eventEnabled: true);
            frameTimer.LimitReached += OnFrame;
            Reset();
        }

        public void Reset()
        {
            frameTimer.Enabled = false;
            regs.Clear();
            loopback.Clear();
            saienA = saienB = dmaenA = dmaenB = false;
            DmaRequestA.Unset();
            DmaRequestB.Unset();
        }

        public long Size => 0x400;

        // DMA-request lines wired to the DMAMUX in the platform:
        //   DmaRequestA -> dmamux1@87 (SAI1_A), DmaRequestB -> dmamux1@88 (SAI1_B).
        public GPIO DmaRequestA { get; }
        public GPIO DmaRequestB { get; }

        public uint ReadDoubleWord(long offset)
        {
            switch((Reg)offset)
            {
            case Reg.ASR:
                return StatusOf(blockAIsReceiver: true);
            case Reg.BSR:
                return StatusOf(blockAIsReceiver: false);
            case Reg.ADR:
                // Block A = RX: pop a looped sample (the DMA reads this).
                return loopback.Count > 0 ? loopback.Dequeue() : 0;
            case Reg.BDR:
                // Block B = TX: reading the TX data register is not meaningful.
                return 0;
            default:
                return regs.TryGetValue(offset, out var v) ? v : 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch((Reg)offset)
            {
            case Reg.ACR1:
                regs[offset] = value;
                UpdateBlock(ref saienA, ref dmaenA, value, "A");
                break;
            case Reg.BCR1:
                regs[offset] = value;
                UpdateBlock(ref saienB, ref dmaenB, value, "B");
                break;
            case Reg.ACR2:
            case Reg.BCR2:
                // FFLUSH (bit 3) clears the FIFO.
                if((value & (1u << 3)) != 0)
                {
                    loopback.Clear();
                }
                regs[offset] = value & ~(1u << 3);
                break;
            case Reg.BDR:
                // Block B = TX: DMA writes samples here → loop into A's RX FIFO.
                loopback.Enqueue(value);
                break;
            case Reg.ADR:
                // Block A = RX: writing its data register is a no-op.
                break;
            case Reg.ACLRFR:
            case Reg.BCLRFR:
                // Write-1-to-clear the sticky error flags — nothing latched here.
                break;
            default:
                regs[offset] = value;
                break;
            }
        }

        private void UpdateBlock(ref bool saien, ref bool dmaen, uint cr1, string name)
        {
            var newSaien = (cr1 & (1u << 16)) != 0;
            dmaen = (cr1 & (1u << 17)) != 0;
            if(newSaien != saien)
            {
                this.Log(LogLevel.Debug, "SAI block {0}: SAIEN={1} DMAEN={2}", name, newSaien, dmaen);
            }
            saien = newSaien;
            // Run the frame pacer whenever either block is enabled.
            frameTimer.Enabled = saienA || saienB;
        }

        // One audio frame: pulse TX first (so its sample is available to the RX
        // loopback in the same frame), then RX. Each pulse triggers one DMA
        // item transfer via the DMAMUX.
        private void OnFrame()
        {
            if(saienB && dmaenB)
            {
                DmaRequestB.Blink();
            }
            if(saienA && dmaenA)
            {
                DmaRequestA.Blink();
            }
        }

        // SAI_xSR: FLVL[18:16] (FIFO level) + FREQ (bit 3). For block A (RX) the
        // level reflects the loopback FIFO; block B (TX) reports empty/ready.
        private uint StatusOf(bool blockAIsReceiver)
        {
            uint flvl;
            uint freq;
            if(blockAIsReceiver)
            {
                var count = loopback.Count;
                flvl = count == 0 ? 0u : (count >= 4 ? 5u : 2u); // empty / mid / full-ish
                freq = count > 0 ? 1u : 0u; // RX FIFO not empty → request pending
            }
            else
            {
                flvl = 0; // TX FIFO drains immediately in this model
                freq = 1; // TX always ready for the next sample
            }
            return (flvl << 16) | (freq << 3);
        }

        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();
        private readonly Queue<uint> loopback;
        private readonly LimitTimer frameTimer;

        private bool saienA, saienB, dmaenA, dmaenB;

        private const uint FrameRateHz = 8000;

        // Register offsets from the SAI base (RM0433 §51.6). Block B = A + 0x20.
        private enum Reg : long
        {
            GCR = 0x00,
            ACR1 = 0x04,
            ACR2 = 0x08,
            AFRCR = 0x0C,
            ASLOTR = 0x10,
            AIM = 0x14,
            ASR = 0x18,
            ACLRFR = 0x1C,
            ADR = 0x20,
            BCR1 = 0x24,
            BCR2 = 0x28,
            BFRCR = 0x2C,
            BSLOTR = 0x30,
            BIM = 0x34,
            BSR = 0x38,
            BCLRFR = 0x3C,
            BDR = 0x40,
        }
    }
}
