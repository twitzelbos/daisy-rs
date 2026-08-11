//! Bridge between the USB UAC audio endpoints and the on-board codec
//! (`daisy-audio`, SAI1 + DMA), making the Daisy a real USB audio interface:
//! host playback (UAC iso OUT) → codec DAC, codec ADC → host capture (UAC iso
//! IN). Two lock-free SPSC rings decouple the USB poll loop (main context)
//! from the audio callback (DMA interrupt).
//!
//! # HARDWARE-INTEGRATION REQUIRED (not runnable in Renode; validate on a board)
//! The clock side is now DONE: the bootloader configures **PLL3_P = 49.152 MHz**
//! (`daisy_bsp::clocks::init`) and stashes the frozen **`CoreClocks`** into
//! Backup SRAM; [`recover_clocks`] hands it back so the HAL SAI + I2C drivers
//! (which need `&CoreClocks`) can be constructed in this XIP app. What still
//! needs a board to finish + validate:
//!   1. Wire `daisy_audio::Audio::new(.., &recover_clocks().unwrap())` and set
//!      SAI1's kernel mux to PLL3P — the register/DMA path is real but untested.
//!   2. **WM8731 pins** confirmed for this board revision — Seed 1.1 wires the
//!      codec over I2C2 (SCL=PH4, SDA=PB11) with no reset line, which differs
//!      from `daisy-audio`'s AK4556-derived `Pins` (PB11 as reset). Reconcile
//!      before trusting audio.
//!   3. **Sample-rate sync** (UAC async feedback) between the USB 1 ms frame
//!      and the codec's free-running 48 kHz — the rings tolerate small drift
//!      but a feedback endpoint is needed for long-run stability.

// `audio_process` and `CLOCKS_HANDOFF` are the entry points the on-hardware
// integrator wires up (register the callback with daisy-audio, read the
// handed-off CoreClocks); they are intentionally not called from the
// codec-agnostic USB bridge that is complete here.
#![allow(dead_code)]

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicUsize, Ordering};

/// Recover the bootloader's frozen `CoreClocks` (stashed in Backup SRAM by
/// `daisy_bsp::clocks::handoff::stash`). Required to construct the HAL SAI +
/// I2C drivers (`daisy_audio::Audio::new`, `I2c::new`) — an XIP app cannot mint
/// its own `CoreClocks`. Returns `None` if the bootloader didn't leave a valid,
/// version-matching hand-off.
pub fn recover_clocks() -> Option<daisy_bsp::hal::rcc::CoreClocks> {
    // SAFETY: reads Backup SRAM; sound because this app and the bootloader are
    // built from the same workspace (identical stm32h7xx-hal), and `restore`
    // guards the layout with a sys_ck check.
    unsafe { daisy_bsp::clocks::handoff::restore() }
}

/// A single-producer / single-consumer f32 ring. Producer and consumer may run
/// in different contexts (main loop vs DMA IRQ) without locks.
pub struct SpscRing<const N: usize> {
    buf: UnsafeCell<[f32; N]>,
    head: AtomicUsize, // next read
    tail: AtomicUsize, // next write
}

// SAFETY: head/tail are atomic; each cell is written by the producer before
// tail is published (Release) and read by the consumer after observing tail
// (Acquire). Exactly one producer and one consumer.
unsafe impl<const N: usize> Sync for SpscRing<N> {}

impl<const N: usize> SpscRing<N> {
    pub const fn new() -> Self {
        Self {
            buf: UnsafeCell::new([0.0; N]),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    pub fn push(&self, value: f32) -> bool {
        let tail = self.tail.load(Ordering::Relaxed);
        let next = (tail + 1) % N;
        if next == self.head.load(Ordering::Acquire) {
            return false; // full
        }
        unsafe {
            (*self.buf.get())[tail] = value;
        }
        self.tail.store(next, Ordering::Release);
        true
    }

    pub fn pop(&self) -> Option<f32> {
        let head = self.head.load(Ordering::Relaxed);
        if head == self.tail.load(Ordering::Acquire) {
            return None; // empty
        }
        let value = unsafe { (*self.buf.get())[head] };
        self.head.store((head + 1) % N, Ordering::Release);
        Some(value)
    }

    /// Samples currently queued. A snapshot across the two contexts, so it is an
    /// estimate — good enough to steer a rate-matching loop, not for exact flow.
    pub fn len(&self) -> usize {
        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Acquire);
        (tail + N - head) % N
    }
}

const RING_LEN: usize = 512;
/// Usable ring capacity (one slot is reserved to tell full from empty). The
/// rate-matching loops steer each ring's fill toward `RING_CAPACITY / 2`.
pub const RING_CAPACITY: usize = RING_LEN - 1;
/// Host playback samples awaiting the codec DAC (USB OUT → audio callback).
pub static PLAYBACK: SpscRing<RING_LEN> = SpscRing::new();
/// Codec ADC samples awaiting the host (audio callback → USB IN).
pub static CAPTURE: SpscRing<RING_LEN> = SpscRing::new();

/// Samples queued in the playback ring (host → codec) — drives the OUT feedback.
pub fn playback_len() -> usize {
    PLAYBACK.len()
}

/// Samples queued in the capture ring (codec → host) — drives async IN sizing.
pub fn capture_len() -> usize {
    CAPTURE.len()
}

/// The `daisy-audio` processing callback (runs in the DMA interrupt): play out
/// what the host sent, and hand the host what the codec captured. Underruns
/// output silence; capture overruns drop (the USB side is the flow limiter).
pub fn audio_process(
    input: &[f32; daisy_audio::BLOCK_SIZE * 2],
    output: &mut [f32; daisy_audio::BLOCK_SIZE * 2],
) {
    for i in 0..(daisy_audio::BLOCK_SIZE * 2) {
        output[i] = PLAYBACK.pop().unwrap_or(0.0);
        let _ = CAPTURE.push(input[i]);
    }
}

/// Convert an interleaved little-endian i16 stereo USB packet into the
/// playback ring (host → codec).
pub fn push_playback_bytes(bytes: &[u8]) {
    for frame in bytes.chunks_exact(2) {
        let s = i16::from_le_bytes([frame[0], frame[1]]) as f32 / 32768.0;
        let _ = PLAYBACK.push(s);
    }
}

/// Drain up to `out.len()/2` capture samples into an interleaved i16 packet
/// (codec → host). Returns the number of bytes written.
pub fn pop_capture_bytes(out: &mut [u8]) -> usize {
    let mut n = 0;
    while n + 1 < out.len() {
        match CAPTURE.pop() {
            Some(s) => {
                let v = (s.clamp(-1.0, 1.0) * 32767.0) as i16;
                let b = v.to_le_bytes();
                out[n] = b[0];
                out[n + 1] = b[1];
                n += 2;
            }
            None => break,
        }
    }
    n
}
