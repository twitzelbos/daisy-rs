#![no_std]

//! USB-MIDI 1.0 packetization: turn a raw **serial MIDI byte stream** (the kind a
//! DIN/TRS MIDI-IN delivers over a 31250-baud UART) into 32-bit **USB-MIDI Event
//! Packets** (USB Device Class Definition for MIDI Devices 1.0, §4), which are
//! what a USB-MIDI class's bulk IN endpoint carries to the host.
//!
//! Feed the UART bytes one at a time to [`UsbMidiEncoder::push`]; each byte yields
//! at most one event packet (`[(cable << 4) | CIN, status, data1, data2]`). The
//! encoder handles the awkward parts of a real MIDI wire:
//!
//! - **Running status** — channel-voice data bytes with no fresh status reuse the
//!   last channel status.
//! - **Interleaved System Real-Time** (`0xF8..=0xFF`) — a clock/start/stop byte can
//!   arrive in the *middle* of another message; it is emitted immediately as its
//!   own single-byte packet without disturbing the message in progress.
//! - **System Common** (`0xF1`/`0xF2`/`0xF3`/`0xF6`).
//! - **SysEx** (`0xF0 … 0xF7`) — accumulated into `CIN 0x4` continuation chunks and
//!   an `0x5`/`0x6`/`0x7` end chunk by the number of trailing bytes.
//!
//! `no_std`, no alloc, no heap — a tiny fixed state machine.
//!
//! ```
//! use daisy_midi::UsbMidiEncoder;
//! let mut enc = UsbMidiEncoder::new(0);
//! // Note-on, channel 1, note 60, velocity 64 (0x90 0x3C 0x40):
//! assert_eq!(enc.push(0x90), None);
//! assert_eq!(enc.push(0x3C), None);
//! assert_eq!(enc.push(0x40), Some([0x09, 0x90, 0x3C, 0x40]));
//! ```

// The library is `no_std`; the host test harness pulls in `std` for `Vec`.
#[cfg(test)]
extern crate std;

#[cfg(feature = "usb")]
mod class;
#[cfg(feature = "usb")]
pub use class::UsbMidiClass;

/// A 32-bit USB-MIDI Event Packet: `[(cable << 4) | CIN, status, data1, data2]`.
pub type EventPacket = [u8; 4];

/// Serial-MIDI → USB-MIDI event-packet encoder for one virtual cable.
#[derive(Debug, Clone)]
pub struct UsbMidiEncoder {
    cable: u8,
    status: u8, // active channel-voice / system-common status; 0 = none
    data: [u8; 2],
    count: u8,    // data bytes collected for the current message
    expected: u8, // data bytes the current message needs
    sysex: bool,
    sx: [u8; 3], // current SysEx chunk accumulator
    sxn: u8,     // bytes in the current SysEx chunk (0..=3)
}

/// Data bytes a channel-voice status carries: 1 for Program Change (`0xC`) and
/// Channel Pressure (`0xD`), 2 for everything else.
fn channel_data_len(status: u8) -> u8 {
    match status >> 4 {
        0xC | 0xD => 1,
        _ => 2,
    }
}

impl UsbMidiEncoder {
    /// New encoder tagging every packet with `cable` (the USB-MIDI cable number,
    /// 0..=15 — one virtual port per cable).
    #[must_use]
    pub const fn new(cable: u8) -> Self {
        Self {
            cable: cable & 0x0F,
            status: 0,
            data: [0; 2],
            count: 0,
            expected: 0,
            sysex: false,
            sx: [0; 3],
            sxn: 0,
        }
    }

    #[inline]
    fn hdr(&self, cin: u8) -> u8 {
        (self.cable << 4) | (cin & 0x0F)
    }

    /// Feed one serial MIDI byte. Returns a complete event packet if this byte
    /// finished a message (or is a real-time byte), else `None`.
    pub fn push(&mut self, b: u8) -> Option<EventPacket> {
        // System Real-Time: single byte, emitted at once, does NOT disturb a
        // message (running status or SysEx) in progress.
        if b >= 0xF8 {
            return Some([self.hdr(0x0F), b, 0, 0]);
        }

        if self.sysex {
            return self.push_sysex(b);
        }

        if b >= 0x80 {
            self.push_status(b)
        } else {
            self.push_data(b)
        }
    }

    fn push_status(&mut self, b: u8) -> Option<EventPacket> {
        match b {
            0xF0 => {
                // SysEx start.
                self.status = 0;
                self.sysex = true;
                self.sx = [0xF0, 0, 0];
                self.sxn = 1;
                None
            }
            0xF7 => None, // stray End-of-Exclusive outside SysEx: ignore.
            0xF1 | 0xF3 => {
                // System Common, one data byte.
                self.status = b;
                self.expected = 1;
                self.count = 0;
                None
            }
            0xF2 => {
                // Song Position, two data bytes.
                self.status = b;
                self.expected = 2;
                self.count = 0;
                None
            }
            0xF6 => {
                // Tune Request: single-byte System Common.
                self.status = 0;
                Some([self.hdr(0x05), 0xF6, 0, 0])
            }
            0xF4 | 0xF5 => {
                // Undefined System Common: drop, and clear running status.
                self.status = 0;
                None
            }
            _ => {
                // Channel-voice status (0x80..=0xEF).
                self.status = b;
                self.expected = channel_data_len(b);
                self.count = 0;
                None
            }
        }
    }

    fn push_data(&mut self, b: u8) -> Option<EventPacket> {
        if self.status == 0 {
            return None; // orphan data byte (no running status): drop.
        }
        self.data[self.count as usize] = b;
        self.count += 1;
        if self.count < self.expected {
            return None;
        }
        self.count = 0;

        let status = self.status;
        let d1 = self.data[0];
        let d2 = if self.expected == 2 { self.data[1] } else { 0 };

        let cin = if status >= 0xF0 {
            // System Common: does not run — clear the active status.
            self.status = 0;
            match status {
                0xF2 => 0x03, // three-byte
                _ => 0x02,    // two-byte (0xF1, 0xF3)
            }
        } else {
            status >> 4 // channel voice — running status persists
        };
        Some([self.hdr(cin), status, d1, d2])
    }

    fn push_sysex(&mut self, b: u8) -> Option<EventPacket> {
        // A non-real-time status byte inside SysEx implicitly ends it (some
        // devices omit the F7); drop the partial chunk and reprocess the byte.
        if b >= 0x80 && b != 0xF7 {
            self.sysex = false;
            self.sxn = 0;
            return self.push(b);
        }

        self.sx[self.sxn as usize] = b;
        self.sxn += 1;

        if b == 0xF7 {
            // End: CIN by the number of bytes in the final chunk.
            let cin = match self.sxn {
                1 => 0x05,
                2 => 0x06,
                _ => 0x07,
            };
            let p = [self.hdr(cin), self.sx[0], self.sx[1], self.sx[2]];
            self.reset_sysex();
            return Some(p);
        }
        if self.sxn == 3 {
            // Full continuation chunk.
            let p = [self.hdr(0x04), self.sx[0], self.sx[1], self.sx[2]];
            self.sx = [0; 3];
            self.sxn = 0;
            return Some(p);
        }
        None
    }

    fn reset_sysex(&mut self) {
        self.sysex = false;
        self.sx = [0; 3];
        self.sxn = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    // Feed a byte slice; collect every emitted packet.
    fn run(enc: &mut UsbMidiEncoder, bytes: &[u8]) -> Vec<EventPacket> {
        bytes.iter().filter_map(|&b| enc.push(b)).collect()
    }

    #[test]
    fn note_on_off() {
        let mut e = UsbMidiEncoder::new(0);
        assert_eq!(run(&mut e, &[0x90, 0x3C, 0x40]), [[0x09, 0x90, 0x3C, 0x40]]);
        assert_eq!(run(&mut e, &[0x80, 0x3C, 0x40]), [[0x08, 0x80, 0x3C, 0x40]]);
    }

    #[test]
    fn running_status() {
        let mut e = UsbMidiEncoder::new(0);
        // 0x90 then two note pairs with no repeated status.
        let out = run(&mut e, &[0x90, 0x3C, 0x40, 0x3E, 0x50]);
        assert_eq!(out, [[0x09, 0x90, 0x3C, 0x40], [0x09, 0x90, 0x3E, 0x50]]);
    }

    #[test]
    fn control_change_and_pitchbend() {
        let mut e = UsbMidiEncoder::new(0);
        assert_eq!(run(&mut e, &[0xB0, 0x07, 0x7F]), [[0x0B, 0xB0, 0x07, 0x7F]]);
        assert_eq!(run(&mut e, &[0xE0, 0x00, 0x40]), [[0x0E, 0xE0, 0x00, 0x40]]);
    }

    #[test]
    fn one_data_byte_messages() {
        let mut e = UsbMidiEncoder::new(0);
        // Program change (0xC) and channel pressure (0xD): one data byte, d2 = 0.
        assert_eq!(run(&mut e, &[0xC0, 0x05]), [[0x0C, 0xC0, 0x05, 0x00]]);
        assert_eq!(run(&mut e, &[0xD0, 0x40]), [[0x0D, 0xD0, 0x40, 0x00]]);
    }

    #[test]
    fn realtime_interleaved_mid_message() {
        let mut e = UsbMidiEncoder::new(0);
        // Clock (0xF8) arrives between the note number and velocity.
        let out = run(&mut e, &[0x90, 0x3C, 0xF8, 0x40]);
        assert_eq!(out, [[0x0F, 0xF8, 0x00, 0x00], [0x09, 0x90, 0x3C, 0x40]]);
    }

    #[test]
    fn realtime_standalone() {
        let mut e = UsbMidiEncoder::new(0);
        for b in [0xF8u8, 0xFA, 0xFB, 0xFC, 0xFE, 0xFF] {
            assert_eq!(e.push(b), Some([0x0F, b, 0x00, 0x00]));
        }
    }

    #[test]
    fn system_common() {
        let mut e = UsbMidiEncoder::new(0);
        assert_eq!(run(&mut e, &[0xF1, 0x7F]), [[0x02, 0xF1, 0x7F, 0x00]]); // MTC
        assert_eq!(run(&mut e, &[0xF2, 0x00, 0x40]), [[0x03, 0xF2, 0x00, 0x40]]); // song pos
        assert_eq!(run(&mut e, &[0xF3, 0x05]), [[0x02, 0xF3, 0x05, 0x00]]); // song select
        assert_eq!(e.push(0xF6), Some([0x05, 0xF6, 0x00, 0x00])); // tune request
    }

    #[test]
    fn system_common_clears_running_status() {
        let mut e = UsbMidiEncoder::new(0);
        run(&mut e, &[0x90, 0x3C, 0x40]); // set channel running status
        run(&mut e, &[0xF3, 0x05]); // system common
                                    // A bare data byte now has no running status → dropped.
        assert_eq!(e.push(0x3E), None);
    }

    #[test]
    fn sysex_three_then_three() {
        let mut e = UsbMidiEncoder::new(0);
        // F0 7E 7F 09 01 F7 → one full chunk + a 3-byte end chunk.
        let out = run(&mut e, &[0xF0, 0x7E, 0x7F, 0x09, 0x01, 0xF7]);
        assert_eq!(out, [[0x04, 0xF0, 0x7E, 0x7F], [0x07, 0x09, 0x01, 0xF7]]);
    }

    #[test]
    fn sysex_end_lengths() {
        // Empty SysEx F0 F7 → ends with 2 bytes (CIN 0x6).
        let mut e = UsbMidiEncoder::new(0);
        assert_eq!(run(&mut e, &[0xF0, 0xF7]), [[0x06, 0xF0, 0xF7, 0x00]]);
        // F0 41 F7 → single 3-byte end chunk (CIN 0x7).
        let mut e = UsbMidiEncoder::new(0);
        assert_eq!(run(&mut e, &[0xF0, 0x41, 0xF7]), [[0x07, 0xF0, 0x41, 0xF7]]);
        // F0 7E 7F 41 F7 → full chunk then a 2-byte end (CIN 0x6).
        let mut e = UsbMidiEncoder::new(0);
        assert_eq!(
            run(&mut e, &[0xF0, 0x7E, 0x7F, 0x41, 0xF7]),
            [[0x04, 0xF0, 0x7E, 0x7F], [0x06, 0x41, 0xF7, 0x00]]
        );
        // ...one more data byte → a 1-byte end (CIN 0x5).
        let mut e = UsbMidiEncoder::new(0);
        assert_eq!(
            run(&mut e, &[0xF0, 0x7E, 0x7F, 0x41, 0x42, 0xF7]),
            [[0x04, 0xF0, 0x7E, 0x7F], [0x07, 0x41, 0x42, 0xF7]]
        );
    }

    #[test]
    fn realtime_during_sysex() {
        let mut e = UsbMidiEncoder::new(0);
        // Clock lands between SysEx bytes: emitted immediately, SysEx unbroken.
        let out = run(&mut e, &[0xF0, 0x7E, 0xF8, 0x7F, 0x09, 0xF7]);
        assert_eq!(
            out,
            [
                [0x0F, 0xF8, 0x00, 0x00],
                [0x04, 0xF0, 0x7E, 0x7F],
                [0x06, 0x09, 0xF7, 0x00],
            ]
        );
    }

    #[test]
    fn status_byte_ends_sysex_implicitly() {
        let mut e = UsbMidiEncoder::new(0);
        // A note-on interrupts an unterminated SysEx: partial chunk dropped, the
        // note is parsed normally.
        let out = run(&mut e, &[0xF0, 0x12, 0x90, 0x3C, 0x40]);
        assert_eq!(out, [[0x09, 0x90, 0x3C, 0x40]]);
    }

    #[test]
    fn orphan_data_dropped() {
        let mut e = UsbMidiEncoder::new(0);
        assert_eq!(e.push(0x3C), None); // no status yet
        assert_eq!(e.push(0x40), None);
    }

    #[test]
    fn cable_number_in_header() {
        let mut e = UsbMidiEncoder::new(3);
        // Cable 3 → high nibble 0x3 in every header byte.
        assert_eq!(run(&mut e, &[0x90, 0x3C, 0x40]), [[0x39, 0x90, 0x3C, 0x40]]);
        assert_eq!(e.push(0xF8), Some([0x3F, 0xF8, 0x00, 0x00]));
    }
}
