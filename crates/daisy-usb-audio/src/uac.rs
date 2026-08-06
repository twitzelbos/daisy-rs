//! Minimal USB Audio Class 1.0 (UAC1) implementation: a stereo, 16-bit,
//! 48 kHz device that is BOTH a USB speaker (host → device, playback) and a
//! USB microphone (device → host, capture). Designed to sit alongside a CDC
//! ACM serial function in one composite device (see `main.rs`).
//!
//! Scope / status: the descriptor set below follows the UAC1 spec (USB Audio
//! Device Class 1.0) and enumerates as a full-duplex audio device. It builds
//! and is descriptor-correct; end-to-end streaming (isochronous data flow,
//! alt-setting-gated FIFO, sample-rate feedback) needs validation on hardware
//! or against the Renode OTG model — usb-device 0.3 does not surface
//! SET_INTERFACE alt changes to the class, so the app treats the iso endpoints
//! as always-live. Marked clearly where that matters.
//!
//! Refs: USB Device Class Definition for Audio Devices 1.0; USB 2.0 §9/§5.12.

use usb_device::class_prelude::*;
use usb_device::endpoint::{IsochronousSynchronizationType as Sync, IsochronousUsageType as Usage};
use usb_device::Result;

// --- Audio class constants (UAC1 spec) ---
const USB_CLASS_AUDIO: u8 = 0x01;
const SUBCLASS_AUDIOCONTROL: u8 = 0x01;
const SUBCLASS_AUDIOSTREAMING: u8 = 0x02;
const PROTOCOL_NONE: u8 = 0x00;

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;

// AudioControl interface descriptor subtypes.
const AC_HEADER: u8 = 0x01;
const AC_INPUT_TERMINAL: u8 = 0x02;
const AC_OUTPUT_TERMINAL: u8 = 0x03;
// AudioStreaming interface descriptor subtypes.
const AS_GENERAL: u8 = 0x01;
const AS_FORMAT_TYPE: u8 = 0x02;
// Endpoint descriptor subtype.
const AS_EP_GENERAL: u8 = 0x01;

// Terminal types.
const TT_USB_STREAMING: u16 = 0x0101;
const TT_SPEAKER: u16 = 0x0301;
const TT_MICROPHONE: u16 = 0x0201;

// Terminal IDs.
const TID_USB_IN: u8 = 1; // USB streaming in (host → device, playback source)
const TID_SPEAKER: u8 = 2; // speaker out
const TID_MIC: u8 = 3; // microphone in (capture source)
const TID_USB_OUT: u8 = 4; // USB streaming out (device → host, capture sink)

// Audio format: PCM stereo 16-bit 48 kHz.
const CHANNELS: u8 = 2;
const SUBFRAME_BYTES: u8 = 2; // 16-bit
const BIT_RESOLUTION: u8 = 16;
const SAMPLE_RATE: u32 = 48_000;
/// Bytes per 1 ms USB frame at 48 kHz stereo 16-bit, plus one sample of slop
/// for asynchronous/adaptive rate drift (48 → up to 49 frames).
pub const AUDIO_PACKET_SIZE: u16 = (SAMPLE_RATE as u16 / 1000 + 1) * CHANNELS as u16 * SUBFRAME_BYTES as u16;

// Class-specific audio control requests (USB Audio 1.0 §5.2).
const SET_CUR: u8 = 0x01;
const GET_CUR: u8 = 0x81;
const SAMPLING_FREQ_CONTROL: u8 = 0x01;

/// Full-duplex UAC1 audio class. Owns the two isochronous data endpoints and
/// the three audio interface numbers (1 control + 2 streaming).
pub struct UsbAudioClass<'a, B: UsbBus> {
    control_if: InterfaceNumber,
    stream_out_if: InterfaceNumber, // playback (host → device)
    stream_in_if: InterfaceNumber,  // capture (device → host)
    ep_out: EndpointOut<'a, B>,     // iso OUT: playback samples from host
    ep_in: EndpointIn<'a, B>,       // iso IN: capture samples to host
}

impl<'a, B: UsbBus> UsbAudioClass<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        Self {
            control_if: alloc.interface(),
            stream_out_if: alloc.interface(),
            stream_in_if: alloc.interface(),
            // Playback sink is adaptive to the host clock; capture source is
            // asynchronous (free-running device clock — the codec's).
            ep_out: alloc.isochronous(Sync::Adaptive, Usage::Data, AUDIO_PACKET_SIZE, 1),
            ep_in: alloc.isochronous(Sync::Asynchronous, Usage::Data, AUDIO_PACKET_SIZE, 1),
        }
    }

    /// Read one playback packet the host sent (host → device). Returns the
    /// number of bytes, or `WouldBlock` if none pending.
    pub fn read_playback(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.ep_out.read(buf)
    }

    /// Send one capture packet to the host (device → host).
    pub fn write_capture(&mut self, buf: &[u8]) -> Result<usize> {
        self.ep_in.write(buf)
    }

    fn write_format_type(&self, writer: &mut DescriptorWriter) -> Result<()> {
        // Format Type I (PCM), 2ch, 16-bit, single discrete sample rate 48 kHz.
        let freq = SAMPLE_RATE.to_le_bytes();
        writer.write(
            CS_INTERFACE,
            &[
                AS_FORMAT_TYPE,
                0x01, // bFormatType = FORMAT_TYPE_I
                CHANNELS,
                SUBFRAME_BYTES,
                BIT_RESOLUTION,
                0x01, // bSamFreqType = 1 discrete frequency
                freq[0],
                freq[1],
                freq[2],
            ],
        )
    }
}

impl<B: UsbBus> UsbClass<B> for UsbAudioClass<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        // Group all three audio interfaces under one IAD so the host binds a
        // single audio function even inside a composite device.
        writer.iad(
            self.control_if,
            3,
            USB_CLASS_AUDIO,
            SUBCLASS_AUDIOCONTROL,
            PROTOCOL_NONE,
            None,
        )?;

        // --- AudioControl interface (no endpoints) ---
        writer.interface(self.control_if, USB_CLASS_AUDIO, SUBCLASS_AUDIOCONTROL, PROTOCOL_NONE)?;

        // Class-specific AC interface header. wTotalLength covers the header +
        // all four terminal descriptors that follow (10 + 12 + 9 + 12 + 9 = 52).
        let total: u16 = 52;
        writer.write(
            CS_INTERFACE,
            &[
                AC_HEADER,
                0x00,
                0x01, // bcdADC = 1.00
                total as u8,
                (total >> 8) as u8,
                0x02, // bInCollection = 2 streaming interfaces
                self.stream_out_if.into(),
                self.stream_in_if.into(),
            ],
        )?;

        // Playback path: USB-streaming input terminal → speaker output terminal.
        let usb_in = TT_USB_STREAMING.to_le_bytes();
        writer.write(
            CS_INTERFACE,
            &[
                AC_INPUT_TERMINAL,
                TID_USB_IN,
                usb_in[0],
                usb_in[1],
                0x00, // bAssocTerminal
                CHANNELS,
                0x03,
                0x00, // wChannelConfig = L+R
                0x00, // iChannelNames
                0x00, // iTerminal
            ],
        )?;
        let spk = TT_SPEAKER.to_le_bytes();
        writer.write(
            CS_INTERFACE,
            &[AC_OUTPUT_TERMINAL, TID_SPEAKER, spk[0], spk[1], 0x00, TID_USB_IN, 0x00],
        )?;

        // Capture path: microphone input terminal → USB-streaming output terminal.
        let mic = TT_MICROPHONE.to_le_bytes();
        writer.write(
            CS_INTERFACE,
            &[
                AC_INPUT_TERMINAL,
                TID_MIC,
                mic[0],
                mic[1],
                0x00,
                CHANNELS,
                0x03,
                0x00,
                0x00,
                0x00,
            ],
        )?;
        let usb_out = TT_USB_STREAMING.to_le_bytes();
        writer.write(
            CS_INTERFACE,
            &[AC_OUTPUT_TERMINAL, TID_USB_OUT, usb_out[0], usb_out[1], 0x00, TID_MIC, 0x00],
        )?;

        // --- AudioStreaming OUT (playback): alt 0 idle, alt 1 active ---
        writer.interface_alt(self.stream_out_if, 0, USB_CLASS_AUDIO, SUBCLASS_AUDIOSTREAMING, PROTOCOL_NONE, None)?;
        writer.interface_alt(self.stream_out_if, 1, USB_CLASS_AUDIO, SUBCLASS_AUDIOSTREAMING, PROTOCOL_NONE, None)?;
        // CS AS general: links to the USB-streaming input terminal, PCM format.
        writer.write(CS_INTERFACE, &[AS_GENERAL, TID_USB_IN, 0x00, 0x01, 0x00])?;
        self.write_format_type(writer)?;
        // Iso data endpoint (9-byte audio EP descriptor: +bRefresh +bSynchAddress).
        writer.endpoint_ex(&self.ep_out, |extra| {
            extra[0] = 0; // bRefresh
            extra[1] = 0; // bSynchAddress
            Ok(2)
        })?;
        // CS AS iso audio data endpoint descriptor.
        writer.write(CS_ENDPOINT, &[AS_EP_GENERAL, 0x00, 0x00, 0x00, 0x00])?;

        // --- AudioStreaming IN (capture): alt 0 idle, alt 1 active ---
        writer.interface_alt(self.stream_in_if, 0, USB_CLASS_AUDIO, SUBCLASS_AUDIOSTREAMING, PROTOCOL_NONE, None)?;
        writer.interface_alt(self.stream_in_if, 1, USB_CLASS_AUDIO, SUBCLASS_AUDIOSTREAMING, PROTOCOL_NONE, None)?;
        writer.write(CS_INTERFACE, &[AS_GENERAL, TID_USB_OUT, 0x00, 0x01, 0x00])?;
        self.write_format_type(writer)?;
        writer.endpoint_ex(&self.ep_in, |extra| {
            extra[0] = 0;
            extra[1] = 0;
            Ok(2)
        })?;
        writer.write(CS_ENDPOINT, &[AS_EP_GENERAL, 0x00, 0x00, 0x00, 0x00])?;

        Ok(())
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        // GET_CUR on an endpoint's sampling-frequency control → report 48 kHz.
        if req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Endpoint
            && req.request == GET_CUR
            && (req.value >> 8) as u8 == SAMPLING_FREQ_CONTROL
        {
            let f = SAMPLE_RATE.to_le_bytes();
            let _ = xfer.accept_with(&[f[0], f[1], f[2]]);
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        // SET_CUR of the sampling frequency: we only support 48 kHz, accept it.
        if req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Endpoint
            && req.request == SET_CUR
            && (req.value >> 8) as u8 == SAMPLING_FREQ_CONTROL
        {
            let _ = xfer.accept();
        }
    }
}
