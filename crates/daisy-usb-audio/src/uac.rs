//! Minimal USB Audio Class 1.0 (UAC1) implementation: a stereo, 16-bit,
//! 48 kHz device that is BOTH a USB speaker (host → device, playback) and a
//! USB microphone (device → host, capture). Designed to sit alongside a CDC
//! ACM serial function in one composite device (see `main.rs`).
//!
//! The codec is the sample-clock master, so both streams are **asynchronous**
//! and the host rate-matches to us: the playback (OUT) path is paired with an
//! **explicit feedback endpoint** ([`write_feedback`](UsbAudioClass::write_feedback),
//! a 3-byte 10.14 Ff, its address wired into the data endpoint's `bSynchAddress`)
//! that reports our true sample rate each frame; the capture (IN) path is steered
//! by varying the packet size around [`NOMINAL_CAPTURE_BYTES`].
//!
//! Scope / status: the descriptor set follows the UAC1 spec (USB Audio Device
//! Class 1.0) and enumerates as a full-duplex async audio device with alt 0
//! (idle) / alt 1 (active) on both streaming interfaces. Alt-setting gating is
//! wired ([`UsbClass::set_alt_setting`]/[`UsbClass::get_alt_setting`], which
//! usb-device 0.3.2 forwards; [`playback_active`](UsbAudioClass::playback_active)/
//! [`capture_active`](UsbAudioClass::capture_active) gate the iso endpoints). The
//! feedback *plumbing* (endpoint enumerates + carries an Ff per frame) is proven
//! in sim; the feedback *value* — its control loop against real clock drift — is
//! hardware-tuned, as is FIFO under-/overrun behaviour. Marked where it matters.
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
pub const AUDIO_PACKET_SIZE: u16 =
    (SAMPLE_RATE as u16 / 1000 + 1) * CHANNELS as u16 * SUBFRAME_BYTES as u16;

/// Bytes per stereo audio sample — the granularity every packet size is a
/// multiple of, and the ± step for async IN sizing.
pub const FRAME_BYTES: usize = CHANNELS as usize * SUBFRAME_BYTES as usize;

/// Bytes in one nominal (48-sample) capture packet — the centre value the async
/// IN packet size is nudged around (± one [`FRAME_BYTES`]) to track the codec clock.
#[allow(dead_code)] // used only in the codec build (async IN sizing)
pub const NOMINAL_CAPTURE_BYTES: usize = (SAMPLE_RATE as usize / 1000) * FRAME_BYTES;

/// Nominal explicit-feedback value (USB 2.0 §5.12.4.2): samples per 1 ms frame in
/// full-speed 10.14 fixed point. 48.0 samples/frame = `48 << 14`. The app nudges
/// this ± to steer the host's send rate against the playback ring's fill level.
pub const NOMINAL_FEEDBACK_Q10_14: u32 = 48 << 14;

// Class-specific audio control requests (USB Audio 1.0 §5.2).
const SET_CUR: u8 = 0x01;
const GET_CUR: u8 = 0x81;
const GET_MIN: u8 = 0x82;
const GET_MAX: u8 = 0x83;
const GET_RES: u8 = 0x84;
const SAMPLING_FREQ_CONTROL: u8 = 0x01;

// A Feature Unit (master mute + volume) on each path: speaker (playback) and
// mic (capture). Applied in our DSP — the codec has no volume registers.
const AC_FEATURE_UNIT: u8 = 0x06;
const FU_SPEAKER_ID: u8 = 5; // playback unit ID — terminals use 1..=4
const FU_MIC_ID: u8 = 6; // capture unit ID
const MUTE_CONTROL: u8 = 0x01;
const VOLUME_CONTROL: u8 = 0x02;
// Volume range in UAC 1/256 dB fixed point. 0x8000 is the spec's "silence".
const VOL_MIN: i16 = -60 * 256; // -60 dB
const VOL_MAX: i16 = 0; // 0 dB
const VOL_RES: i16 = 256; // 1 dB step
const VOL_SILENCE: i16 = i16::MIN; // 0x8000

/// Full-duplex UAC1 audio class. Owns the two isochronous data endpoints and
/// the three audio interface numbers (1 control + 2 streaming).
pub struct UsbAudioClass<'a, B: UsbBus> {
    control_if: InterfaceNumber,
    stream_out_if: InterfaceNumber, // playback (host → device)
    stream_in_if: InterfaceNumber,  // capture (device → host)
    ep_out: EndpointOut<'a, B>,     // iso OUT: playback samples from host
    ep_fb: EndpointIn<'a, B>,       // iso IN: explicit feedback for the OUT path
    ep_in: EndpointIn<'a, B>,       // iso IN: capture samples to host
    // Current alternate setting of each streaming interface (0 = idle/zero
    // bandwidth, 1 = streaming). The host toggles these with SET_INTERFACE to
    // start/stop each direction; they reset to 0 on bus reset / SET_CONFIGURATION.
    alt_out: u8,
    alt_in: u8,
    // Feature Unit state (master mute + volume, 1/256 dB CUR) for each path. The
    // host sets these via SET_CUR; the app reads [`gain`](Self::gain) /
    // [`capture_gain`](Self::capture_gain) and applies them in the DSP — the codec
    // has no volume registers, so all level control lives here.
    spk_mute: bool,
    spk_volume: i16,
    mic_mute: bool,
    mic_volume: i16,
}

impl<'a, B: UsbBus> UsbAudioClass<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        Self {
            control_if: alloc.interface(),
            stream_out_if: alloc.interface(),
            stream_in_if: alloc.interface(),
            // Both directions run off the device (codec) clock, so both are
            // Asynchronous: the codec is the sample-clock master and the host
            // rate-matches. Playback (OUT) is paired with an explicit feedback
            // endpoint (below) that reports our true sample rate each frame;
            // capture (IN) is adjusted by varying the packet size instead.
            ep_out: alloc.isochronous(Sync::Asynchronous, Usage::Data, AUDIO_PACKET_SIZE, 1),
            ep_fb: alloc.isochronous(Sync::NoSynchronization, Usage::Feedback, 3, 1),
            ep_in: alloc.isochronous(Sync::Asynchronous, Usage::Data, AUDIO_PACKET_SIZE, 1),
            alt_out: 0,
            alt_in: 0,
            spk_mute: false,
            spk_volume: VOL_MAX, // 0 dB
            mic_mute: false,
            mic_volume: VOL_MAX,
        }
    }

    /// Linear playback (speaker) gain from its Feature Unit — multiply into the
    /// samples bound for the DAC.
    #[must_use]
    pub fn gain(&self) -> f32 {
        Self::gain_of(self.spk_mute, self.spk_volume)
    }

    /// Linear capture (mic) gain from its Feature Unit — multiply into the samples
    /// sent to the host.
    #[must_use]
    pub fn capture_gain(&self) -> f32 {
        Self::gain_of(self.mic_mute, self.mic_volume)
    }

    /// mute + volume (1/256 dB) → linear multiplier. `0.0` when muted or at the
    /// silence sentinel; otherwise `10^(dB/20)` with `dB = volume / 256`.
    fn gain_of(mute: bool, volume: i16) -> f32 {
        if mute || volume == VOL_SILENCE {
            return 0.0;
        }
        libm::powf(10.0, (volume as f32 / 256.0) / 20.0)
    }

    /// If `req` is a class request addressed to one of our Feature Units, return
    /// `(entity_id, control_selector)`; else `None`.
    fn fu_selector(&self, req: &control::Request) -> Option<(u8, u8)> {
        let entity = (req.index >> 8) as u8;
        if req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && (entity == FU_SPEAKER_ID || entity == FU_MIC_ID)
            && (req.index & 0xff) as u8 == u8::from(self.control_if)
        {
            Some((entity, (req.value >> 8) as u8))
        } else {
            None
        }
    }

    /// Whether the host has selected the streaming alt setting for **playback**
    /// (host → device). Only read [`read_playback`](Self::read_playback) while true.
    #[must_use]
    pub fn playback_active(&self) -> bool {
        self.alt_out == 1
    }

    /// Whether the host has selected the streaming alt setting for **capture**
    /// (device → host). Only call [`write_capture`](Self::write_capture) while true —
    /// stuffing the iso IN FIFO on the idle alt setting wastes cycles on packets
    /// the host isn't scheduling.
    #[must_use]
    pub fn capture_active(&self) -> bool {
        self.alt_in == 1
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

    /// Publish the explicit-feedback value for the playback (OUT) path: the host
    /// reads this iso IN endpoint to decide how many samples to send next frame.
    /// `samples_q10_14` is samples/frame in FS 10.14 fixed point (see
    /// [`NOMINAL_FEEDBACK_Q10_14`]). Call once per frame while
    /// [`playback_active`](Self::playback_active).
    pub fn write_feedback(&mut self, samples_q10_14: u32) -> Result<usize> {
        let b = samples_q10_14.to_le_bytes();
        self.ep_fb.write(&b[..3])
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
        writer.interface(
            self.control_if,
            USB_CLASS_AUDIO,
            SUBCLASS_AUDIOCONTROL,
            PROTOCOL_NONE,
        )?;

        // Class-specific AC interface header. wTotalLength covers the header, the
        // four terminals and the two Feature Units:
        // 10 + 12 + 10 + 9 + 12 + 10 + 9 = 72.
        let total: u16 = 72;
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
        // Feature Unit on the playback path (USB-in → speaker): master mute +
        // volume. bmaControls[0] = master (bit0 Mute, bit1 Volume); the two
        // channels carry no per-channel controls.
        writer.write(
            CS_INTERFACE,
            &[
                AC_FEATURE_UNIT,
                FU_SPEAKER_ID,
                TID_USB_IN, // bSourceID
                0x01,       // bControlSize = 1 byte per control bitmap
                0x03,       // bmaControls[0] master: Mute | Volume
                0x00,       // bmaControls[1] left
                0x00,       // bmaControls[2] right
                0x00,       // iFeature
            ],
        )?;
        let spk = TT_SPEAKER.to_le_bytes();
        writer.write(
            CS_INTERFACE,
            &[
                AC_OUTPUT_TERMINAL,
                TID_SPEAKER,
                spk[0],
                spk[1],
                0x00,
                FU_SPEAKER_ID, // bSourceID ← Feature Unit (was USB-in directly)
                0x00,
            ],
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
        // Feature Unit on the capture path (mic → USB-out): master mute + volume.
        writer.write(
            CS_INTERFACE,
            &[
                AC_FEATURE_UNIT,
                FU_MIC_ID,
                TID_MIC, // bSourceID
                0x01,    // bControlSize
                0x03,    // bmaControls[0] master: Mute | Volume
                0x00,    // bmaControls[1] left
                0x00,    // bmaControls[2] right
                0x00,    // iFeature
            ],
        )?;
        let usb_out = TT_USB_STREAMING.to_le_bytes();
        writer.write(
            CS_INTERFACE,
            &[
                AC_OUTPUT_TERMINAL,
                TID_USB_OUT,
                usb_out[0],
                usb_out[1],
                0x00,
                FU_MIC_ID, // bSourceID ← Feature Unit (was mic directly)
                0x00,
            ],
        )?;

        // --- AudioStreaming OUT (playback): alt 0 idle, alt 1 active ---
        writer.interface_alt(
            self.stream_out_if,
            0,
            USB_CLASS_AUDIO,
            SUBCLASS_AUDIOSTREAMING,
            PROTOCOL_NONE,
            None,
        )?;
        writer.interface_alt(
            self.stream_out_if,
            1,
            USB_CLASS_AUDIO,
            SUBCLASS_AUDIOSTREAMING,
            PROTOCOL_NONE,
            None,
        )?;
        // CS AS general: links to the USB-streaming input terminal, PCM format.
        writer.write(CS_INTERFACE, &[AS_GENERAL, TID_USB_IN, 0x00, 0x01, 0x00])?;
        self.write_format_type(writer)?;
        // Iso data OUT endpoint (9-byte audio EP descriptor). bSynchAddress points
        // the host at the feedback endpoint below, where we report our rate.
        let fb_addr = u8::from(self.ep_fb.address());
        writer.endpoint_ex(&self.ep_out, |extra| {
            extra[0] = 0; // bRefresh
            extra[1] = fb_addr; // bSynchAddress → feedback endpoint
            Ok(2)
        })?;
        // CS AS iso audio data endpoint descriptor.
        writer.write(CS_ENDPOINT, &[AS_EP_GENERAL, 0x00, 0x00, 0x00, 0x00])?;
        // Explicit synchronization feedback endpoint: iso IN carrying the 3-byte
        // 10.14 Ff value the host reads to rate-match its OUT stream to our codec
        // clock. bRefresh = feedback update rate exponent (no CS endpoint descriptor).
        writer.endpoint_ex(&self.ep_fb, |extra| {
            extra[0] = 5; // bRefresh
            extra[1] = 0; // bSynchAddress (none)
            Ok(2)
        })?;

        // --- AudioStreaming IN (capture): alt 0 idle, alt 1 active ---
        writer.interface_alt(
            self.stream_in_if,
            0,
            USB_CLASS_AUDIO,
            SUBCLASS_AUDIOSTREAMING,
            PROTOCOL_NONE,
            None,
        )?;
        writer.interface_alt(
            self.stream_in_if,
            1,
            USB_CLASS_AUDIO,
            SUBCLASS_AUDIOSTREAMING,
            PROTOCOL_NONE,
            None,
        )?;
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
        let req = *xfer.request();
        // GET_CUR on an endpoint's sampling-frequency control → report 48 kHz.
        if req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Endpoint
            && req.request == GET_CUR
            && (req.value >> 8) as u8 == SAMPLING_FREQ_CONTROL
        {
            let f = SAMPLE_RATE.to_le_bytes();
            let _ = xfer.accept_with(&[f[0], f[1], f[2]]);
            return;
        }
        // Feature Unit reads (host querying speaker/mic volume/mute).
        if let Some((entity, selector)) = self.fu_selector(&req) {
            let (mute, volume) = if entity == FU_MIC_ID {
                (self.mic_mute, self.mic_volume)
            } else {
                (self.spk_mute, self.spk_volume)
            };
            match (req.request, selector) {
                (GET_CUR, MUTE_CONTROL) => {
                    let _ = xfer.accept_with(&[mute as u8]);
                }
                (GET_CUR, VOLUME_CONTROL) => {
                    let _ = xfer.accept_with(&volume.to_le_bytes());
                }
                (GET_MIN, VOLUME_CONTROL) => {
                    let _ = xfer.accept_with(&VOL_MIN.to_le_bytes());
                }
                (GET_MAX, VOLUME_CONTROL) => {
                    let _ = xfer.accept_with(&VOL_MAX.to_le_bytes());
                }
                (GET_RES, VOLUME_CONTROL) => {
                    let _ = xfer.accept_with(&VOL_RES.to_le_bytes());
                }
                _ => {}
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = *xfer.request();
        // SET_CUR of the sampling frequency: we only support 48 kHz, accept it.
        if req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Endpoint
            && req.request == SET_CUR
            && (req.value >> 8) as u8 == SAMPLING_FREQ_CONTROL
        {
            let _ = xfer.accept();
            return;
        }
        // Feature Unit writes (host setting speaker/mic volume/mute).
        if req.request == SET_CUR {
            if let Some((entity, selector)) = self.fu_selector(&req) {
                let mic = entity == FU_MIC_ID;
                let data = xfer.data();
                match selector {
                    MUTE_CONTROL if !data.is_empty() => {
                        let m = data[0] != 0;
                        if mic {
                            self.mic_mute = m;
                        } else {
                            self.spk_mute = m;
                        }
                        let _ = xfer.accept();
                    }
                    VOLUME_CONTROL if data.len() >= 2 => {
                        let v = i16::from_le_bytes([data[0], data[1]]);
                        if mic {
                            self.mic_volume = v;
                        } else {
                            self.spk_volume = v;
                        }
                        let _ = xfer.accept();
                    }
                    _ => {}
                }
            }
        }
    }

    /// Bus reset / SET_CONFIGURATION returns every interface to its default
    /// (idle) alt setting, so both streams stop until the host re-selects alt 1.
    fn reset(&mut self) {
        self.alt_out = 0;
        self.alt_in = 0;
    }

    /// Report the current alt setting for one of our streaming interfaces so
    /// GET_INTERFACE answers correctly; `None` for interfaces we don't own.
    fn get_alt_setting(&mut self, interface: InterfaceNumber) -> Option<u8> {
        let iface = u8::from(interface);
        if iface == u8::from(self.stream_out_if) {
            Some(self.alt_out)
        } else if iface == u8::from(self.stream_in_if) {
            Some(self.alt_in)
        } else {
            None
        }
    }

    /// Accept SET_INTERFACE for our two streaming interfaces (alt 0 = idle,
    /// alt 1 = streaming) and latch the new state; returning `true` is what makes
    /// usb-device accept the transfer instead of STALLing a non-zero alt. Only
    /// alt 0/1 are defined, and other interfaces aren't ours → `false` (the
    /// device's default handling then accepts alt 0 / rejects the rest).
    fn set_alt_setting(&mut self, interface: InterfaceNumber, alternative: u8) -> bool {
        if alternative > 1 {
            return false;
        }
        let iface = u8::from(interface);
        if iface == u8::from(self.stream_out_if) {
            self.alt_out = alternative;
            true
        } else if iface == u8::from(self.stream_in_if) {
            self.alt_in = alternative;
            true
        } else {
            false
        }
    }
}
