//! Minimal USB-MIDI 1.0 class (MIDIStreaming — Audio class, subclass 0x03).
//!
//! Presents one bidirectional MIDI port: a bulk OUT endpoint carrying MIDI
//! from the host into an embedded IN jack (routed to an external MIDI OUT),
//! and a bulk IN endpoint carrying MIDI from an external MIDI IN (embedded
//! OUT jack) up to the host. Data is exchanged as 32-bit USB-MIDI Event
//! Packets (USB-MIDI 1.0 §4). Sits alongside the CDC + UAC functions.
//!
//! Status: descriptor-correct per the USB-MIDI 1.0 spec; validate the bulk
//! streaming on hardware / the Renode OTG model.

use usb_device::class_prelude::*;

const USB_CLASS_AUDIO: u8 = 0x01;
const SUBCLASS_MIDISTREAMING: u8 = 0x03;
const PROTOCOL_NONE: u8 = 0x00;

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;

const MS_HEADER: u8 = 0x01;
const MIDI_IN_JACK: u8 = 0x02;
const MIDI_OUT_JACK: u8 = 0x03;
const MS_EP_GENERAL: u8 = 0x01;

const JACK_EMBEDDED: u8 = 0x01;
const JACK_EXTERNAL: u8 = 0x02;

// Jack IDs.
const JID_EMB_IN: u8 = 1; // embedded IN jack (from host, via bulk OUT)
const JID_EXT_IN: u8 = 2; // external IN jack (physical MIDI IN)
const JID_EMB_OUT: u8 = 3; // embedded OUT jack (to host, via bulk IN)
const JID_EXT_OUT: u8 = 4; // external OUT jack (physical MIDI OUT)

const BULK_PACKET_SIZE: u16 = 64;

/// A single bidirectional USB-MIDI port.
pub struct UsbMidiClass<'a, B: UsbBus> {
    iface: InterfaceNumber,
    ep_out: EndpointOut<'a, B>, // host → device MIDI events
    ep_in: EndpointIn<'a, B>,   // device → host MIDI events
}

impl<'a, B: UsbBus> UsbMidiClass<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        Self {
            iface: alloc.interface(),
            ep_out: alloc.bulk(BULK_PACKET_SIZE),
            ep_in: alloc.bulk(BULK_PACKET_SIZE),
        }
    }

    /// Read USB-MIDI event packets sent by the host (4 bytes each).
    pub fn read(&mut self, buf: &mut [u8]) -> usb_device::Result<usize> {
        self.ep_out.read(buf)
    }

    /// Send USB-MIDI event packets to the host (4 bytes each).
    pub fn write(&mut self, buf: &[u8]) -> usb_device::Result<usize> {
        self.ep_in.write(buf)
    }
}

impl<B: UsbBus> UsbClass<B> for UsbMidiClass<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()> {
        // Standard MIDIStreaming interface (bulk endpoints, no alt settings).
        writer.interface(self.iface, USB_CLASS_AUDIO, SUBCLASS_MIDISTREAMING, PROTOCOL_NONE)?;

        // CS MS interface header. wTotalLength = header(7) + 2 IN jacks (6 each)
        // + 2 OUT jacks (9 each) = 37.
        let total: u16 = 37;
        writer.write(CS_INTERFACE, &[MS_HEADER, 0x00, 0x01, total as u8, (total >> 8) as u8])?;

        // Jacks: host → embedded-IN(1) → external-OUT(4); external-IN(2) →
        // embedded-OUT(3) → host.
        writer.write(CS_INTERFACE, &[MIDI_IN_JACK, JACK_EMBEDDED, JID_EMB_IN, 0x00])?;
        writer.write(CS_INTERFACE, &[MIDI_IN_JACK, JACK_EXTERNAL, JID_EXT_IN, 0x00])?;
        writer.write(
            CS_INTERFACE,
            &[MIDI_OUT_JACK, JACK_EMBEDDED, JID_EMB_OUT, 0x01, JID_EXT_IN, 0x01, 0x00],
        )?;
        writer.write(
            CS_INTERFACE,
            &[MIDI_OUT_JACK, JACK_EXTERNAL, JID_EXT_OUT, 0x01, JID_EMB_IN, 0x01, 0x00],
        )?;

        // Bulk OUT endpoint (host → device) + CS MS bulk EP listing embedded IN jack.
        writer.endpoint_ex(&self.ep_out, |extra| {
            extra[0] = 0; // bRefresh
            extra[1] = 0; // bSynchAddress
            Ok(2)
        })?;
        writer.write(CS_ENDPOINT, &[MS_EP_GENERAL, 0x01, JID_EMB_IN])?;

        // Bulk IN endpoint (device → host) + CS MS bulk EP listing embedded OUT jack.
        writer.endpoint_ex(&self.ep_in, |extra| {
            extra[0] = 0;
            extra[1] = 0;
            Ok(2)
        })?;
        writer.write(CS_ENDPOINT, &[MS_EP_GENERAL, 0x01, JID_EMB_OUT])?;

        Ok(())
    }
}
