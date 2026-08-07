//! Standalone firmware that exercises the OTG model's **isochronous** data path
//! — the endpoints UAC audio streams over.
//!
//! It builds a minimal class with two iso endpoints (EP1 OUT playback, EP1 IN
//! capture), the same shape daisy-usb-audio's UAC uses, and in its poll loop
//! loops each playback packet straight back to capture — exactly the no-`codec`
//! branch of the real app. The companion `otg_iso.robot` enumerates the device,
//! injects an iso OUT audio frame (a deterministic byte ramp) through the
//! model's stimulus hook, and checks the firmware received it and the same
//! bytes came back out on iso IN (captured by the model). This drives real
//! `usb-device` isochronous endpoint code — read()/write() over the RxFIFO and
//! TX-FIFO paths — plus the SOF frame cadence.

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;

use bsp::hal::pac;
use bsp::hal::prelude::*;
use bsp::hal::time::Hertz;
use bsp::hal::usb_hs::{UsbBus, USB2};
use daisy_bsp as bsp;

use usb_device::class_prelude::*;
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbDeviceState, UsbVidPid};
use usb_device::endpoint::{IsochronousSynchronizationType as Sync, IsochronousUsageType as Usage};
use usb_device::Result as UsbResult;

// One 48 kHz stereo 16-bit frame (48 samples × 2ch × 2 bytes) + a sample slop.
const PACKET_SIZE: u16 = 196;

// DTCM markers the Renode test reads.
const MARK_STATE: *mut u32 = 0x2001_0000 as *mut u32; // UsbDeviceState
const MARK_RXCOUNT: *mut u32 = 0x2001_0004 as *mut u32; // iso OUT packets read
const MARK_RXBYTES: *mut u32 = 0x2001_0008 as *mut u32; // last packet length
const MARK_FIRST: *mut u32 = 0x2001_000C as *mut u32; // first two bytes of it

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

/// Minimal class owning two isochronous data endpoints, mirroring the UAC's
/// iso OUT (playback) + iso IN (capture) shape.
struct IsoLoop<'a, B: usb_device::bus::UsbBus> {
    iface: InterfaceNumber,
    ep_out: EndpointOut<'a, B>,
    ep_in: EndpointIn<'a, B>,
}

impl<'a, B: usb_device::bus::UsbBus> IsoLoop<'a, B> {
    fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        Self {
            iface: alloc.interface(),
            ep_out: alloc.isochronous(Sync::Adaptive, Usage::Data, PACKET_SIZE, 1),
            ep_in: alloc.isochronous(Sync::Asynchronous, Usage::Data, PACKET_SIZE, 1),
        }
    }
}

impl<'a, B: usb_device::bus::UsbBus> UsbClass<B> for IsoLoop<'a, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> UsbResult<()> {
        // Vendor-specific interface carrying the two iso endpoints — enough for
        // the host to configure the device; the sim host does not parse it.
        writer.interface(self.iface, 0xFF, 0x00, 0x00)?;
        writer.endpoint(&self.ep_out)?;
        writer.endpoint(&self.ep_in)?;
        Ok(())
    }
}

fn state_code(s: UsbDeviceState) -> u32 {
    match s {
        UsbDeviceState::Default => 0,
        UsbDeviceState::Addressed => 1,
        UsbDeviceState::Configured => 2,
        UsbDeviceState::Suspend => 3,
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let ccdr = bsp::clocks::init(dp.PWR, dp.RCC, &dp.SYSCFG);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let _dm = gpioa.pa11.into_alternate::<10>();
    let _dp = gpioa.pa12.into_alternate::<10>();

    unsafe {
        let pwr = &*pac::PWR::ptr();
        pwr.cr3.modify(|_, w| w.usb33den().set_bit());
        while pwr.cr3.read().usb33rdy().bit_is_clear() {}
    }

    let usb = USB2 {
        usb_global: dp.OTG2_HS_GLOBAL,
        usb_device: dp.OTG2_HS_DEVICE,
        usb_pwrclk: dp.OTG2_HS_PWRCLK,
        prec: ccdr.peripheral.USB2OTG,
        hclk: Hertz::from_raw(200_000_000),
    };
    let usb_bus = UsbBus::new(usb, unsafe { &mut *core::ptr::addr_of_mut!(EP_MEMORY) });

    let mut class = IsoLoop::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0002))
        .strings(&[StringDescriptors::default().product("daisy usb-iso")])
        .expect("string descriptors")
        .max_packet_size_0(64)
        .expect("ep0 size")
        .build();

    let mut rx_count: u32 = 0;
    let mut buf = [0u8; 256];
    loop {
        let _ = usb_dev.poll(&mut [&mut class]);

        // Loop one playback frame (iso OUT) straight back to capture (iso IN),
        // exactly like the no-codec branch of daisy-usb-audio.
        if let Ok(n) = class.ep_out.read(&mut buf) {
            rx_count = rx_count.wrapping_add(1);
            let first = (buf[0] as u32) | ((buf[1] as u32) << 8);
            let _ = class.ep_in.write(&buf[..n]);
            unsafe {
                core::ptr::write_volatile(MARK_RXCOUNT, rx_count);
                core::ptr::write_volatile(MARK_RXBYTES, n as u32);
                core::ptr::write_volatile(MARK_FIRST, first);
            }
        }

        unsafe { core::ptr::write_volatile(MARK_STATE, state_code(usb_dev.state())) };
    }
}
