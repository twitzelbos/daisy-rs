//! Standalone firmware that runs the real HAL USB device bring-up, so the
//! `STM32H7_OTG` Renode model is validated against the actual
//! `synopsys-usb-otg` core-init — the code that spins forever on GRSTCTL /
//! GINTSTS when the OTG registers are unmodelled.
//!
//! It mirrors daisy-usb-audio's freeze-free USB path: `clocks::init`, then build
//! the all-`pub`-field `USB2` directly, `UsbBus::new` (which runs the core
//! reset + PHY handshake), build a CDC `SerialPort` device, and poll it. A DTCM
//! marker advances at each step and a poll counter increments in the loop, so
//! the Renode test can prove init completed (no host required) and `poll()`
//! keeps running.

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;

use bsp::hal::pac;
use bsp::hal::prelude::*;
use bsp::hal::time::Hertz;
use bsp::hal::usb_hs::{UsbBus, USB2};
use daisy_bsp as bsp;

use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;

const MARK_STAGE: *mut u32 = 0x2001_0000 as *mut u32;
const MARK_POLLS: *mut u32 = 0x2001_0004 as *mut u32;

// OTG2 (FS mode) has 4 KiB of dedicated FIFO RAM; EP_MEMORY in DTCM.
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[inline(always)]
fn mark(stage: u32) {
    unsafe { core::ptr::write_volatile(MARK_STAGE, stage) };
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    mark(0x10); // entered

    let ccdr = bsp::clocks::init(dp.PWR, dp.RCC, &dp.SYSCFG);
    mark(0x11); // clocks::init done

    // On-board micro-USB pins PA11/PA12 → AF10 (OTG2 FS).
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let _dm = gpioa.pa11.into_alternate::<10>();
    let _dp = gpioa.pa12.into_alternate::<10>();
    mark(0x12); // USB pins

    // External 3.3 V transceiver supply (modelled by the PWR stub).
    unsafe {
        let pwr = &*pac::PWR::ptr();
        pwr.cr3.modify(|_, w| w.usb33den().set_bit());
        while pwr.cr3.read().usb33rdy().bit_is_clear() {}
    }
    mark(0x13); // USB33 ready

    // Freeze-free USB2 (all fields pub). UsbBus::new runs the DWC_OTG core reset
    // + PHY handshake — the polls that hang without the OTG model.
    let usb = USB2 {
        usb_global: dp.OTG2_HS_GLOBAL,
        usb_device: dp.OTG2_HS_DEVICE,
        usb_pwrclk: dp.OTG2_HS_PWRCLK,
        prec: ccdr.peripheral.USB2OTG,
        hclk: Hertz::from_raw(200_000_000),
    };
    let usb_bus = UsbBus::new(usb, unsafe { &mut *core::ptr::addr_of_mut!(EP_MEMORY) });
    mark(0x14); // *** UsbBus::new completed — core init did not hang ***

    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .strings(&[StringDescriptors::default().product("daisy usb-init")])
        .expect("string descriptors")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .max_packet_size_0(64)
        .expect("ep0 size")
        .build();
    mark(0x15); // USB device built (endpoints allocated + configured)

    // Poll the device. With no host attached, poll() returns cleanly each time
    // (no enumeration events) and must not hang.
    let mut polls: u32 = 0;
    loop {
        let _ = usb_dev.poll(&mut [&mut serial]);
        polls = polls.wrapping_add(1);
        unsafe { core::ptr::write_volatile(MARK_POLLS, polls) };
        if polls == 100 {
            mark(0x16); // polled 100× without hanging
        }
    }
}
