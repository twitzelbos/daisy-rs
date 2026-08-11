//! Standalone firmware that runs the real `usb-device` CDC stack and exposes
//! its enumeration state, so the `STM32H7_OTG` Renode model can be driven
//! through a full USB enumeration and verified end-to-end.
//!
//! USB bring-up is identical to `usb-init-exerciser` (clocks::init → freeze-free
//! USB2 → UsbBus::new → CDC device). Then it polls forever, recording the
//! device's `UsbDeviceState` into a DTCM marker every iteration. The companion
//! `otg_enum.robot` injects the host side of enumeration through the model's
//! stimulus hooks (bus reset + speed-enum, then SET_ADDRESS and
//! SET_CONFIGURATION SETUP packets) and asserts the firmware's real stack walks
//! Default → Addressed → Configured, with the assigned address landing in DCFG.

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;

use bsp::hal::pac;
use bsp::hal::prelude::*;
use bsp::hal::time::Hertz;
use bsp::hal::usb_hs::{UsbBus, USB2};
use daisy_bsp as bsp;

use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbDeviceState, UsbVidPid};
use usbd_serial::SerialPort;

// Markers the Renode test reads back — a real `.bss` static (linker-placed clear
// of the stack), resolved in the robot with `sysbus GetSymbolAddress "MARKERS"`.
#[no_mangle]
static mut MARKERS: [u32; 3] = [0; 3];
const M_STATE: usize = 0; // current UsbDeviceState
const M_POLLS: usize = 1; // poll() iteration count
const M_MAXSTATE: usize = 2; // highest state reached

#[inline]
fn mark(slot: usize, v: u32) {
    // SAFETY: single-context volatile writes to distinct slots of a fixed static.
    unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!(MARKERS).cast::<u32>().add(slot), v)
    }
}

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

// UsbDeviceState → a stable numeric code for the marker.
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

    // On-board micro-USB pins PA11/PA12 → AF10 (OTG2 FS).
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let _dm = gpioa.pa11.into_alternate::<10>();
    let _dp = gpioa.pa12.into_alternate::<10>();

    // External 3.3 V transceiver supply (modelled by the PWR stub).
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

    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .strings(&[StringDescriptors::default().product("daisy usb-enum")])
        .expect("string descriptors")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .max_packet_size_0(64)
        .expect("ep0 size")
        .build();

    let mut polls: u32 = 0;
    let mut max_state: u32 = 0;
    loop {
        let _ = usb_dev.poll(&mut [&mut serial]);

        let code = state_code(usb_dev.state());
        if code > max_state {
            max_state = code;
        }
        polls = polls.wrapping_add(1);
        mark(M_STATE, code);
        mark(M_POLLS, polls);
        mark(M_MAXSTATE, max_state);
    }
}
