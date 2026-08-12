//! Standalone **Daisy Pod USB-MIDI input** interface: reads the Pod's hardware
//! DIN/TRS MIDI-IN (USART1 RX = PB7 / Seed D14, 31250 baud 8N1, input-only) and
//! forwards it to the host as a class-compliant USB-MIDI device — plug it into a
//! computer and the Pod's MIDI-IN jack appears as a MIDI input port, no driver.
//!
//! Runs from internal flash, so its own [`clocks::init`](daisy_bsp::clocks::init)
//! mints real `CoreClocks` for both USB and the UART — none of the XIP app's
//! freeze-free dance. Reuses [`daisy_midi`] for the USB-MIDI class and the
//! serial→USB-MIDI packetizer (running status / real-time / SysEx). HARDWARE-ONLY.

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;

use bsp::hal::pac;
use bsp::hal::prelude::*;
use bsp::hal::time::Hertz;
use bsp::hal::usb_hs::{UsbBus, USB2};
use daisy_bsp as bsp;

use daisy_midi::{UsbMidiClass, UsbMidiEncoder};
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};

// OTG2 (FS mode) has 4 KiB of dedicated FIFO RAM; EP_MEMORY in DTCM (never cached).
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

// pid.codes VID; PID distinct from the composite (DA15) and Hothouse (DA16).
const VID: u16 = 0x1209;
const PID: u16 = 0xDA17;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let ccdr = bsp::clocks::init(dp.PWR, dp.RCC, &dp.SYSCFG);

    // USB on the on-board micro-USB (PA11/PA12 → AF10). The bootloader isn't in
    // the picture here; clocks::init leaves HSI48 + the USB kernel mux configured.
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

    let mut midi = UsbMidiClass::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(VID, PID))
        .strings(&[StringDescriptors::default()
            .manufacturer("daisy-rs")
            .product("Daisy Pod MIDI")])
        .expect("string descriptors")
        .max_packet_size_0(64)
        .expect("ep0 size")
        .build();

    // The Pod's MIDI-IN: USART1 RX = PB7, 31250 baud 8N1, RX-only (input-only, so
    // NoTx — this also leaves PB6/the encoder click free; see daisy_bsp::pod).
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let rx_pin = gpiob.pb7.into_alternate::<7>();
    let serial1 = dp
        .USART1
        .serial(
            (bsp::hal::serial::NoTx, rx_pin),
            31_250.bps(),
            ccdr.peripheral.USART1,
            &ccdr.clocks,
        )
        .expect("USART1 MIDI serial");
    let (_tx, mut rx) = serial1.split();
    let mut enc = UsbMidiEncoder::new(0);

    let mut buf = [0u8; 64];
    loop {
        // Forward each byte of hardware DIN MIDI-IN to the host over USB-MIDI.
        while let Ok(b) = rx.read() {
            if let Some(pkt) = enc.push(b) {
                let _ = midi.write(&pkt);
            }
        }
        // Keep the device serviced; drain any host→device MIDI (unused, no MIDI-OUT).
        if usb_dev.poll(&mut [&mut midi]) {
            let _ = midi.read(&mut buf);
        }
    }
}
