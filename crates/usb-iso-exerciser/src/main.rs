//! Standalone firmware that exercises the OTG model's isochronous data path
//! **serviced from the OTG_FS interrupt** — the interrupt-driven servicing model
//! the real USB-audio app uses, so `poll()` runs on every USB event instead of
//! being starved whenever a busy main loop does other work.
//!
//! A minimal class owns two iso data endpoints (EP1 OUT playback + EP1 IN
//! capture) plus an explicit-feedback endpoint (EP2 IN) under one interface with
//! alt 0 (idle) / alt 1 (streaming) — daisy-usb-audio's UAC shape. The device +
//! class live in a shared `Mutex<RefCell<..>>`; the `OTG_FS` handler polls the
//! stack and, while the host has selected alt 1, loops each playback packet
//! straight back to capture and publishes the nominal feedback value. **The main
//! loop only `wfi`s — it never polls** — so the enumeration, the SET_INTERFACE
//! handling, the iso loopback and the feedback below all prove the interrupt path
//! services USB end to end.
//!
//! `otg_iso.robot` drives it: enumerate, SET_INTERFACE(alt 1), inject an iso OUT
//! ramp, check it looped back on iso IN, then SET_INTERFACE(alt 0) and check the
//! stream goes idle. Exercises real `usb-device` iso read()/write() over the
//! Rx/Tx FIFOs, the SOF cadence, the SET_INTERFACE→set_alt_setting gate, and the
//! GINTSTS→NVIC(OTG_FS) delivery path (RM0433 §59.15).

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::mem::MaybeUninit;

use panic_halt as _;

use cortex_m::asm::wfi;
use cortex_m::interrupt::{free as interrupt_free, Mutex};
use cortex_m_rt::entry;

use bsp::hal::pac::{self, interrupt};
use bsp::hal::prelude::*;
use bsp::hal::time::Hertz;
use bsp::hal::usb_hs::{UsbBus, USB2};
use daisy_bsp as bsp;

use usb_device::class_prelude::*;
use usb_device::device::{
    StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid,
};
use usb_device::endpoint::{IsochronousSynchronizationType as Sync, IsochronousUsageType as Usage};
use usb_device::Result as UsbResult;

type Bus = UsbBus<USB2>;

// One 48 kHz stereo 16-bit frame (48 samples × 2ch × 2 bytes) + a sample slop.
const PACKET_SIZE: u16 = 196;

// DTCM markers the Renode test reads.
const MARK_STATE: *mut u32 = 0x2001_0000 as *mut u32; // UsbDeviceState
const MARK_RXCOUNT: *mut u32 = 0x2001_0004 as *mut u32; // iso OUT packets read
const MARK_RXBYTES: *mut u32 = 0x2001_0008 as *mut u32; // last packet length
const MARK_FIRST: *mut u32 = 0x2001_000C as *mut u32; // first two bytes of it
const MARK_ALT: *mut u32 = 0x2001_0010 as *mut u32; // streaming interface alt setting
const MARK_ISR: *mut u32 = 0x2001_0014 as *mut u32; // OTG_FS interrupt invocations
const MARK_MUTE: *mut u32 = 0x2001_0018 as *mut u32; // speaker FU mute state
const MARK_VOL: *mut u32 = 0x2001_001C as *mut u32; // speaker FU volume (u16 zero-ext)

static mut EP_MEMORY: [u32; 1024] = [0; 1024];
// The bus allocator must outlive the device + class (which borrow it) so they can
// live in the shared static the interrupt reaches — hence 'static, set up once.
static mut USB_ALLOC: MaybeUninit<UsbBusAllocator<Bus>> = MaybeUninit::uninit();

// Nominal explicit-feedback value: 48.0 samples/frame in FS 10.14 fixed point.
const NOMINAL_FEEDBACK_Q10_14: u32 = 48 << 14;

// Feature Unit control round-trip (mirrors daisy-usb-audio's speaker + mic
// volume/mute — two independent entities, to prove they route separately).
const SET_CUR: u8 = 0x01;
const GET_CUR: u8 = 0x81;
const FU_SPEAKER_ID: u8 = 5;
const FU_MIC_ID: u8 = 6;
const MUTE_CONTROL: u8 = 0x01;
const VOLUME_CONTROL: u8 = 0x02;

/// Minimal class owning two isochronous data endpoints, mirroring the UAC's
/// iso OUT (playback) + iso IN (capture) shape, with alt 0 (idle) / alt 1 (live).
struct IsoLoop<'a, B: usb_device::bus::UsbBus> {
    iface: InterfaceNumber,
    ep_out: EndpointOut<'a, B>,
    ep_in: EndpointIn<'a, B>,
    ep_fb: EndpointIn<'a, B>, // explicit feedback for the OUT (playback) path
    alt: u8,                  // current alt setting: 0 = idle, 1 = streaming
    spk_mute: bool,           // speaker Feature Unit (entity 5)
    spk_volume: i16,
    mic_mute: bool, // mic Feature Unit (entity 6)
    mic_volume: i16,
}

impl<'a, B: usb_device::bus::UsbBus> IsoLoop<'a, B> {
    fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        Self {
            iface: alloc.interface(),
            ep_out: alloc.isochronous(Sync::Asynchronous, Usage::Data, PACKET_SIZE, 1),
            ep_in: alloc.isochronous(Sync::Asynchronous, Usage::Data, PACKET_SIZE, 1),
            ep_fb: alloc.isochronous(Sync::NoSynchronization, Usage::Feedback, 3, 1),
            alt: 0,
            spk_mute: false,
            spk_volume: 0,
            mic_mute: false,
            mic_volume: 0,
        }
    }

    fn active(&self) -> bool {
        self.alt == 1
    }

    // Is `req` a class request on one of our Feature Units? Returns (entity, selector).
    fn fu_selector(&self, req: &control::Request) -> Option<(u8, u8)> {
        let entity = (req.index >> 8) as u8;
        (req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && (entity == FU_SPEAKER_ID || entity == FU_MIC_ID))
            .then_some((entity, (req.value >> 8) as u8))
    }
}

impl<'a, B: usb_device::bus::UsbBus> UsbClass<B> for IsoLoop<'a, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> UsbResult<()> {
        // Vendor-specific interface with alt 0 (idle, no endpoints) and alt 1
        // (the two iso endpoints), mirroring UAC. The host selects alt 1 to
        // stream. The sim host does not parse the descriptor; the alt structure
        // matters for the SET_INTERFACE round-trip, not descriptor parsing.
        writer.interface_alt(self.iface, 0, 0xFF, 0x00, 0x00, None)?;
        writer.interface_alt(self.iface, 1, 0xFF, 0x00, 0x00, None)?;
        writer.endpoint(&self.ep_out)?;
        writer.endpoint(&self.ep_in)?;
        writer.endpoint(&self.ep_fb)?;
        Ok(())
    }

    fn reset(&mut self) {
        self.alt = 0;
    }

    fn get_alt_setting(&mut self, interface: InterfaceNumber) -> Option<u8> {
        (u8::from(interface) == u8::from(self.iface)).then_some(self.alt)
    }

    fn set_alt_setting(&mut self, interface: InterfaceNumber, alternative: u8) -> bool {
        if alternative <= 1 && u8::from(interface) == u8::from(self.iface) {
            self.alt = alternative;
            true
        } else {
            false
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = *xfer.request();
        if req.request != SET_CUR {
            return;
        }
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

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = *xfer.request();
        if req.request != GET_CUR {
            return;
        }
        let Some((entity, selector)) = self.fu_selector(&req) else {
            return;
        };
        let (mute, volume) = if entity == FU_MIC_ID {
            (self.mic_mute, self.mic_volume)
        } else {
            (self.spk_mute, self.spk_volume)
        };
        match selector {
            MUTE_CONTROL => {
                let _ = xfer.accept_with(&[mute as u8]);
            }
            VOLUME_CONTROL => {
                let _ = xfer.accept_with(&volume.to_le_bytes());
            }
            _ => {}
        }
    }
}

/// Everything the `OTG_FS` handler needs — the device, the class, and the packet
/// / interrupt counters — shared between `main` (installs it) and the interrupt
/// (drives it). `cortex_m::interrupt::Mutex` gates access to a critical section.
struct Shared {
    dev: UsbDevice<'static, Bus>,
    class: IsoLoop<'static, Bus>,
    rx_count: u32,
    isr_count: u32,
}

static SHARED: Mutex<RefCell<Option<Shared>>> = Mutex::new(RefCell::new(None));

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

    // 'static bus allocator (see USB_ALLOC) so the device + class can be stored
    // in SHARED for the interrupt handler.
    let alloc: &'static UsbBusAllocator<Bus> = unsafe {
        let p = core::ptr::addr_of_mut!(USB_ALLOC);
        (*p).write(UsbBus::new(usb, &mut *core::ptr::addr_of_mut!(EP_MEMORY)));
        (*p).assume_init_ref()
    };

    let class = IsoLoop::new(alloc);
    let dev = UsbDeviceBuilder::new(alloc, UsbVidPid(0x1209, 0x0002))
        .strings(&[StringDescriptors::default().product("daisy usb-iso")])
        .expect("string descriptors")
        .max_packet_size_0(64)
        .expect("ep0 size")
        .build();

    interrupt_free(|cs| {
        SHARED.borrow(cs).replace(Some(Shared {
            dev,
            class,
            rx_count: 0,
            isr_count: 0,
        }));
    });

    // build() already ran UsbBus::enable() (GAHBCFG.GINT + the GINTMSK sources),
    // so unmasking the NVIC line is all that starts interrupt-driven servicing.
    unsafe { pac::NVIC::unmask(pac::Interrupt::OTG_FS) };

    // The main loop does NO USB work — all servicing is in OTG_FS below.
    loop {
        wfi();
    }
}

/// OTG_FS global interrupt — the sole USB servicer. Poll the stack; while the
/// host has activated the stream (alt 1), loop the playback packet back to
/// capture. `main` never polls, so reaching Configured and looping frames here
/// proves the interrupt path drives USB end to end.
#[interrupt]
fn OTG_FS() {
    interrupt_free(|cs| {
        if let Some(s) = SHARED.borrow(cs).borrow_mut().as_mut() {
            service(s);
        }
    });
}

fn service(s: &mut Shared) {
    s.isr_count = s.isr_count.wrapping_add(1);

    let mut buf = [0u8; 256];
    if s.dev.poll(&mut [&mut s.class]) && s.class.active() {
        if let Ok(n) = s.class.ep_out.read(&mut buf) {
            s.rx_count = s.rx_count.wrapping_add(1);
            let first = (buf[0] as u32) | ((buf[1] as u32) << 8);
            let _ = s.class.ep_in.write(&buf[..n]);
            unsafe {
                core::ptr::write_volatile(MARK_RXCOUNT, s.rx_count);
                core::ptr::write_volatile(MARK_RXBYTES, n as u32);
                core::ptr::write_volatile(MARK_FIRST, first);
            }
        }
        // Publish the explicit-feedback value (3-byte 10.14 Ff) on the feedback
        // endpoint — the host reads it to rate-match its OUT stream.
        let fb = NOMINAL_FEEDBACK_Q10_14.to_le_bytes();
        let _ = s.class.ep_fb.write(&fb[..3]);
    }

    unsafe {
        core::ptr::write_volatile(MARK_ISR, s.isr_count);
        core::ptr::write_volatile(MARK_ALT, s.class.alt as u32);
        core::ptr::write_volatile(MARK_MUTE, s.class.spk_mute as u32);
        core::ptr::write_volatile(MARK_VOL, (s.class.spk_volume as u16) as u32);
        core::ptr::write_volatile(MARK_STATE, state_code(s.dev.state()));
    }
}
