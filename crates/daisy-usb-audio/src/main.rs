#![no_std]
#![no_main]
#![allow(deprecated)] // cortex-m-rt 0.7 #[pre_init]; migrate later.

//! Daisy Seed USB composite device: **CDC-ACM serial + UAC1 audio (stereo
//! 48 kHz in AND out) + USB-MIDI**, simultaneously, over the on-board
//! micro-USB (OTG2_HS in FS mode, PA11/PA12).
//!
//! This is the **XIP application** form: it is linked to execute in place from
//! QSPI flash at 0x9000_0000 and is loaded onto the board with `daisy flash`
//! through the bootloader (which validates the vector table and jumps). It
//! therefore runs AFTER the bootloader froze the clocks, so it must NOT call
//! `clocks::init` again. USB is brought up freeze-free:
//!
//!   * The bootloader already left HSI48 running, the USB kernel mux on HSI48,
//!     and PWR.CR3.USB33DEN set — so no clock-tree config is needed here.
//!   * `USB2::new` needs a `CoreClocks` (which only `freeze()` mints), so we
//!     bypass it: mint the `rec::Usb2Otg` peripheral token via the HAL's
//!     `unsafe steal_peripheral_rec()`, and build the (all-`pub`-field) `USB2`
//!     struct literal directly with a hard-coded `hclk` (the frozen 200 MHz,
//!     used only for a >30 MHz sanity check). `UsbBus::new` re-runs the
//!     idempotent OTG2 enable/reset.
//!
//! `#[pre_init]` sets up the MPU + L1 caches + DWT (same as the app template).
//! `EP_MEMORY` lives in DTCM, which the Cortex-M7 never caches, so it is
//! DMA/USB-coherent without a special non-cacheable section.
//!
//! Status: enumerates as a composite CDC + UAC + MIDI device. Audio/MIDI are
//! loopback stubs; wire `daisy-audio` (codec SAI/DMA) in for real I/O. Validate
//! USB streaming on hardware (Renode has no OTG model yet).

use cortex_m_rt::{entry, pre_init};
use daisy_bsp::hal;
use hal::pac;
use panic_halt as _;

#[cfg(not(feature = "renode_test"))]
use hal::{
    prelude::*,
    time::Hertz,
    usb_hs::{UsbBus, USB2},
};
#[cfg(not(feature = "renode_test"))]
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};
#[cfg(not(feature = "renode_test"))]
use usbd_serial::SerialPort;

#[cfg(feature = "codec")]
mod codec;
#[cfg(not(feature = "renode_test"))]
mod midi;
#[cfg(not(feature = "renode_test"))]
mod uac;
#[cfg(not(feature = "renode_test"))]
use midi::UsbMidiClass;
#[cfg(not(feature = "renode_test"))]
use uac::UsbAudioClass;

// OTG2_HS (FS mode) has 4 KiB of dedicated FIFO RAM. In DTCM (never cached).
#[cfg(not(feature = "renode_test"))]
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[cfg(not(feature = "renode_test"))]
const VID: u16 = 0x1209;
#[cfg(not(feature = "renode_test"))]
const PID: u16 = 0xDA15;

// --- GPIO / debug / MPU registers (raw, for #[pre_init]) ---
const GPIOC_MODER: *mut u32 = 0x5802_0800 as *mut u32;
const GPIOC_BSRR: *mut u32 = 0x5802_0818 as *mut u32;
const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;
const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32 = 0xE000_EDA0 as *mut u32;

#[inline(always)]
unsafe fn led_output() {
    let mut m = core::ptr::read_volatile(GPIOC_MODER);
    m &= !(0b11 << 14);
    m |= 0b01 << 14;
    core::ptr::write_volatile(GPIOC_MODER, m);
}

#[inline(always)]
unsafe fn enable_dwt() {
    core::ptr::write_volatile(DEMCR, core::ptr::read_volatile(DEMCR) | (1 << 24));
    core::ptr::write_volatile(DWT_CTRL, core::ptr::read_volatile(DWT_CTRL) | 1);
}

#[inline(always)]
unsafe fn mpu_region(n: u32, base: u32, log2_bytes: u32, tex: u32, s: u32, c: u32, b: u32, xn: u32) {
    core::ptr::write_volatile(MPU_RNR, n);
    core::ptr::write_volatile(MPU_RBAR, base);
    let rasr = 1
        | ((log2_bytes - 1) << 1)
        | (b << 16)
        | (c << 17)
        | (s << 18)
        | (tex << 19)
        | (0b011 << 24)
        | (xn << 28);
    core::ptr::write_volatile(MPU_RASR, rasr);
}

/// MPU (libDaisy regions) + L1 caches. See the app-template for the rationale;
/// MPU-before-cache ordering is mandatory.
unsafe fn configure_mpu_and_caches() {
    core::ptr::write_volatile(MPU_CTRL, 0);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    mpu_region(0, 0x3000_0000, 15, 1, 1, 0, 0, 0); // SRAM_D2 DMA pool, non-cacheable
    mpu_region(1, 0xC000_0000, 26, 0, 0, 1, 1, 0); // SDRAM, write-back cacheable
    mpu_region(2, 0x3880_0000, 12, 1, 1, 0, 0, 0); // Backup SRAM, non-cacheable
    core::ptr::write_volatile(MPU_CTRL, (1 << 0) | (1 << 2)); // ENABLE | PRIVDEFENA
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    let mut cp = cortex_m::Peripherals::steal();
    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
}

#[pre_init]
unsafe fn pre_init() {
    configure_mpu_and_caches();
    enable_dwt();
    led_output();
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    run(dp)
}

/// Renode boot check: sim has no OTG_HS model, so skip USB and prove the XIP
/// app booted + `pre_init` ran by toggling the LED (Renode samples PC7).
#[cfg(feature = "renode_test")]
fn run(_dp: pac::Peripherals) -> ! {
    let mut hb: u32 = 0;
    loop {
        hb = hb.wrapping_add(1);
        unsafe {
            core::ptr::write_volatile(GPIOC_BSRR, if hb & 0x8_0000 != 0 { 1 << 7 } else { 1 << 23 });
        }
    }
}

#[cfg(not(feature = "renode_test"))]
fn run(dp: pac::Peripherals) -> ! {
    // Mint the USB2 peripheral clock token WITHOUT freezing (the bootloader
    // already configured the clock tree). `constrain()` just wraps RCC.
    let rcc = dp.RCC.constrain();
    let rec = unsafe { rcc.steal_peripheral_rec() };

    // PA11/PA12 → AF10 (OTG2_HS FS). split() enables the GPIOA clock via its rec.
    let gpioa = dp.GPIOA.split(rec.GPIOA);
    let _pin_dm = gpioa.pa11.into_alternate::<10>();
    let _pin_dp = gpioa.pa12.into_alternate::<10>();

    // Defensive: ensure the external USB 3.3 V detector is up (bootloader sets it).
    unsafe {
        let pwr = &*pac::PWR::ptr();
        pwr.cr3.modify(|_, w| w.usb33den().set_bit());
        while pwr.cr3.read().usb33rdy().bit_is_clear() {}
    }

    // Build USB2 directly (all fields pub) — no CoreClocks / freeze needed.
    let usb = USB2 {
        usb_global: dp.OTG2_HS_GLOBAL,
        usb_device: dp.OTG2_HS_DEVICE,
        usb_pwrclk: dp.OTG2_HS_PWRCLK,
        prec: rec.USB2OTG,
        hclk: Hertz::from_raw(200_000_000),
    };
    let usb_bus = UsbBus::new(usb, unsafe { &mut *core::ptr::addr_of_mut!(EP_MEMORY) });

    let mut serial = SerialPort::new(&usb_bus);
    let mut audio = UsbAudioClass::new(&usb_bus);
    let mut midi = UsbMidiClass::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(VID, PID))
        .composite_with_iads()
        .device_class(0xEF)
        .device_sub_class(0x02)
        .device_protocol(0x01)
        .strings(&[StringDescriptors::default()
            .manufacturer("daisy-rs")
            .product("Daisy USB Audio + Serial + MIDI")
            .serial_number("DAISY-USB-1")])
        .expect("string descriptors")
        .max_packet_size_0(64)
        .expect("ep0 size")
        .build();

    let mut audio_buf = [0u8; uac::AUDIO_PACKET_SIZE as usize];
    let mut midi_buf = [0u8; 64];
    let mut heartbeat: u32 = 0;

    loop {
        // LED heartbeat so the XIP boot is observable (Renode samples PC7).
        heartbeat = heartbeat.wrapping_add(1);
        unsafe {
            core::ptr::write_volatile(GPIOC_BSRR, if heartbeat & 0x8_0000 != 0 { 1 << 7 } else { 1 << 23 });
        }

        if !usb_dev.poll(&mut [&mut serial, &mut audio, &mut midi]) {
            continue;
        }

        // CDC echo.
        let mut buf = [0u8; 64];
        if let Ok(count) = serial.read(&mut buf) {
            let mut written = 0;
            while written < count {
                match serial.write(&buf[written..count]) {
                    Ok(n) => written += n,
                    Err(_) => break,
                }
            }
        }

        // Audio: bridge the UAC iso endpoints to the codec rings (host
        // playback → codec, codec capture → host) when the `codec` feature is
        // on; otherwise loop playback straight back to capture.
        #[cfg(feature = "codec")]
        {
            if let Ok(n) = audio.read_playback(&mut audio_buf) {
                codec::push_playback_bytes(&audio_buf[..n]);
            }
            let n = codec::pop_capture_bytes(&mut audio_buf);
            if n > 0 {
                let _ = audio.write_capture(&audio_buf[..n]);
            }
        }
        #[cfg(not(feature = "codec"))]
        {
            if let Ok(n) = audio.read_playback(&mut audio_buf) {
                let _ = audio.write_capture(&audio_buf[..n]);
            }
        }

        // MIDI loopback.
        if let Ok(n) = midi.read(&mut midi_buf) {
            let _ = midi.write(&midi_buf[..n]);
        }
    }
}
