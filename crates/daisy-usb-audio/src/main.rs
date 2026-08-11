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
//! USB is serviced from the `OTG_FS` interrupt: the device + its three classes
//! live in a shared cell, and the handler polls the stack and moves audio between
//! the UAC iso endpoints and the codec rings on every USB event — so servicing
//! never depends on main-loop latency (the main loop only drives the LED, and,
//! with `tui`, the CDC terminal UI). Audio is gated on the host's alt setting;
//! wire the codec with `--features seed3`. The interrupt-driven servicing pattern
//! is proven in sim by `usb-iso-exerciser` / `otg_iso.robot`; end-to-end
//! isochronous streaming still needs hardware validation.

use cortex_m_rt::{entry, pre_init};
use daisy_bsp::hal;
use hal::pac;
use panic_halt as _;

#[cfg(not(feature = "renode_test"))]
use core::cell::RefCell;
#[cfg(not(feature = "renode_test"))]
use cortex_m::interrupt::{free as interrupt_free, Mutex};
#[cfg(not(feature = "renode_test"))]
use hal::pac::interrupt;
#[cfg(not(feature = "renode_test"))]
use hal::{
    prelude::*,
    time::Hertz,
    usb_hs::{UsbBus, USB2},
};
#[cfg(not(feature = "renode_test"))]
use usb_device::bus::UsbBusAllocator;
#[cfg(not(feature = "renode_test"))]
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
#[cfg(not(feature = "renode_test"))]
use usbd_serial::SerialPort;

#[cfg(feature = "codec")]
mod codec;
#[cfg(feature = "tui")]
mod tui;

// ratatui needs a global allocator. Placed in AXI SRAM (0x2400_0000, 512 KiB),
// which our linker script leaves unused — so the whole region is the heap's.
#[cfg(feature = "tui")]
mod heap {
    use embedded_alloc::LlffHeap as Heap;
    #[global_allocator]
    static HEAP: Heap = Heap::empty();
    /// Initialise the heap. Call once before any allocation.
    pub fn init() {
        const AXI_SRAM_BASE: usize = 0x2400_0000;
        const HEAP_SIZE: usize = 384 * 1024;
        // SAFETY: AXI SRAM is powered and unused by the linker script.
        unsafe { HEAP.init(AXI_SRAM_BASE, HEAP_SIZE) };
    }
}
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

#[cfg(not(feature = "renode_test"))]
type Bus = UsbBus<USB2>;

// The bus allocator must outlive the device + classes (which borrow it) so they
// can live in the shared static the OTG_FS interrupt reaches — 'static, set once.
#[cfg(not(feature = "renode_test"))]
static mut USB_ALLOC: core::mem::MaybeUninit<UsbBusAllocator<Bus>> =
    core::mem::MaybeUninit::uninit();

/// The composite device + its three classes, shared between `main` (which builds
/// and installs it) and the `OTG_FS` interrupt (which services USB every event).
#[cfg(not(feature = "renode_test"))]
struct UsbShared {
    dev: UsbDevice<'static, Bus>,
    serial: SerialPort<'static, Bus>,
    audio: UsbAudioClass<'static, Bus>,
    midi: UsbMidiClass<'static, Bus>,
}

#[cfg(not(feature = "renode_test"))]
static USB: Mutex<RefCell<Option<UsbShared>>> = Mutex::new(RefCell::new(None));

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
#[allow(clippy::too_many_arguments)] // one arg per MPU RASR field; a struct would just move the noise
unsafe fn mpu_region(
    n: u32,
    base: u32,
    log2_bytes: u32,
    tex: u32,
    s: u32,
    c: u32,
    b: u32,
    xn: u32,
) {
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
    mpu_region(3, 0x9000_0000, 23, 0, 0, 1, 0, 0); // QSPI XIP flash (8 MB), write-through cacheable + exec
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
            core::ptr::write_volatile(
                GPIOC_BSRR,
                if hb & 0x8_0000 != 0 { 1 << 7 } else { 1 << 23 },
            );
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
    // 'static bus allocator (see USB_ALLOC) so the device + classes can be moved
    // into USB for the OTG_FS interrupt handler.
    let alloc: &'static UsbBusAllocator<Bus> = unsafe {
        let p = core::ptr::addr_of_mut!(USB_ALLOC);
        (*p).write(UsbBus::new(usb, &mut *core::ptr::addr_of_mut!(EP_MEMORY)));
        (*p).assume_init_ref()
    };

    let serial = SerialPort::new(alloc);
    let audio = UsbAudioClass::new(alloc);
    let midi = UsbMidiClass::new(alloc);

    let dev = UsbDeviceBuilder::new(alloc, UsbVidPid(VID, PID))
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

    // Bring up the on-board codec (Seed 3 = TAC5242) and route UAC audio through
    // it. The codec is hardware-strapped (no I2C/reset) — daisy-audio just sets
    // up SAI1. `audio_process` (running in the DMA-IRQ) moves samples between the
    // SPSC rings and the SAI; the OTG_FS interrupt moves them between the rings
    // and the UAC iso endpoints. `_codec` is held for the app's lifetime.
    #[cfg(feature = "seed3")]
    let _codec = {
        let clocks = codec::recover_clocks().expect("CoreClocks hand-off from the bootloader");
        // Point the SAI1 kernel mux at PLL3P: the bootloader ran PLL3, and this
        // syncs the HAL rec token so `i2s_ch_a` computes MCKDIV from the real
        // ~49.152 MHz kernel clock (it also re-writes D2CCIP1R.SAI1SEL, harmlessly
        // matching the board-specific bootloader).
        let sai1_rec = rec.SAI1.kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::Pll3P);
        let gpioe = dp.GPIOE.split(rec.GPIOE);
        let pins = daisy_audio::Pins {
            mclk_a: gpioe.pe2,
            sck_a: gpioe.pe5,
            fs_a: gpioe.pe4,
            sd_a: gpioe.pe6,
            sd_b: gpioe.pe3,
        };
        let mut codec_audio =
            daisy_audio::Audio::new(dp.SAI1, dp.DMA1, rec.DMA1, sai1_rec, pins, &clocks);
        codec_audio.start(codec::audio_process);
        codec_audio
    };

    // Hand the device + classes to the OTG_FS interrupt, then start it. build()
    // already ran UsbBus::enable() (GAHBCFG.GINT + the GINTMSK sources), so
    // unmasking the NVIC line is all that begins interrupt-driven servicing.
    interrupt_free(|cs| {
        USB.borrow(cs).replace(Some(UsbShared {
            dev,
            serial,
            audio,
            midi,
        }));
    });
    unsafe { pac::NVIC::unmask(pac::Interrupt::OTG_FS) };

    // The CDC terminal UI (`tui` feature) renders from the main loop; audio, MIDI
    // and (non-tui) CDC echo are all serviced in the OTG_FS interrupt below.
    #[cfg(feature = "tui")]
    let mut tui = {
        heap::init();
        tui::Tui::new()
    };

    let mut heartbeat: u32 = 0;
    loop {
        // LED heartbeat so the XIP boot is observable (Renode samples PC7).
        heartbeat = heartbeat.wrapping_add(1);
        unsafe {
            core::ptr::write_volatile(
                GPIOC_BSRR,
                if heartbeat & 0x8_0000 != 0 {
                    1 << 7
                } else {
                    1 << 23
                },
            );
        }

        // With `tui`, drive the terminal UI: exchange CDC bytes with the shared
        // device inside the OTG_FS critical section, but keep the heavy render()
        // outside the lock. Without `tui`, the main loop has nothing to do — all
        // USB servicing is in the interrupt — so sleep until the next event.
        #[cfg(feature = "tui")]
        {
            interrupt_free(|cs| {
                if let Some(u) = USB.borrow(cs).borrow_mut().as_mut() {
                    let mut buf = [0u8; 64];
                    if let Ok(count) = u.serial.read(&mut buf) {
                        tui.on_input(&buf[..count]); // CPR size reply + keystrokes
                    }
                }
            });
            if !tui.output_pending() && heartbeat & 0x000F_FFFF == 0 {
                tui.render();
            }
            interrupt_free(|cs| {
                if let Some(u) = USB.borrow(cs).borrow_mut().as_mut() {
                    tui.drain_to(|bytes| u.serial.write(bytes).ok());
                }
            });
        }
        #[cfg(not(feature = "tui"))]
        {
            cortex_m::asm::wfi();
        }
    }
}

/// OTG_FS global interrupt — the sole USB servicer. Polls the composite device,
/// then services the classes: CDC echo (non-`tui`), the UAC iso ↔ codec-ring
/// bridge (gated on the host's alt setting), and MIDI loopback. Running on every
/// USB event means servicing never waits on the main loop's other work.
#[cfg(not(feature = "renode_test"))]
#[interrupt]
fn OTG_FS() {
    interrupt_free(|cs| {
        if let Some(u) = USB.borrow(cs).borrow_mut().as_mut() {
            service_usb(u);
        }
    });
}

#[cfg(not(feature = "renode_test"))]
fn service_usb(u: &mut UsbShared) {
    if !u.dev.poll(&mut [&mut u.serial, &mut u.audio, &mut u.midi]) {
        return;
    }

    // CDC echo (default). With `tui`, the main loop owns CDC interaction.
    #[cfg(not(feature = "tui"))]
    {
        let mut buf = [0u8; 64];
        if let Ok(count) = u.serial.read(&mut buf) {
            let mut written = 0;
            while written < count {
                match u.serial.write(&buf[written..count]) {
                    Ok(n) => written += n,
                    Err(_) => break,
                }
            }
        }
    }

    // Audio bridge, gated on the host having activated each stream (alt 1): with
    // the codec, host playback → rings and rings → host capture; otherwise loop
    // playback straight back to capture. We never touch an iso endpoint the host
    // isn't scheduling.
    let mut audio_buf = [0u8; uac::AUDIO_PACKET_SIZE as usize];
    #[cfg(feature = "codec")]
    {
        if u.audio.playback_active() {
            if let Ok(n) = u.audio.read_playback(&mut audio_buf) {
                codec::push_playback_bytes(&audio_buf[..n]);
            }
        }
        if u.audio.capture_active() {
            let n = codec::pop_capture_bytes(&mut audio_buf);
            if n > 0 {
                let _ = u.audio.write_capture(&audio_buf[..n]);
            }
        }
    }
    #[cfg(not(feature = "codec"))]
    {
        if u.audio.playback_active() && u.audio.capture_active() {
            if let Ok(n) = u.audio.read_playback(&mut audio_buf) {
                let _ = u.audio.write_capture(&audio_buf[..n]);
            }
        }
    }

    // MIDI loopback.
    let mut midi_buf = [0u8; 64];
    if let Ok(n) = u.midi.read(&mut midi_buf) {
        let _ = u.midi.write(&midi_buf[..n]);
    }
}
