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
//! USB is serviced by [`service_usb`] from the `OTG_FS` interrupt: it polls the
//! composite stack and moves audio between the UAC iso endpoints and the codec
//! rings on every event. The idle main loop only has to avoid sleeping — the OTG
//! interrupt does not reliably wake the M7 from `wfi` on this XIP setup even
//! though every RM-required condition is met (GINTMSK/GAHBCFG.GINT/NVIC IRQ 101,
//! USB2OTG(LP)EN, QSPILPEN, PCGCCTL ungated, HSI48, SLEEPDEEP=0), an H7 D2-domain
//! Sleep-wakeup subtlety — so it spins (`nop`) rather than `wfi` (under `wfi` the
//! ISR fired ~2× and enumeration stalled at Default; kept awake it fires
//! normally). Moot for an audio device, whose sample loop never idles. Sources:
//! `docs/references.md`. The default build loops the host's UAC playback stream
//! straight back to its capture stream (HW-validated end-to-end: a 1 kHz tone
//! played to the device is recorded back intact). Audio is gated on the host's
//! alt setting; wire the on-board codec with `--features seed3`.

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
#[cfg(all(not(feature = "renode_test"), feature = "dsp-loop"))]
mod dsp_loop;
#[cfg(not(feature = "renode_test"))]
mod uac;
#[cfg(not(feature = "renode_test"))]
use daisy_midi::UsbMidiClass;
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
    #[cfg(feature = "dsp-loop")]
    dsp: dsp_loop::DspLoop,
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
// --- codec-build-only bring-up helpers (D2-SRAM clock + backup-SRAM markers) ---
// All `#[cfg(feature = "codec")]` so the default (verified) app links none of it.
#[cfg(feature = "codec")]
const RCC_AHB2ENR: *mut u32 = 0x5802_44DC as *mut u32;
#[cfg(feature = "codec")]
const RCC_AHB4ENR: *mut u32 = 0x5802_44E0 as *mut u32;
#[cfg(feature = "codec")]
const PWR_CR1: *mut u32 = 0x5802_4800 as *mut u32;
// Progress marker in Backup SRAM (0x3880_0200) — immune to the stack/.bss (unlike
// DTCM markers) and survives resets, so it shows the furthest boot stage reached
// even across a reset loop. Needs `enable_backup_access()` (BKPRAMEN + DBP) first.
#[cfg(feature = "codec")]
const BKP_MARK: *mut u32 = 0x3880_0200 as *mut u32;
#[cfg(feature = "codec")]
#[inline(always)]
unsafe fn bmark(n: u32) {
    core::ptr::write_volatile(BKP_MARK, n);
    cortex_m::asm::dsb();
}
/// Enable writes to Backup SRAM: BKPRAMEN (RCC_AHB4ENR b28, the clock) + DBP
/// (PWR_CR1 b8, disable backup-domain write protection). A reset clears DBP, and
/// the app never sets it, so backup-SRAM *writes* are otherwise dropped.
#[cfg(feature = "codec")]
#[inline(always)]
unsafe fn enable_backup_access() {
    core::ptr::write_volatile(RCC_AHB4ENR, core::ptr::read_volatile(RCC_AHB4ENR) | (1 << 28));
    core::ptr::write_volatile(PWR_CR1, core::ptr::read_volatile(PWR_CR1) | (1 << 8));
    cortex_m::asm::dsb();
}

/// Enable the D2-domain SRAM1/2/3 clocks (RCC_AHB2ENR bits 29-31). They are OFF
/// at reset, so any access to `0x3000_0000` — our non-cacheable DMA pool, where
/// the SAI/DMA audio buffers (`.sram_d2`) live — faults with an imprecise
/// BusError until they're on. Read-back + DSB so the enable takes effect before
/// first use (an RCC clock-enable has a short latency).
#[cfg(feature = "codec")]
#[inline(always)]
unsafe fn enable_d2_sram() {
    let v = core::ptr::read_volatile(RCC_AHB2ENR) | (0b111 << 29);
    core::ptr::write_volatile(RCC_AHB2ENR, v);
    let _ = core::ptr::read_volatile(RCC_AHB2ENR);
    cortex_m::asm::dsb();
}

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
    // The codec build (only) uses the D2-SRAM DMA pool + backup-SRAM markers.
    // Everything else here is identical to the default (verified) app.
    #[cfg(feature = "codec")]
    {
        enable_backup_access(); // before any bmark
        enable_d2_sram(); // before configure_mpu_and_caches maps 0x3000_0000
    }
    configure_mpu_and_caches();
    enable_dwt();
    led_output();
    #[cfg(feature = "codec")]
    bmark(1); // pre_init complete
}

/// Capture faults reliably (the ST-Link's debug-register reads are flaky). Writes
/// the CPU's stacked faulting PC + CFSR/BFAR/HFSR to Backup SRAM `0x3880_0210..`.
/// Codec-build-only debug aid; the default app keeps cortex-m-rt's default.
#[cfg(feature = "codec")]
#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    let m = 0x3880_0210 as *mut u32; // Backup SRAM, next to the stage marker
    core::ptr::write_volatile(m, 0xFA17_0000);
    core::ptr::write_volatile(m.add(1), ef.pc()); // faulting instruction address
    core::ptr::write_volatile(m.add(2), core::ptr::read_volatile(0xE000_ED28 as *const u32)); // CFSR
    core::ptr::write_volatile(m.add(3), core::ptr::read_volatile(0xE000_ED38 as *const u32)); // BFAR
    core::ptr::write_volatile(m.add(4), core::ptr::read_volatile(0xE000_ED2C as *const u32)); // HFSR
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    #[cfg(feature = "codec")]
    unsafe {
        bmark(2)
    }; // main() entered (.data/.bss init done)
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
    #[cfg(feature = "codec")]
    unsafe {
        bmark(3)
    }; // run() entered
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
    #[cfg(feature = "codec")]
    unsafe {
        bmark(4)
    }; // USB device built, before codec block

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

    // Bring up the on-board codec (original Daisy Seed = AKM AK4556) and route
    // UAC audio through it. The AK4556 has no I2C — its format is hardware-
    // strapped on the board, and it is brought out of power-down via a reset
    // GPIO on PB11 (there is no I2C bus to configure, unlike the WM8731).
    // daisy-audio sets up SAI1 (block A=TX master, B=RX slave, 24-bit) + DMA;
    // `audio_process` (DMA IRQ) moves samples between the SPSC rings and the
    // SAI, and OTG_FS moves them between the rings and the UAC iso endpoints.
    // Guitar → codec IN → capture ring → UAC IN is the DI-interface path.
    #[cfg(feature = "ak4556")]
    let _codec = {
        // Bring-up stage markers in Backup SRAM to localise faults.
        let st = |n: u32| unsafe { bmark(n) };
        st(5);
        let clocks = codec::recover_clocks().expect("CoreClocks hand-off from the bootloader");
        st(6);
        // Point the SAI1 kernel mux at PLL3P (bootloader ran PLL3) so `i2s_ch_a`
        // computes MCKDIV from the real ~49.152 MHz kernel clock.
        let sai1_rec = rec.SAI1.kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::Pll3P);
        let gpioe = dp.GPIOE.split(rec.GPIOE);
        let gpiob = dp.GPIOB.split(rec.GPIOB);
        st(7);
        let pins = daisy_audio::Pins {
            mclk_a: gpioe.pe2,
            sck_a: gpioe.pe5,
            fs_a: gpioe.pe4,
            sd_a: gpioe.pe6,
            sd_b: gpioe.pe3,
            // AK4556 reset (active-low PDN) on PB11; Audio::new drives the pulse.
            codec_reset: gpiob.pb11.into_push_pull_output(),
        };
        st(9);
        let mut codec_audio =
            daisy_audio::Audio::new(dp.SAI1, dp.DMA1, rec.DMA1, sai1_rec, pins, &clocks);
        st(10);
        codec_audio.start(codec::audio_process);
        st(11);
        codec_audio
    };

    // Bring up the on-board codec (Seed 1.1 = WM8731) and route UAC audio through
    // it. Unlike the TAC5242, the WM8731 is configured over I2C2 (SCL=PH4,
    // SDA=PB11, open-drain AF4) and has NO reset line. daisy-audio sets up SAI1
    // (block A=RX master, B=TX slave, 24-bit) + DMA; `audio_process` (DMA IRQ)
    // moves samples between the SPSC rings and the SAI, and the OTG_FS interrupt
    // moves them between the rings and the UAC iso endpoints. Guitar → codec IN
    // → capture ring → UAC IN is the DI-interface path.
    #[cfg(all(feature = "codec", not(feature = "seed3"), not(feature = "ak4556")))]
    let _codec = {
        // Bring-up stage markers in Backup SRAM to localise faults.
        let st = |n: u32| unsafe { bmark(n) };
        st(5);
        let clocks = codec::recover_clocks().expect("CoreClocks hand-off from the bootloader");
        st(6);
        // Point the SAI1 kernel mux at PLL3P (bootloader ran PLL3) so `i2s_ch_a`
        // computes MCKDIV from the real ~49.152 MHz kernel clock.
        let sai1_rec = rec.SAI1.kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::Pll3P);
        let gpioe = dp.GPIOE.split(rec.GPIOE);
        let gpiob = dp.GPIOB.split(rec.GPIOB);
        let gpioh = dp.GPIOH.split(rec.GPIOH);
        st(7);
        // WM8731 control bus: I2C2 @ 400 kHz on PH4 (SCL) / PB11 (SDA).
        let i2c2 = dp.I2C2.i2c(
            (
                gpioh.ph4.into_alternate_open_drain(),
                gpiob.pb11.into_alternate_open_drain(),
            ),
            400.kHz(),
            rec.I2C2,
            &clocks,
        );
        st(8);
        let pins = daisy_audio::Pins {
            mclk_a: gpioe.pe2,
            sck_a: gpioe.pe5,
            fs_a: gpioe.pe4,
            sd_a: gpioe.pe6,
            sd_b: gpioe.pe3,
        };
        st(9);
        let mut codec_audio =
            daisy_audio::Audio::new(dp.SAI1, dp.DMA1, rec.DMA1, sai1_rec, i2c2, pins, &clocks);
        st(10);
        codec_audio.start(codec::audio_process);
        st(11);
        codec_audio
    };

    // Daisy Pod: bring up USART1 RX (PB7 / Seed D14, 31250 baud 8N1) to read the
    // hardware DIN/TRS MIDI-IN. RX-only — the Pod's MIDI is input-only — and the
    // bootloader's frozen CoreClocks (via the hand-off) fixes the baud divisor.
    #[cfg(feature = "pod")]
    let (mut midi_rx, mut midi_enc) = {
        // SAFETY: reads Backup SRAM; sound because this app and the bootloader
        // share the workspace (identical HAL) and `restore` guards the layout.
        let clocks = unsafe { daisy_bsp::clocks::handoff::restore() }
            .expect("CoreClocks hand-off from the bootloader (needed for USART1)");
        let gpiob = dp.GPIOB.split(rec.GPIOB);
        let rx_pin = gpiob.pb7.into_alternate::<7>();
        let serial1 = dp
            .USART1
            .serial(
                (hal::serial::NoTx, rx_pin),
                31_250.bps(),
                rec.USART1,
                &clocks,
            )
            .expect("USART1 MIDI serial");
        let (_tx, rx) = serial1.split();
        (rx, daisy_midi::UsbMidiEncoder::new(0))
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
            #[cfg(feature = "dsp-loop")]
            dsp: dsp_loop::DspLoop::new(),
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
        // Pod: forward hardware DIN MIDI-IN to the host. Drain the UART, packetize
        // (running status / real-time / SysEx handled in daisy-midi), and write
        // each USB-MIDI event packet to the bulk IN under the OTG_FS lock.
        #[cfg(feature = "pod")]
        {
            while let Ok(b) = midi_rx.read() {
                if let Some(pkt) = midi_enc.push(b) {
                    interrupt_free(|cs| {
                        if let Some(u) = USB.borrow(cs).borrow_mut().as_mut() {
                            let _ = u.midi.write(&pkt);
                        }
                    });
                }
            }
        }

        // USB is serviced entirely by the `OTG_FS` interrupt (below). The main
        // loop's only job here is to NOT sleep: the OTG interrupt does not
        // reliably wake the M7 from `wfi` on this XIP setup — even though every
        // RM-required condition is met (GINTMSK/GAHBCFG.GINT/NVIC IRQ101,
        // USB2OTG(LP)EN, QSPILPEN, PCGCCTL ungated, HSI48, SLEEPDEEP=0), an H7
        // D2-domain Sleep-wakeup subtlety. Under `wfi` the ISR fired ~2× and
        // enumeration stalled at Default; kept awake the ISR fires normally and
        // enumerates. "USB OTG + wfi → CPU doesn't wake" is known cross-family
        // STM32 behavior — the fix is to not sleep while running OTG (moot for
        // audio, whose sample loop never idles). Sources: docs/references.md
        // (§Web-sourced).
        #[cfg(all(not(feature = "tui"), not(feature = "pod")))]
        {
            cortex_m::asm::nop();
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
            let gain = u.audio.gain();
            if let Ok(n) = u.audio.read_playback(&mut audio_buf) {
                codec::push_playback_bytes(&audio_buf[..n], gain);
            }
            // Explicit feedback: steer the host's OUT rate to hold the playback
            // ring near half full. Proportional nudge, clamped to ±1 sample; the
            // gain is a placeholder to tune on hardware against real clock drift.
            let err = (codec::RING_CAPACITY / 2) as i32 - codec::playback_len() as i32;
            let adj = (err * 64).clamp(-(1 << 14), 1 << 14);
            let ff = (uac::NOMINAL_FEEDBACK_Q10_14 as i32 + adj) as u32;
            let _ = u.audio.write_feedback(ff);
        }
        if u.audio.capture_active() {
            // Async IN sizing: send one sample more/less than nominal to hold the
            // capture ring near half full (the codec ADC clock is the master).
            let fill = codec::capture_len();
            let target = codec::RING_CAPACITY / 2;
            let bytes = if fill > target + 24 {
                uac::NOMINAL_CAPTURE_BYTES + uac::FRAME_BYTES
            } else if fill + 24 < target {
                uac::NOMINAL_CAPTURE_BYTES - uac::FRAME_BYTES
            } else {
                uac::NOMINAL_CAPTURE_BYTES
            };
            let n = codec::pop_capture_bytes(&mut audio_buf[..bytes], u.audio.capture_gain());
            if n > 0 {
                let _ = u.audio.write_capture(&audio_buf[..n]);
            }
        }
    }
    #[cfg(not(feature = "codec"))]
    {
        if u.audio.playback_active() && u.audio.capture_active() {
            // Loopback is host → speaker → mic → host, so both Feature Units apply.
            let gain = u.audio.gain() * u.audio.capture_gain();
            if let Ok(n) = u.audio.read_playback(&mut audio_buf) {
                apply_gain_i16le(&mut audio_buf[..n], gain);
                // With `dsp-loop`, run the block through a daisy-dsp core (biquad)
                // before it goes back to the host — the streaming HW DSP test.
                #[cfg(feature = "dsp-loop")]
                u.dsp.process_i16le_stereo(&mut audio_buf[..n]);
                let _ = u.audio.write_capture(&audio_buf[..n]);
            }
        }
        // No codec clock to track in loopback: keep the feedback endpoint serviced
        // with the nominal rate so the host's OUT scheduling stays happy.
        if u.audio.playback_active() {
            let _ = u.audio.write_feedback(uac::NOMINAL_FEEDBACK_Q10_14);
        }
    }

    // MIDI: loop host→host (default). Under `pod`, hardware DIN MIDI-IN drives
    // device→host from the main loop, so here we just drain any host→device
    // packets to keep the bulk OUT from backing up.
    let mut midi_buf = [0u8; 64];
    #[cfg(not(feature = "pod"))]
    if let Ok(n) = u.midi.read(&mut midi_buf) {
        let _ = u.midi.write(&midi_buf[..n]);
    }
    #[cfg(feature = "pod")]
    {
        let _ = u.midi.read(&mut midi_buf);
    }
}

/// Scale an interleaved little-endian i16 buffer in place by `gain` — applies the
/// UAC Feature Unit's volume/mute to the loopback path (the codec build scales in
/// `codec::push_playback_bytes` instead).
#[cfg(all(not(feature = "renode_test"), not(feature = "codec")))]
fn apply_gain_i16le(buf: &mut [u8], gain: f32) {
    if gain == 1.0 {
        return;
    }
    for s in buf.chunks_exact_mut(2) {
        let v = i16::from_le_bytes([s[0], s[1]]) as f32 / 32768.0 * gain;
        let o = (v.clamp(-1.0, 1.0) * 32767.0) as i16;
        s.copy_from_slice(&o.to_le_bytes());
    }
}
