#![no_std]
#![no_main]
#![allow(deprecated)] // cortex-m-rt 0.7 #[pre_init]; migrate later.

//! **Daisy Seed (in a Hothouse pedal) → live control panel over USB-CDC.**
//!
//! A QSPI-XIP app (loaded via the bootloader, like `daisy-usb-audio`): it boots,
//! brings up the on-board USB as a CDC-ACM serial port, and renders the Hothouse
//! front panel — six pot bars, three toggle positions, two footswitches — as a
//! ratatui TUI that refreshes live as you move the controls. Open the CDC port
//! in any terminal (`picocom -b 115200 /dev/tty…`).
//!
//! Bring-up mirrors `daisy-usb-audio`: `#[pre_init]` sets the MPU + L1 caches +
//! DWT; USB2 is built freeze-free from the peripheral tokens (the bootloader
//! already configured the clock tree, whose `CoreClocks` we recover from Backup
//! SRAM to clock the ADC + SysTick). The switch debounce and render are paced
//! off DWT cycle-count (≈1 kHz debounce, matching libDaisy; ≈20 Hz redraw), not
//! the raw loop rate.
//!
//! HARDWARE-ONLY for the UI (Renode has no OTG model). `--features renode_test`
//! skips USB so the XIP boot + `pre_init` can still be smoke-tested in sim.

use cortex_m_rt::{entry, pre_init};
use daisy_bsp::hal;
use hal::pac;
use panic_halt as _;

mod panel;

#[cfg(not(feature = "renode_test"))]
use daisy_bsp::hothouse::{Footswitch, Hothouse, Knobs, Leds, Switches, Toggle};
#[cfg(not(feature = "renode_test"))]
use hal::adc::Adc;
#[cfg(not(feature = "renode_test"))]
use hal::prelude::*;
#[cfg(not(feature = "renode_test"))]
use hal::time::Hertz;
#[cfg(not(feature = "renode_test"))]
use hal::usb_hs::{UsbBus, USB2};
#[cfg(not(feature = "renode_test"))]
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};
#[cfg(not(feature = "renode_test"))]
use usbd_serial::SerialPort;

// ratatui needs a global allocator. Placed in AXI SRAM (0x2400_0000, 512 KiB),
// which the linker script leaves unused — so the whole region is the heap's.
mod heap {
    use core::alloc::{GlobalAlloc, Layout};
    use embedded_alloc::LlffHeap as Heap;

    static HEAP: Heap = Heap::empty();

    // Diagnostic wrapper: records the last requested alloc size at 0x2001_0008
    // and stamps 0x2001_000C = 0xA110_FA11 on the first failure, so a debugger
    // can read over SWD exactly which allocation blew up (size vs overflow).
    struct Traced;
    unsafe impl GlobalAlloc for Traced {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            core::ptr::write_volatile(0x2001_0008 as *mut u32, layout.size() as u32);
            let p = HEAP.alloc(layout);
            if p.is_null() {
                core::ptr::write_volatile(0x2001_000C as *mut u32, 0xA110_FA11);
            }
            p
        }
        unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
            HEAP.dealloc(ptr, layout)
        }
    }

    #[global_allocator]
    static ALLOC: Traced = Traced;

    /// Initialise the heap. Call once before any allocation. Uses the full
    /// 512 KiB AXI SRAM (the app's stack/data live in DTCM, so it's all free).
    pub fn init() {
        const HEAP_BASE: usize = 0x2400_0000;
        const HEAP_SIZE: usize = 512 * 1024;
        unsafe { HEAP.init(HEAP_BASE, HEAP_SIZE) }
    }
}

#[cfg(not(feature = "renode_test"))]
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

// pd.org VID + our app PID (same block as daisy-usb-audio).
#[cfg(not(feature = "renode_test"))]
const VID: u16 = 0x1209;
#[cfg(not(feature = "renode_test"))]
const PID: u16 = 0xDA16;

// --- GPIO / debug / MPU registers (raw, for #[pre_init]) ---
const GPIOC_MODER: *mut u32 = 0x5802_0800 as *mut u32;
const GPIOC_BSRR: *mut u32 = 0x5802_0818 as *mut u32;
const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;
const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32 = 0xE000_EDA0 as *mut u32;

// --- SWD-visible boot progress markers in DTCM (0x2000_0000). The Cortex-M7
// never caches TCM, so a debugger reading over SWD sees writes immediately, with
// no MPU/cache/BKPRAMEN dependency. 0x2001_0000 sits above .data/.bss (~4 KiB at
// the base) and below the stack (top of the 128 KiB region), so it's untouched. ---
const MARK_STAGE: *mut u32 = 0x2001_0000 as *mut u32;
const MARK_LOOP: *mut u32 = 0x2001_0004 as *mut u32;
#[inline(always)]
unsafe fn mark(n: u32) {
    core::ptr::write_volatile(MARK_STAGE, n);
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
    core::ptr::write_volatile(MPU_CTRL, (1 << 0) | (1 << 2)); // ENABLE | PRIVDEFENA
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    mark(0x12); // MPU on, before caches
    let mut cp = cortex_m::Peripherals::steal();
    cp.SCB.enable_icache();
    mark(0x13); // I-cache on

    // D-cache DELIBERATELY LEFT OFF. On the H750, an enabled D-cache over the
    // QSPI/XIP region (0x9000_0000, Normal-cacheable in the default map) causes
    // speculative-read hard faults — a well-known Daisy/STM32H7 issue (see the
    // libDaisy forum "QSPI buffer data corruption/hard faults" thread and ST's
    // H750 SCB_EnableDCache reports). The I-cache (read-only code) is safe and
    // kept for speed. If the D-cache is wanted later, first add an MPU region
    // marking 0x9000_0000 non-cacheable/Device.
    // cp.SCB.enable_dcache(&mut cp.CPUID);
    let _ = &mut cp;
    mark(0x14); // caches configured (I-cache only)
}

#[pre_init]
unsafe fn pre_init() {
    mark(0x10); // pre_init entered
    configure_mpu_and_caches();
    enable_dwt();
    led_output();
    mark(0x1F); // pre_init done
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    run(dp)
}

/// Renode boot smoke test: no USB (no OTG model in sim). Just blink PC7 so the
/// XIP boot + `pre_init` are observable.
#[cfg(feature = "renode_test")]
fn run(_dp: pac::Peripherals) -> ! {
    let mut heartbeat: u32 = 0;
    loop {
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
    }
}

#[cfg(not(feature = "renode_test"))]
fn run(dp: pac::Peripherals) -> ! {
    // Initialise the heap allocator FIRST, before anything that might allocate.
    // With LTO the optimiser can hoist a later `Vec`/ratatui allocation ahead of
    // where `heap::init()` sits in source (no visible data dependency between
    // them), so the allocator would run uninitialised → null → alloc-error spin.
    // The compiler fence pins the ordering: no allocation can move before this.
    heap::init();
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    // Mint the peripheral clock tokens WITHOUT freezing (the bootloader already
    // configured the clock tree). Recover its frozen `CoreClocks` from Backup
    // SRAM — the ADC and the SysTick delay both need it.
    unsafe { mark(0x20) }; // run() entered
    let rcc = dp.RCC.constrain();
    let rec = unsafe { rcc.steal_peripheral_rec() };
    let clocks = unsafe { daisy_bsp::clocks::handoff::restore() }
        .expect("CoreClocks hand-off from the bootloader");
    unsafe { mark(0x21) }; // clocks recovered

    // Split every GPIO port the panel + USB touch (each enables its bus clock).
    let gpioa = dp.GPIOA.split(rec.GPIOA);
    unsafe { mark(0x31) };
    let gpiob = dp.GPIOB.split(rec.GPIOB);
    unsafe { mark(0x32) };
    let gpioc = dp.GPIOC.split(rec.GPIOC);
    unsafe { mark(0x33) };
    let gpiod = dp.GPIOD.split(rec.GPIOD);
    unsafe { mark(0x34) };
    // GPIOG carries PG6 = QSPI NCS, and we are executing from QSPI XIP right now.
    // The normal `.split()` does `prec.enable().reset()` — the reset pulse wipes
    // PG6's AF10 (QSPI) config, deasserts chip-select, and the next XIP
    // instruction fetch hard-hangs the core (verified: the app froze at exactly
    // this line, marker stuck at 0x34). `split_without_reset` enables the port
    // clock without the reset, preserving the bootloader's QSPI pin setup. The
    // A/B/C/D ports above are safe to reset — none of them carry a QSPI pin.
    let gpiog = dp.GPIOG.split_without_reset(rec.GPIOG);
    unsafe { mark(0x22) }; // GPIO split

    // USB pins PA11/PA12 → AF10 (OTG2_HS FS).
    let _pin_dm = gpioa.pa11.into_alternate::<10>();
    let _pin_dp = gpioa.pa12.into_alternate::<10>();
    unsafe { mark(0x23) }; // USB pins

    // Defensive: ensure the external USB 3.3 V detector is up (bootloader sets it).
    unsafe {
        let pwr = &*pac::PWR::ptr();
        pwr.cr3.modify(|_, w| w.usb33den().set_bit());
        while pwr.cr3.read().usb33rdy().bit_is_clear() {}
    }
    unsafe { mark(0x24) }; // USB33 ready

    // Hothouse front panel (control→pin map lives in daisy_bsp::hothouse).
    let switches = Switches::new(
        gpiob.pb4, gpiob.pb5, // toggle 1: D9 / D10
        gpiog.pg10, gpiog.pg11, // toggle 2: D7 / D8
        gpiod.pd2, gpioc.pc12, // toggle 3: D5 / D6
        gpioa.pa0, gpiod.pd11, // footswitch 1 / 2: D25 / D26
    );
    let leds = Leds::new(gpioa.pa5, gpioa.pa4); // LED 1 / 2: D22 / D23
    let knobs = Knobs::new(
        gpioa.pa3, gpiob.pb1, gpioa.pa7, gpioa.pa6, gpioc.pc1, gpioc.pc4, // K1..K6: D16..D21
    );
    let mut hothouse = Hothouse::new(switches, leds, knobs);
    unsafe { mark(0x25) }; // controls constructed

    // ADC1 for the six knobs (all on ADC1). Uses SysTick for the power-up delay.
    // `take()` would return None here: pre_init already did `Peripherals::steal()`
    // (for the MPU/cache setup), which latches the global "taken" flag, so
    // take().unwrap() panicked → panic_halt spun forever (app froze at 0x25).
    // Steal again — single-threaded bare metal, we own the timing.
    let cp = unsafe { cortex_m::Peripherals::steal() };
    let mut delay = cp.SYST.delay(clocks);
    let adc1 = Adc::adc1(dp.ADC1, 4.MHz(), &mut delay, rec.ADC12, &clocks);
    let mut adc1 = adc1.enable(); // default 16-bit resolution
    unsafe { mark(0x26) }; // ADC ready

    // Freeze-free USB2 CDC-ACM (all fields pub → no CoreClocks/freeze needed).
    let usb = USB2 {
        usb_global: dp.OTG2_HS_GLOBAL,
        usb_device: dp.OTG2_HS_DEVICE,
        usb_pwrclk: dp.OTG2_HS_PWRCLK,
        prec: rec.USB2OTG,
        hclk: Hertz::from_raw(200_000_000),
    };
    let usb_bus = UsbBus::new(usb, unsafe { &mut *core::ptr::addr_of_mut!(EP_MEMORY) });
    unsafe { mark(0x27) }; // UsbBus::new done
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(VID, PID))
        .strings(&[StringDescriptors::default()
            .manufacturer("daisy-rs")
            .product("Daisy Hothouse Control Panel")
            .serial_number("HOTHOUSE-1")])
        .expect("string descriptors")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .max_packet_size_0(64)
        .expect("ep0 size")
        .build();

    unsafe { mark(0x28) }; // USB device built
    let mut ui = panel::Panel::new();
    unsafe { mark(0x29) }; // panel ready (heap init'd at run() entry)

    // Pace debounce (~1 kHz, like libDaisy) and redraw (~20 Hz) off DWT cycles,
    // not the loop rate. sys_ck came from the recovered CoreClocks.
    let cyc_per_ms = clocks.sys_ck().raw() / 1_000;
    let debounce_period = cyc_per_ms; // 1 ms
    let render_period = cyc_per_ms * 50; // 50 ms → 20 Hz
    let mut last_debounce = cortex_m::peripheral::DWT::cycle_count();
    let mut last_render = last_debounce;
    let mut heartbeat: u32 = 0;
    let mut last_dtr = false;

    unsafe { mark(0x2A) }; // loop entered
    loop {
        heartbeat = heartbeat.wrapping_add(1);
        unsafe {
            core::ptr::write_volatile(MARK_LOOP, heartbeat); // SWD-visible loop counter
            core::ptr::write_volatile(
                GPIOC_BSRR,
                if heartbeat & 0x8_0000 != 0 {
                    1 << 7
                } else {
                    1 << 23
                },
            );
        }

        // USB must be polled promptly; do it every pass.
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            if let Ok(count) = serial.read(&mut buf) {
                ui.on_input(&buf[..count]); // CPR size reply + keystrokes
            }
        }

        // Detect a terminal attaching (DTR false→true) and force a full repaint,
        // otherwise a client connecting after boot only receives empty diffs.
        let dtr = serial.dtr();
        if dtr && !last_dtr {
            ui.on_connect();
        }
        last_dtr = dtr;

        let now = cortex_m::peripheral::DWT::cycle_count();

        // Debounce the switches at ~1 kHz.
        if now.wrapping_sub(last_debounce) >= debounce_period {
            last_debounce = now;
            hothouse.switches.update();
        }

        // Redraw at ~20 Hz, but only once the previous frame has fully drained.
        if now.wrapping_sub(last_render) >= render_period && !ui.output_pending() {
            last_render = now;
            let controls = panel::Controls {
                knobs: hothouse.knobs.read_all(&mut adc1),
                toggles: [
                    hothouse.switches.toggle(Toggle::One),
                    hothouse.switches.toggle(Toggle::Two),
                    hothouse.switches.toggle(Toggle::Three),
                ],
                footswitches: [
                    hothouse.switches.footswitch_pressed(Footswitch::One),
                    hothouse.switches.footswitch_pressed(Footswitch::Two),
                ],
            };
            ui.render(&controls);
        }

        ui.drain_to(|bytes| serial.write(bytes).ok());
    }
}
