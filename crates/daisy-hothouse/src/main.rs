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
    use embedded_alloc::LlffHeap as Heap;

    #[global_allocator]
    static HEAP: Heap = Heap::empty();

    /// Initialise the heap. Call once before any allocation.
    pub fn init() {
        const HEAP_BASE: usize = 0x2400_0000;
        const HEAP_SIZE: usize = 256 * 1024;
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
    // Mint the peripheral clock tokens WITHOUT freezing (the bootloader already
    // configured the clock tree). Recover its frozen `CoreClocks` from Backup
    // SRAM — the ADC and the SysTick delay both need it.
    let rcc = dp.RCC.constrain();
    let rec = unsafe { rcc.steal_peripheral_rec() };
    let clocks = unsafe { daisy_bsp::clocks::handoff::restore() }
        .expect("CoreClocks hand-off from the bootloader");

    // Split every GPIO port the panel + USB touch (each enables its bus clock).
    let gpioa = dp.GPIOA.split(rec.GPIOA);
    let gpiob = dp.GPIOB.split(rec.GPIOB);
    let gpioc = dp.GPIOC.split(rec.GPIOC);
    let gpiod = dp.GPIOD.split(rec.GPIOD);
    let gpiog = dp.GPIOG.split(rec.GPIOG);

    // USB pins PA11/PA12 → AF10 (OTG2_HS FS).
    let _pin_dm = gpioa.pa11.into_alternate::<10>();
    let _pin_dp = gpioa.pa12.into_alternate::<10>();

    // Defensive: ensure the external USB 3.3 V detector is up (bootloader sets it).
    unsafe {
        let pwr = &*pac::PWR::ptr();
        pwr.cr3.modify(|_, w| w.usb33den().set_bit());
        while pwr.cr3.read().usb33rdy().bit_is_clear() {}
    }

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

    // ADC1 for the six knobs (all on ADC1). Uses SysTick for the power-up delay.
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = cp.SYST.delay(clocks);
    let adc1 = Adc::adc1(dp.ADC1, 4.MHz(), &mut delay, rec.ADC12, &clocks);
    let mut adc1 = adc1.enable(); // default 16-bit resolution

    // Freeze-free USB2 CDC-ACM (all fields pub → no CoreClocks/freeze needed).
    let usb = USB2 {
        usb_global: dp.OTG2_HS_GLOBAL,
        usb_device: dp.OTG2_HS_DEVICE,
        usb_pwrclk: dp.OTG2_HS_PWRCLK,
        prec: rec.USB2OTG,
        hclk: Hertz::from_raw(200_000_000),
    };
    let usb_bus = UsbBus::new(usb, unsafe { &mut *core::ptr::addr_of_mut!(EP_MEMORY) });
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

    heap::init();
    let mut ui = panel::Panel::new();

    // Pace debounce (~1 kHz, like libDaisy) and redraw (~20 Hz) off DWT cycles,
    // not the loop rate. sys_ck came from the recovered CoreClocks.
    let cyc_per_ms = clocks.sys_ck().raw() / 1_000;
    let debounce_period = cyc_per_ms; // 1 ms
    let render_period = cyc_per_ms * 50; // 50 ms → 20 Hz
    let mut last_debounce = cortex_m::peripheral::DWT::cycle_count();
    let mut last_render = last_debounce;
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

        // USB must be polled promptly; do it every pass.
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            if let Ok(count) = serial.read(&mut buf) {
                ui.on_input(&buf[..count]); // CPR size reply + keystrokes
            }
        }

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
