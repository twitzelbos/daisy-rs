#![no_std]
#![no_main]
// cortex-m-rt 0.7 deprecated `#[pre_init]`; migrate later. Silence at crate scope.
#![allow(deprecated)]

//! **SDRAM March memory-test app** for the Daisy Seed, linked to execute from
//! QSPI XIP at 0x9000_0000 (loaded via `daisy flash` through the bootloader).
//!
//! It brings up the external SDRAM (Alliance AS4C16M32SB-6BCN, 16M × 32-bit =
//! 64 MiB at 0xC000_0000) via [`daisy_bsp::sdram::init`] — the FIRST on-target
//! use of that bring-up, so it is treated as un-validated — then runs a memory
//! test suite over the full 64 MiB and streams progress + any failures over a
//! USB-CDC serial port. Open it with `picocom -b 115200 /dev/tty.usbmodem*`
//! (baud is ignored — it is USB).
//!
//! The SDRAM MPU region is mapped **non-cacheable** (unlike the app template,
//! which makes it write-back cacheable): a memory test MUST exercise the actual
//! DRAM cells and the FMC bus, not the L1 D-cache — a cached test would pass
//! even on dead DRAM. All accesses are `read_volatile`/`write_volatile`.
//!
//! Test suite (word-wise over 16 Mi 32-bit words):
//!   1. **own-address** — write each cell's own index, read all back. Catches
//!      address-decoder / FMC-mapping faults + aliasing (the most likely failure
//!      of a mis-configured controller) and stuck bits.
//!   2. **data patterns** — 0x0000_0000, 0xFFFF_FFFF, 0xAAAA_AAAA, 0x5555_5555.
//!      Catches data-line stuck-at / shorts.
//!   3. **March C−** — ⇑(w0); ⇑(r0,w1); ⇑(r1,w0); ⇓(r0,w1); ⇓(r1,w0); ⇓(r0).
//!      Classic coverage of SAF, TF, CF, AF.
//!
//! Like the app template, USB is brought up freeze-free (the bootloader already
//! froze the clocks) and timing uses DWT_CYCCNT. LED fault handlers stay from
//! the template so an SDRAM bus fault blinks a diagnostic instead of hanging.

use core::fmt::Write as _;
use core::mem::MaybeUninit;

use cortex_m_rt::{entry, exception, pre_init};
use daisy_bsp::hal;
use hal::pac;
use hal::prelude::*;
use hal::time::Hertz;
use hal::usb_hs::{UsbBus, USB2};
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

// DWT_CYCCNT ticks at the bootloader's 400 MHz sysclk, so 1 ms = 400_000 cycles.
const CYCLES_PER_MS: u32 = 400_000;

// Peripheral MMIO (raw, for #[pre_init] + the LED).
const GPIOC_MODER: *mut u32 = 0x5802_0800 as *mut u32;
const GPIOC_BSRR: *mut u32 = 0x5802_0818 as *mut u32;
const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;
const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
const DWT_CYCCNT: *const u32 = 0xE000_1004 as *const u32;
const CFSR: *const u32 = 0xE000_ED28 as *const u32;
const SHCSR: *mut u32 = 0xE000_ED24 as *mut u32;
const MMFAR: *const u32 = 0xE000_ED34 as *const u32;
const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32 = 0xE000_EDA0 as *mut u32;

#[inline(always)]
fn dwt() -> u32 {
    unsafe { core::ptr::read_volatile(DWT_CYCCNT) }
}

/// Block for `ms` milliseconds using DWT_CYCCNT, bounded so it can't hang if
/// the counter is stuck.
#[inline(always)]
unsafe fn delay_ms(ms: u32) {
    const BURN: u32 = 4096;
    let n = ms.saturating_mul(CYCLES_PER_MS);
    let start = dwt();
    let max_iters = (n / BURN).saturating_add(64).saturating_mul(3);
    let mut iters = 0u32;
    while dwt().wrapping_sub(start) < n {
        if iters >= max_iters {
            return;
        }
        iters = iters.wrapping_add(1);
        cortex_m::asm::delay(BURN);
    }
}

/// Block for at least `us` microseconds (400 cycles/µs at 400 MHz). Bounded.
#[inline(always)]
unsafe fn delay_us(us: u32) {
    let n = us.saturating_mul(CYCLES_PER_MS / 1000);
    let start = dwt();
    let mut iters = 0u32;
    while dwt().wrapping_sub(start) < n {
        if iters >= n.saturating_add(1_000_000) {
            return;
        }
        iters = iters.wrapping_add(1);
        cortex_m::asm::nop();
    }
}

#[inline(always)]
unsafe fn led_output() {
    let mut m = core::ptr::read_volatile(GPIOC_MODER);
    m &= !(0b11 << 14);
    m |= 0b01 << 14;
    core::ptr::write_volatile(GPIOC_MODER, m);
}

#[inline(always)]
unsafe fn led_set(on: bool) {
    core::ptr::write_volatile(GPIOC_BSRR, if on { 1 << 7 } else { 1 << 23 });
}

#[inline(always)]
unsafe fn enable_dwt() {
    core::ptr::write_volatile(DEMCR, core::ptr::read_volatile(DEMCR) | (1 << 24));
    core::ptr::write_volatile(DWT_CTRL, core::ptr::read_volatile(DWT_CTRL) | 1);
}

/// Program one MPU region (see the app template for the field layout).
#[inline(always)]
#[allow(clippy::too_many_arguments)]
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

/// MPU (libDaisy-style regions) + L1 caches. IDENTICAL to the app template
/// EXCEPT region 1 (SDRAM) is **non-cacheable** here so the memory test hits
/// the DRAM cells / FMC bus rather than the D-cache.
unsafe fn configure_mpu_and_caches() {
    core::ptr::write_volatile(MPU_CTRL, 0);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Region 0: SRAM_D2 DMA pool — non-cacheable, shareable.
    mpu_region(0, 0x3000_0000, 15, 1, 1, 0, 0, 0);
    // Region 1: SDRAM (64 MiB) — NON-cacheable, shareable (TEX=1,C=0,B=0).
    // The memory test must exercise real DRAM, not write-back cache.
    mpu_region(1, 0xC000_0000, 26, 1, 1, 0, 0, 0);
    // Region 2: Backup SRAM — non-cacheable.
    mpu_region(2, 0x3880_0000, 12, 1, 1, 0, 0, 0);
    // Region 3: QSPI XIP flash (8 MiB) — write-through cacheable + exec.
    mpu_region(3, 0x9000_0000, 23, 0, 0, 1, 0, 0);

    core::ptr::write_volatile(MPU_CTRL, (1 << 0) | (1 << 2)); // ENABLE | PRIVDEFENA
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    let mut cp = cortex_m::Peripherals::steal();
    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
}

#[inline(always)]
unsafe fn enable_memfault() {
    core::ptr::write_volatile(SHCSR, core::ptr::read_volatile(SHCSR) | (1 << 16));
}

/// `n` pulses of `on_ms`/`off_ms`, then `gap_ms` dark. Loops forever.
unsafe fn led_pattern_forever(n: u32, on_ms: u32, off_ms: u32, gap_ms: u32) -> ! {
    enable_dwt();
    led_output();
    loop {
        for _ in 0..n {
            led_set(true);
            delay_ms(on_ms);
            led_set(false);
            delay_ms(off_ms);
        }
        delay_ms(gap_ms);
    }
}

// Panic: TRIPLE-BURST — 3 fast blinks + 1 s gap.
#[panic_handler]
fn on_panic(_info: &core::panic::PanicInfo) -> ! {
    unsafe { led_pattern_forever(3, 100, 100, 1000) };
}

// HardFault: decode CFSR into a pulse count (a mis-configured FMC makes an SDRAM
// access a PRECISERR/IMPRECISERR bus fault → 3 or 4 pulses).
#[exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    let cfsr = core::ptr::read_volatile(CFSR);
    let n: u32 = if (cfsr & (1 << 10)) != 0 {
        4 // IMPRECISERR — async data bus error (buffered store)
    } else if (cfsr & (1 << 9)) != 0 {
        3 // PRECISERR — synchronous data bus error
    } else if (cfsr & (1 << 8)) != 0 {
        2 // IBUSERR — instruction fetch bus error
    } else if (cfsr & (1 << 12)) != 0 {
        1 // STKERR — stack push failed
    } else {
        5
    };
    led_pattern_forever(n, 400, 400, 1500);
}

// MemManage: decode MMFSR (low byte of CFSR); distinct 250 ms cadence.
#[exception]
unsafe fn MemoryManagement() -> ! {
    let mmfsr = core::ptr::read_volatile(CFSR) & 0xFF;
    let _fault_addr = core::ptr::read_volatile(MMFAR);
    let n: u32 = if (mmfsr & (1 << 7)) != 0 {
        4
    } else if (mmfsr & ((1 << 3) | (1 << 4))) != 0 {
        3
    } else if (mmfsr & (1 << 1)) != 0 {
        2
    } else if (mmfsr & (1 << 0)) != 0 {
        1
    } else {
        5
    };
    led_pattern_forever(n, 250, 250, 2000);
}

#[pre_init]
unsafe fn pre_init() {
    configure_mpu_and_caches();
    enable_memfault();
    enable_dwt();
    led_output();
}

// --- USB (freeze-free, mirrors daisy-usb-audio) --------------------------
// OTG2_HS (FS mode) 4 KiB FIFO RAM in DTCM (never cached).
static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static mut USB_ALLOC: MaybeUninit<UsbBusAllocator<UsbBus<USB2>>> = MaybeUninit::uninit();
const VID: u16 = 0x1209;
const PID: u16 = 0xDA57;

/// USB-CDC console: a `core::fmt::Write` sink that services USB while it writes,
/// so control transfers keep flowing during a multi-second test phase.
struct Console {
    dev: UsbDevice<'static, UsbBus<USB2>>,
    serial: SerialPort<'static, UsbBus<USB2>>,
}

impl Console {
    #[inline(always)]
    fn poll(&mut self) -> bool {
        self.dev.poll(&mut [&mut self.serial])
    }

    fn dtr(&self) -> bool {
        self.serial.dtr()
    }

    /// Write all bytes, polling USB between attempts. Bounded so a host that
    /// stopped reading drops the remainder instead of hanging the test.
    fn write_bytes(&mut self, mut data: &[u8]) {
        let mut stall = 0u32;
        while !data.is_empty() {
            self.poll();
            match self.serial.write(data) {
                Ok(n) if n > 0 => {
                    data = &data[n..];
                    stall = 0;
                }
                _ => {
                    stall = stall.wrapping_add(1);
                    if stall > 200_000 {
                        break; // host not reading — drop the rest
                    }
                }
            }
        }
        let _ = self.serial.flush();
    }
}

impl core::fmt::Write for Console {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // Translate '\n' → "\r\n" so terminals (picocom) don't staircase.
        for part in s.split_inclusive('\n') {
            if let Some(line) = part.strip_suffix('\n') {
                self.write_bytes(line.as_bytes());
                self.write_bytes(b"\r\n");
            } else {
                self.write_bytes(part.as_bytes());
            }
        }
        Ok(())
    }
}

// --- SDRAM under test ----------------------------------------------------
const SDRAM_BASE: usize = daisy_bsp::sdram::config::BASE_ADDRESS as usize; // 0xC000_0000
const SDRAM_SIZE: usize = daisy_bsp::sdram::config::SIZE_BYTES as usize; // 64 MiB
const SDRAM_WORDS: usize = SDRAM_SIZE / 4; // 16 Mi words
const CHUNK: usize = 1 << 16; // words per USB-service interval (256 KiB)
const MAX_REPORT: u32 = 32; // cap failure lines per pass

#[inline(always)]
fn wr(i: usize, v: u32) {
    unsafe { core::ptr::write_volatile((SDRAM_BASE + i * 4) as *mut u32, v) }
}
#[inline(always)]
fn rd(i: usize) -> u32 {
    unsafe { core::ptr::read_volatile((SDRAM_BASE + i * 4) as *const u32) }
}

/// Run one chunked pass over the whole array (ascending or descending),
/// applying `op` to each word index; `op` returns `Err((expected, got))` on a
/// read mismatch. USB is serviced between chunks; up to `MAX_REPORT` failures
/// are printed. Returns the error count. Prints a per-pass result+timing line.
fn march_pass<F>(con: &mut Console, ascending: bool, label: &str, mut op: F) -> u32
where
    F: FnMut(usize) -> Result<(), (u32, u32)>,
{
    let t0 = dwt();
    let mut errs = 0u32;
    let mut done = 0usize;
    while done < SDRAM_WORDS {
        let n = (SDRAM_WORDS - done).min(CHUNK);
        for k in 0..n {
            let j = if ascending {
                done + k
            } else {
                SDRAM_WORDS - 1 - (done + k)
            };
            if let Err((exp, got)) = op(j) {
                errs += 1;
                if errs <= MAX_REPORT {
                    let _ = writeln!(
                        con,
                        "    FAIL @0x{:08X}: exp 0x{:08X} got 0x{:08X}",
                        SDRAM_BASE + j * 4,
                        exp,
                        got
                    );
                }
            }
        }
        con.poll();
        done += n;
    }
    let ms = dwt().wrapping_sub(t0) / CYCLES_PER_MS;
    let _ = writeln!(con, "  {label}: {errs} err ({ms} ms)");
    errs
}

fn test_address(con: &mut Console) -> u32 {
    let _ = writeln!(con, "[1] own-address (write index, read back)");
    march_pass(con, true, "write", |j| {
        wr(j, j as u32);
        Ok(())
    });
    march_pass(con, true, "verify", |j| {
        let g = rd(j);
        if g != j as u32 {
            Err((j as u32, g))
        } else {
            Ok(())
        }
    })
}

fn test_pattern(con: &mut Console, name: &str, pat: u32) -> u32 {
    let _ = writeln!(con, "[2] pattern {name} (0x{pat:08X})");
    march_pass(con, true, "write", |j| {
        wr(j, pat);
        Ok(())
    });
    march_pass(con, true, "verify", |j| {
        let g = rd(j);
        if g != pat {
            Err((pat, g))
        } else {
            Ok(())
        }
    })
}

fn test_march(con: &mut Console) -> u32 {
    let _ = writeln!(con, "[3] March C- (w0; r0w1; r1w0; -r0w1; -r1w0; -r0)");
    let mut e = 0;
    e += march_pass(con, true, "M0 up w0", |j| {
        wr(j, 0);
        Ok(())
    });
    e += march_pass(con, true, "M1 up r0w1", |j| {
        let g = rd(j);
        wr(j, !0);
        if g != 0 {
            Err((0, g))
        } else {
            Ok(())
        }
    });
    e += march_pass(con, true, "M2 up r1w0", |j| {
        let g = rd(j);
        wr(j, 0);
        if g != !0 {
            Err((!0, g))
        } else {
            Ok(())
        }
    });
    e += march_pass(con, false, "M3 dn r0w1", |j| {
        let g = rd(j);
        wr(j, !0);
        if g != 0 {
            Err((0, g))
        } else {
            Ok(())
        }
    });
    e += march_pass(con, false, "M4 dn r1w0", |j| {
        let g = rd(j);
        wr(j, 0);
        if g != !0 {
            Err((!0, g))
        } else {
            Ok(())
        }
    });
    e += march_pass(con, false, "M5 dn r0", |j| {
        let g = rd(j);
        if g != 0 {
            Err((0, g))
        } else {
            Ok(())
        }
    });
    e
}

#[entry]
fn main() -> ! {
    unsafe {
        enable_dwt();
        led_output();
    }
    let dp = pac::Peripherals::take().unwrap();

    // USB2 (OTG2_HS FS) freeze-free: the bootloader already froze the clocks and
    // left HSI48 + USB33DEN up. Mint the peripheral rec via `steal`, build the
    // all-pub USB2 struct with a hard-coded hclk (used only for a >30 MHz check).
    let rcc = dp.RCC.constrain();
    let rec = unsafe { rcc.steal_peripheral_rec() };
    let gpioa = dp.GPIOA.split(rec.GPIOA);
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
        prec: rec.USB2OTG,
        hclk: Hertz::from_raw(200_000_000),
    };
    let alloc: &'static UsbBusAllocator<UsbBus<USB2>> = unsafe {
        let p = core::ptr::addr_of_mut!(USB_ALLOC);
        (*p).write(UsbBus::new(usb, &mut *core::ptr::addr_of_mut!(EP_MEMORY)));
        (*p).assume_init_ref()
    };
    let serial = SerialPort::new(alloc);
    let dev = UsbDeviceBuilder::new(alloc, UsbVidPid(VID, PID))
        .device_class(USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer("daisy-rs")
            .product("Daisy SDRAM March Test")
            .serial_number("DAISY-SDRAM-1")])
        .expect("string descriptors")
        .max_packet_size_0(64)
        .expect("ep0 size")
        .build();
    let mut con = Console { dev, serial };

    // Wait for the host to enumerate and open the port (DTR), or press a key, or
    // a coarse fail-safe. Blink slowly (~2 Hz) while waiting.
    let mut waited = 0u32;
    loop {
        con.poll();
        if con.dtr() {
            break;
        }
        let mut b = [0u8; 8];
        if con.serial.read(&mut b).unwrap_or(0) > 0 {
            break;
        }
        waited = waited.wrapping_add(1);
        unsafe { led_set(waited & 0x0080_0000 != 0) };
        if waited > 60_000_000 {
            break; // fail-safe (~tens of seconds) — run even if never opened
        }
    }
    unsafe {
        led_set(true);
        delay_ms(150);
    }

    let _ = writeln!(con, "\r\n=== Daisy SDRAM March test ===");
    let _ = writeln!(
        con,
        "target: 64 MiB @ 0x{SDRAM_BASE:08X} ({SDRAM_WORDS} words), non-cacheable"
    );
    let _ = writeln!(con, "bringing up SDRAM (FMC + JEDEC power-up)...");
    // SAFETY: we own all the FMC/GPIO/RCC-PLL2 peripherals here; the bootloader
    // configured PLL2R which sdram::init selects as the FMC kernel clock.
    unsafe { daisy_bsp::sdram::init(|us| delay_us(us)) };
    let _ = writeln!(
        con,
        "SDRAM up. running test suite (this takes ~1 minute)...\r\n"
    );

    let mut errors = 0u32;
    errors += test_address(&mut con);
    errors += test_pattern(&mut con, "zeros", 0x0000_0000);
    errors += test_pattern(&mut con, "ones", 0xFFFF_FFFF);
    errors += test_pattern(&mut con, "AAAA", 0xAAAA_AAAA);
    errors += test_pattern(&mut con, "5555", 0x5555_5555);
    errors += test_march(&mut con);

    let _ = writeln!(
        con,
        "\r\n=== {} : {errors} total error(s) ===",
        if errors == 0 { "PASS" } else { "FAIL" }
    );

    // Idle: LED steady ON = PASS, fast blink = FAIL. Keep servicing USB so the
    // report stays readable and the port stays enumerated.
    let mut tick = 0u32;
    loop {
        con.poll();
        tick = tick.wrapping_add(1);
        unsafe {
            if errors == 0 {
                led_set(true);
            } else {
                led_set(tick & 0x0010_0000 != 0);
            }
        }
    }
}
