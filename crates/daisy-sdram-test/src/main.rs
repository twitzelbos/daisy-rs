#![no_std]
#![no_main]
// cortex-m-rt 0.7 deprecated `#[pre_init]`; migrate later. Silence at crate scope.
#![allow(deprecated)]

//! **SDRAM March memory-test app** for the Daisy Seed, linked to execute from
//! QSPI XIP at 0x9000_0000 (loaded via `daisy flash` through the bootloader).
//!
//! It brings up the external SDRAM (Alliance AS4C16M32SB-6BCN, 16M × 32-bit =
//! 64 MiB at 0xC000_0000) via [`daisy_bsp::sdram::init`], then runs a memory
//! test suite ([`sdram_march`]) continuously over the full 64 MiB and renders a
//! live memtest86-style [`Tui`] over USB-CDC. Open it with
//! `picocom -b 115200 /dev/tty.usbmodem*` (baud is ignored — it is USB).
//!
//! The SDRAM MPU region is mapped **non-cacheable** (unlike the app template,
//! which makes it write-back cacheable): a memory test MUST exercise the actual
//! DRAM cells and the FMC bus, not the L1 D-cache — a cached test would pass
//! even on dead DRAM. All accesses are `read_volatile`/`write_volatile`.
//!
//! Test suite (word-wise over 16 Mi 32-bit words) — see [`sdram_march`]:
//! own-address, four data patterns, then March C−. The suite definition + the
//! generic runner are host-tested in that crate; here we supply the volatile
//! DRAM access and drive the TUI between chunks.
//!
//! Like the app template, USB is brought up freeze-free (the bootloader already
//! froze the clocks) and timing uses DWT_CYCCNT. LED fault handlers stay from
//! the template so an SDRAM bus fault blinks a diagnostic instead of hanging.

extern crate alloc;

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

use sdram_march::{run_phase, NGROUPS, SUITE};

mod tui;
use tui::{Status, TestState, Tui};

// ratatui needs a global allocator. Placed in AXI SRAM (0x2400_0000, 512 KiB),
// which our linker script leaves unused — so the whole region is the heap's.
mod heap {
    use embedded_alloc::LlffHeap as Heap;
    #[global_allocator]
    static HEAP: Heap = Heap::empty();
    /// Initialise the heap. Call once before any allocation.
    pub fn init() {
        const AXI_SRAM_BASE: usize = 0x2400_0000;
        const HEAP_SIZE: usize = 480 * 1024;
        // SAFETY: AXI SRAM is powered and unused by the linker script.
        unsafe { HEAP.init(AXI_SRAM_BASE, HEAP_SIZE) };
    }
}

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

// Panic: record the panic site to DTCM markers (SWD-readable — DTCM is never
// cached, so a debugger sees the writes immediately), then TRIPLE-BURST the LED
// (3 fast blinks + 1 s gap). Markers at 0x2001_0020, well below the stack top
// (0x2002_0000; stack depth is < 16 KiB) so they don't collide.
#[panic_handler]
fn on_panic(info: &core::panic::PanicInfo) -> ! {
    unsafe {
        let m = 0x2001_0020 as *mut u32;
        core::ptr::write_volatile(m, 0x5041_4E43); // "PANC" magic
        if let Some(loc) = info.location() {
            core::ptr::write_volatile(m.add(1), loc.line());
            // Up to 48 bytes of the panicking file's path, as raw bytes.
            let f = loc.file().as_bytes();
            let base = 0x2001_0030 as *mut u8;
            let n = f.len().min(48);
            let mut i = 0;
            while i < n {
                core::ptr::write_volatile(base.add(i), f[i]);
                i += 1;
            }
            while i < 48 {
                core::ptr::write_volatile(base.add(i), 0);
                i += 1;
            }
        }
        cortex_m::asm::dsb();
        led_pattern_forever(3, 100, 100, 1000)
    };
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

/// USB-CDC holder: the device + serial class, with the poll/read/write helpers
/// the test loop uses to keep control transfers flowing during a long pass.
struct Usb {
    dev: UsbDevice<'static, UsbBus<USB2>>,
    serial: SerialPort<'static, UsbBus<USB2>>,
}

impl Usb {
    #[inline(always)]
    fn poll(&mut self) -> bool {
        self.dev.poll(&mut [&mut self.serial])
    }
    fn dtr(&self) -> bool {
        self.serial.dtr()
    }
    fn read(&mut self, buf: &mut [u8]) -> usize {
        self.serial.read(buf).unwrap_or(0)
    }
    /// Write as much of `data` as the endpoint accepts right now; returns the
    /// count (the TUI's `drain_to` re-offers the remainder next tick).
    fn write(&mut self, data: &[u8]) -> Option<usize> {
        self.serial.write(data).ok()
    }
}

// --- SDRAM under test ----------------------------------------------------
const SDRAM_BASE: usize = daisy_bsp::sdram::config::BASE_ADDRESS as usize; // 0xC000_0000
const SDRAM_SIZE: usize = daisy_bsp::sdram::config::SIZE_BYTES as usize; // 64 MiB
const SDRAM_WORDS: usize = SDRAM_SIZE / 4; // 16 Mi words
const CHUNK: usize = 1 << 16; // words per USB-service / repaint interval (256 KiB)

#[inline(always)]
fn wr(i: usize, v: u32) {
    unsafe { core::ptr::write_volatile((SDRAM_BASE + i * 4) as *mut u32, v) }
}
#[inline(always)]
fn rd(i: usize) -> u32 {
    unsafe { core::ptr::read_volatile((SDRAM_BASE + i * 4) as *const u32) }
}

/// Control keys detected in the host's input this tick.
#[derive(Clone, Copy, Default)]
struct Keys {
    /// SPACE — start / pause / resume (context-dependent).
    toggle: bool,
    /// `s` — stop.
    stop: bool,
}

/// Service USB + repaint the TUI once (called between chunks). Feeds any RX
/// bytes to the TUI (CPR size reply), scans them for control keys, renders a
/// frame if none is pending, and pushes pending ANSI to the host.
fn poll_ui(usb: &mut Usb, ui: &mut Tui, state: &TestState) -> Keys {
    usb.poll();
    let mut rx = [0u8; 16];
    let n = usb.read(&mut rx);
    let mut keys = Keys::default();
    if n > 0 {
        ui.on_input(&rx[..n]); // CPR size reply
        for &b in &rx[..n] {
            match b {
                b' ' => keys.toggle = true,
                b's' | b'S' => keys.stop = true,
                // Reboot keys — lowercase only: a CPR reply ends in uppercase
                // `R`, so `r`/`R` as a command would fire on every size query.
                b'r' => daisy_bsp::reset::reboot(), // never returns
                b'b' => daisy_bsp::reset::reboot_to_bootloader(), // → DFU; never returns
                _ => {}
            }
        }
    }
    if !ui.output_pending() {
        ui.render(state);
    }
    ui.drain_to(|b| usb.write(b));
    usb.poll();
    keys
}

/// Drives the [`sdram_march`] suite against the real SDRAM while keeping the USB
/// TUI live: `read`/`write` hit the DRAM, `on_error` feeds the error log, and
/// `service` (every chunk) updates progress, repaints, and handles the
/// pause/stop keys — returning `false` to abort the phase on a stop request.
struct Runner<'a> {
    usb: &'a mut Usb,
    ui: &'a mut Tui,
    state: &'a mut TestState,
    ascending: bool,
    /// Cumulative words processed this run + the per-phase mark, for throughput.
    processed: u64,
    last_done: usize,
    /// Cycle count at run start, for elapsed time.
    start_cyc: u32,
    /// Set when the user pressed stop; the caller ends the run.
    stopped: bool,
}

impl Runner<'_> {
    fn begin_phase(&mut self) {
        self.last_done = 0;
    }

    /// Update elapsed / throughput / progress / address from `done`.
    fn update_stats(&mut self, done: usize) {
        self.processed += (done - self.last_done) as u64;
        self.last_done = done;
        let elapsed_cyc = dwt().wrapping_sub(self.start_cyc);
        let elapsed_ms = elapsed_cyc / CYCLES_PER_MS;
        self.state.elapsed_s = elapsed_ms / 1000;
        if elapsed_ms > 0 {
            // bytes / elapsed_ms = bytes per ms = KB/s; ÷1000 → MB/s.
            let bytes = self.processed.saturating_mul(4);
            self.state.throughput_mb_s = (bytes / elapsed_ms as u64 / 1000) as u32;
        }
        self.state.phase_pct = (done as u64 * 100 / SDRAM_WORDS as u64) as u16;
        let word = if self.ascending {
            done.min(SDRAM_WORDS - 1)
        } else {
            SDRAM_WORDS.saturating_sub(done)
        };
        self.state.addr = (SDRAM_BASE + word * 4) as u32;
    }
}

impl sdram_march::Harness for Runner<'_> {
    #[inline(always)]
    fn read(&mut self, index: usize) -> u32 {
        rd(index)
    }
    #[inline(always)]
    fn write(&mut self, index: usize, value: u32) {
        wr(index, value);
    }
    fn on_error(&mut self, index: usize, expected: u32, got: u32) {
        let group = self.state.cur_group;
        self.state.group_errs[group] = self.state.group_errs[group].saturating_add(1);
        self.state
            .errlog
            .push((SDRAM_BASE + index * 4) as u32, expected, got);
    }
    fn service(&mut self, done: usize) -> bool {
        self.update_stats(done);
        // One service tick, plus a pause spin: keep polling (which repaints and
        // reads keys) until resumed or stopped.
        loop {
            let keys = poll_ui(self.usb, self.ui, self.state);
            if keys.stop {
                self.stopped = true;
                return false; // abort the phase
            }
            if keys.toggle {
                if self.state.status == Status::Paused {
                    self.state.status = Status::Running;
                } else {
                    self.state.status = Status::Paused;
                }
            }
            if self.state.status != Status::Paused {
                return true;
            }
        }
    }
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
    let mut usb = Usb { dev, serial };
    heap::init(); // ratatui allocates; must precede Tui::new
    let mut ui = Tui::new();
    let mut state = TestState::new(SDRAM_WORDS, 100); // SDCLK 100 MHz

    // Wait for a terminal to open the port (DTR), rendering the waiting screen.
    // Fail-safe after ~tens of seconds so it runs even headless.
    state.status = Status::Waiting;
    let mut waited = 0u32;
    let mut dtr_prev = false;
    loop {
        let _ = poll_ui(&mut usb, &mut ui, &state);
        let dtr = usb.dtr();
        if dtr && !dtr_prev {
            ui.on_connect();
        }
        dtr_prev = dtr;
        if dtr {
            break;
        }
        waited = waited.wrapping_add(1);
        unsafe { led_set(waited & 0x0080_0000 != 0) };
        if waited > 60_000_000 {
            break; // fail-safe — run even if never opened
        }
    }
    // Mop up any banner the client printed after our attach-time clear.
    for _ in 0..3 {
        ui.repaint();
        let _ = poll_ui(&mut usb, &mut ui, &state);
        unsafe { delay_ms(200) };
    }

    // Bring up the SDRAM (FMC + JEDEC power-up). SAFETY: we own the FMC/GPIO/
    // RCC-PLL2 peripherals; the bootloader configured PLL2R, which sdram::init
    // selects as the FMC kernel clock.
    state.status = Status::BringUp;
    for _ in 0..4 {
        let _ = poll_ui(&mut usb, &mut ui, &state);
    }
    unsafe { daisy_bsp::sdram::init(|us| delay_us(us)) };

    // Control loop. The suite runs continuously; SPACE pauses/resumes, `s` stops
    // (→ idle), and SPACE from idle restarts. The LED shows the live verdict:
    // steady = no errors, blinking = errors seen.
    let mut running = true;
    let mut pass = 0u32;
    loop {
        if !running {
            state.status = Status::Stopped;
            let keys = poll_ui(&mut usb, &mut ui, &state);
            unsafe { led_set(state.total_errs() == 0) };
            if keys.toggle {
                running = true;
                pass = 0;
                state.group_errs = [0; NGROUPS];
                state.errlog = tui::ErrLog::new();
            }
            continue;
        }

        pass = pass.wrapping_add(1);
        state.pass = pass;
        state.status = Status::Running;
        let mut runner = Runner {
            usb: &mut usb,
            ui: &mut ui,
            state: &mut state,
            ascending: true,
            processed: 0,
            last_done: 0,
            start_cyc: dwt(),
            stopped: false,
        };
        for phase in SUITE {
            runner.state.cur_group = phase.group;
            runner.state.phase_label = phase.label;
            runner.ascending = phase.ascending;
            runner.begin_phase();
            run_phase(phase, SDRAM_WORDS, CHUNK, &mut runner);
            if runner.stopped {
                break;
            }
        }
        let stopped = runner.stopped;
        unsafe { led_set(state.total_errs() == 0) };
        if stopped {
            running = false;
        }
    }
}
