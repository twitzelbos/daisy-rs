#![no_std]
#![no_main]
// cortex-m-rt 0.7.7 deprecated the `#[pre_init]` attribute macro in
// favour of a `global_asm!`-defined `__pre_init`. Migrate later; for
// now silence at crate scope so `daisy flash` is warning-free.
#![allow(deprecated)]

//! Template Daisy Seed application. Linked to execute from QSPI XIP at
//! 0x9000_0000 after the bootloader has configured OCTOSPI memory-mapped
//! mode.
//!
//! For now this deliberately does NOT call `clocks::init` — the
//! bootloader already ran the full clock config and set 400 MHz sysclk.
//! Re-running the HAL's PWR/RCC freeze on top of an already-frozen
//! state has been observed to hang the app on some silicon revs. All
//! timing here uses the DWT cycle counter (assumed to be enabled and
//! ticking at 400 MHz from the bootloader), converted to milliseconds
//! via a fixed CYCLES_PER_MS constant. That's precise regardless of
//! dual-issue behaviour or QSPI XIP fetch bandwidth — plain register
//! polling. __pre_init also (idempotently) enables DWT so the app can
//! also run standalone (e.g. under Renode's `app_standalone.robot`).

use cortex_m_rt::{entry, exception, pre_init};
// Pulls in stm32h7xx-hal's PAC, which supplies the interrupt vector
// table cortex-m-rt links against.
use daisy_bsp::hal as _;

// The bootloader's `daisy_bsp::clocks::init` sets sysclk = 400 MHz.
// DWT_CYCCNT ticks at sysclk, so 1 ms = 400_000 cycles.
const CYCLES_PER_MS: u32 = 400_000;

// Peripheral MMIO addresses.
const GPIOC_MODER: *mut u32 = 0x5802_0800 as *mut u32;
const GPIOC_BSRR: *mut u32 = 0x5802_0818 as *mut u32;

// Cortex-M debug + DWT registers.
const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;
const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
const DWT_CYCCNT: *const u32 = 0xE000_1004 as *const u32;

// SCB fault status register (RM0433 Cortex-M7 arch ref). MMFSR is its low
// byte (MemManage), the next byte is BFSR (BusFault).
const CFSR: *const u32 = 0xE000_ED28 as *const u32;
// SCB System Handler Control and State: MEMFAULTENA is bit 16.
const SHCSR: *mut u32 = 0xE000_ED24 as *mut u32;
// MemManage Fault Address Register — valid only when MMFSR.MMARVALID (bit 7).
const MMFAR: *const u32 = 0xE000_ED34 as *const u32;

// Cortex-M7 MPU (PM0253 / RM0433 arch ref).
const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32 = 0xE000_EDA0 as *mut u32;

/// Block for `ms` milliseconds. Uses DWT_CYCCNT for a precise EXIT check
/// and `cortex_m::asm::delay(BURN)` as the cycle-burning mechanism between
/// DWT reads (~1000× fewer MMIO polls than a tight loop — Renode
/// simulation stays fast). Hard cap on iterations so the delay can't hang
/// even if DWT_CYCCNT is stuck.
#[inline(always)]
unsafe fn delay_ms(ms: u32) {
    const BURN: u32 = 4096;
    let n = ms * CYCLES_PER_MS;
    let start = core::ptr::read_volatile(DWT_CYCCNT);
    let max_iters = (n / BURN).saturating_add(64).saturating_mul(3);
    let mut iters = 0u32;
    loop {
        if core::ptr::read_volatile(DWT_CYCCNT).wrapping_sub(start) >= n {
            return;
        }
        if iters >= max_iters {
            return;
        }
        iters = iters.wrapping_add(1);
        cortex_m::asm::delay(BURN);
    }
}

/// Configure PC7 as GPIO output. Idempotent.
#[inline(always)]
unsafe fn led_output() {
    let mut m = core::ptr::read_volatile(GPIOC_MODER);
    m &= !(0b11 << 14);
    m |= 0b01 << 14;
    core::ptr::write_volatile(GPIOC_MODER, m);
}

/// Idempotently enable DWT_CYCCNT.
#[inline(always)]
unsafe fn enable_dwt() {
    core::ptr::write_volatile(DEMCR, core::ptr::read_volatile(DEMCR) | (1 << 24));
    core::ptr::write_volatile(DWT_CTRL, core::ptr::read_volatile(DWT_CTRL) | 1);
}

/// Program one MPU region. `log2_bytes` = log2(size in bytes); the region
/// covers 2^log2_bytes bytes (RASR.SIZE = log2_bytes - 1). AP is fixed to
/// full access (0b011). `tex`/`s`/`c`/`b` are the ARMv7-M memory attributes;
/// `xn` = execute-never. Bases must be size-aligned.
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
    let rasr = 1 // ENABLE
        | ((log2_bytes - 1) << 1) // SIZE
        | (b << 16)
        | (c << 17)
        | (s << 18)
        | (tex << 19)
        | (0b011 << 24) // AP = full access
        | (xn << 28);
    core::ptr::write_volatile(MPU_RASR, rasr);
}

/// Configure the MPU to mirror Electrosmith libDaisy's `System::ConfigureMpu`,
/// then enable the L1 I- and D-caches. MPU-before-cache ordering is mandatory.
///
/// Only regions that differ from the ARMv7-M default map are programmed;
/// PRIVDEFENA keeps the default map for everything else (code, QSPI XIP,
/// peripherals, DTCM, AXI SRAM):
///   - SRAM_D2 @0x30000000 (32 KB): non-cacheable, shareable — DMA buffer
///     pool, keeps CPU/DMA coherent without cache maintenance.
///   - SDRAM   @0xC0000000 (64 MB): normal write-back cacheable — overrides
///     the default Device/XN attribute so external RAM is usable and fast.
///   - Backup SRAM @0x38800000 (4 KB): non-cacheable.
///
/// NOTE: enabling caches changes real-silicon timing and introduces
/// cache-coherency considerations that Renode cannot model — this must be
/// validated on hardware (probe-rs). With the I-cache on, XIP instruction
/// fetches from QSPI hit cache instead of serializing over the SPI bus.
unsafe fn configure_mpu_and_caches() {
    core::ptr::write_volatile(MPU_CTRL, 0); // disable while programming
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Region 0: SRAM_D2 DMA pool — non-cacheable (TEX=1,C=0,B=0), shareable.
    mpu_region(0, 0x3000_0000, 15, 1, 1, 0, 0, 0);
    // Region 1: SDRAM — normal write-back cacheable (TEX=0,C=1,B=1).
    mpu_region(1, 0xC000_0000, 26, 0, 0, 1, 1, 0);
    // Region 2: Backup SRAM — non-cacheable.
    mpu_region(2, 0x3880_0000, 12, 1, 1, 0, 0, 0);
    // Region 3: QSPI XIP flash (8 MB) — Normal write-through cacheable + exec
    // (TEX=0,C=1,B=0). Explicit so XIP code is I/D-cached by intent rather than
    // via the default map. Safe with the D-cache on because DMA buffers live in
    // the non-cacheable SRAM_D2 region above (matches libDaisy's cache setup).
    mpu_region(3, 0x9000_0000, 23, 0, 0, 1, 0, 0);

    // ENABLE | PRIVDEFENA — keep the default background map for privileged
    // accesses so DTCM/AXI SRAM/peripherals continue to work.
    core::ptr::write_volatile(MPU_CTRL, (1 << 0) | (1 << 2));
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Caches AFTER the MPU. cortex-m performs the correct invalidate sequence.
    let mut cp = cortex_m::Peripherals::steal();
    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
}

/// Enable the MemManage (memfault) exception so MPU violations trap to the
/// `MemoryManagement` handler instead of escalating to HardFault. BusFault
/// and UsageFault are deliberately left to escalate into the CFSR-decoding
/// HardFault handler.
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
            core::ptr::write_volatile(GPIOC_BSRR, 1 << 7);
            delay_ms(on_ms);
            core::ptr::write_volatile(GPIOC_BSRR, 1 << 23);
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

// HardFault handler that decodes CFSR into a pulse count.
//   1 pulse  = STKERR      (bit 12) — exception-entry stack push failed
//   2 pulses = IBUSERR     (bit  8) — instruction fetch bus error
//   3 pulses = PRECISERR   (bit  9) — synchronous data bus error
//   4 pulses = IMPRECISERR (bit 10) — async data bus error (buffered store)
//   5 pulses = other / unknown
#[exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    let cfsr = core::ptr::read_volatile(CFSR);
    let n: u32 = if (cfsr & (1 << 10)) != 0 {
        4
    } else if (cfsr & (1 << 9)) != 0 {
        3
    } else if (cfsr & (1 << 8)) != 0 {
        2
    } else if (cfsr & (1 << 12)) != 0 {
        1
    } else {
        5
    };
    led_pattern_forever(n, 400, 400, 1500);
}

// MemManage (memfault) handler — decodes MMFSR (low byte of CFSR) into a
// pulse count. Uses a distinct 250 ms cadence so it's tellable apart from
// the HardFault pattern.
//   1 pulse  = IACCVIOL (bit 0) — instruction fetch from an XN/no-access region
//   2 pulses = DACCVIOL (bit 1) — data access violation
//   3 pulses = MSTKERR/MUNSTKERR (bits 4/3) — fault stacking on exception entry
//   4 pulses = MMARVALID (bit 7) — MMFAR holds the faulting address
//   5 pulses = other / unknown
#[exception]
unsafe fn MemoryManagement() -> ! {
    let mmfsr = core::ptr::read_volatile(CFSR) & 0xFF;
    let _fault_addr = core::ptr::read_volatile(MMFAR); // valid iff MMARVALID
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

// __pre_init runs BEFORE .bss zeroing / .data copy. Blinks slowly for
// ~5 s so we can visually confirm the bootloader→app jump landed and
// Reset started executing app code.
#[pre_init]
unsafe fn pre_init() {
    // MPU + L1 caches first (MPU must precede cache enable), then arm the
    // MemManage fault so an MPU violation blinks a diagnostic instead of
    // silently escalating. Runs before .data/.bss init — touches only
    // registers, no statics.
    configure_mpu_and_caches();
    enable_memfault();
    enable_dwt();
    led_output();
    for _ in 0..5 {
        core::ptr::write_volatile(GPIOC_BSRR, 1 << 7);
        delay_ms(500);
        core::ptr::write_volatile(GPIOC_BSRR, 1 << 23);
        delay_ms(500);
    }
}

#[entry]
fn main() -> ! {
    unsafe {
        enable_dwt();
        led_output();
        // Distinctive pattern: 1 s ON, 1 s OFF, 4 rapid 100 ms blinks,
        // 1 s OFF, repeat. The 1 s steady-ON is impossible to produce
        // by any handler here, so seeing it proves `main` is running.
        loop {
            core::ptr::write_volatile(GPIOC_BSRR, 1 << 7);
            delay_ms(1000);
            core::ptr::write_volatile(GPIOC_BSRR, 1 << 23);
            delay_ms(1000);
            for _ in 0..4 {
                core::ptr::write_volatile(GPIOC_BSRR, 1 << 7);
                delay_ms(100);
                core::ptr::write_volatile(GPIOC_BSRR, 1 << 23);
                delay_ms(100);
            }
            delay_ms(1000);
        }
    }
}
