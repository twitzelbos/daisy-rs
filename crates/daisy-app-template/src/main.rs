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

// SCB fault status register (RM0433 Cortex-M7 arch ref).
const CFSR: *const u32 = 0xE000_ED28 as *const u32;

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

// __pre_init runs BEFORE .bss zeroing / .data copy. Blinks slowly for
// ~5 s so we can visually confirm the bootloader→app jump landed and
// Reset started executing app code.
#[pre_init]
unsafe fn pre_init() {
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
