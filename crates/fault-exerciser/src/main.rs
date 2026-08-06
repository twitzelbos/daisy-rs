#![no_std]
#![no_main]

//! Fault / interrupt vector exerciser.
//!
//! Deliberately drives the Cortex-M7 exception paths so Renode's model can be
//! verified against RM0433 / the ARMv7-M architecture, end to end:
//!
//!   1. **Reset vector** — reaching `main` at all proves LoadELF set SP+PC
//!      from the vector table and the reset handler ran.
//!   2. **SysTick** (system exception 15) — pended directly via ICSR.PENDSTSET.
//!   3. **External NVIC interrupt** (TIM2) — set-pending via the NVIC, proving
//!      the external interrupt path + vector dispatch (no timer hardware needed;
//!      NVIC set-pending activates the handler regardless of source).
//!   4. **MemManage fault** (system exception 4) — an MPU no-access region over
//!      a valid cell faults on access; the handler records the hit, disables the
//!      MPU, and returns, so the faulting load RE-EXECUTES and succeeds — proving
//!      both fault entry AND clean return/recovery.
//!
//! Each handler writes a marker word into a fixed DTCM array; a Renode test
//! reads the array back and asserts every vector fired. Runs on the reset
//! default clock — no PLL/RCC/PWR setup needed.

use cortex_m_rt::{entry, exception};
use panic_halt as _;
use stm32h7xx_hal::pac;
use stm32h7xx_hal::pac::interrupt;

// Marker array in DTCM (see memory.x). Indices:
//   0 reset, 1 systick, 2 tim2 irq, 3 memmanage, 4 done, 5 recovered-load value
const MARKERS: *mut u32 = 0x2001_F000 as *mut u32;

// Core registers (raw MMIO, ARMv7-M arch ref).
const SHCSR: *mut u32 = 0xE000_ED24 as *mut u32; // MEMFAULTENA = bit 16
const ICSR: *mut u32 = 0xE000_ED04 as *mut u32; // PENDSTSET = bit 26
const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32 = 0xE000_EDA0 as *mut u32;

// A valid, backed cell (AXI SRAM) used as the MPU-fault target.
const TEST_ADDR: *mut u32 = 0x2400_0000 as *mut u32;
const TEST_VALUE: u32 = 0x00C0_FFEE;

#[inline(always)]
unsafe fn mark(index: isize, value: u32) {
    core::ptr::write_volatile(MARKERS.offset(index), value);
}

#[entry]
fn main() -> ! {
    unsafe {
        for i in 0..8 {
            mark(i, 0);
        }
        mark(0, 1); // reset vector → main reached

        // 1) SysTick: pend the system exception directly (no timer needed).
        core::ptr::write_volatile(ICSR, 1 << 26);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        // 2) External NVIC interrupt: unmask + set-pending TIM2. The handler
        //    activates from the pending bit alone — no timer hardware required.
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
        cortex_m::peripheral::NVIC::pend(pac::Interrupt::TIM2);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        // 3) MemManage fault via an MPU no-access region over TEST_ADDR.
        core::ptr::write_volatile(TEST_ADDR, TEST_VALUE); // prime the cell first
        // Enable the MemManage fault so the violation does NOT escalate to HardFault.
        core::ptr::write_volatile(SHCSR, core::ptr::read_volatile(SHCSR) | (1 << 16));
        // Region 0: 32 bytes @ TEST_ADDR, AP=000 (no access), enabled.
        core::ptr::write_volatile(MPU_RNR, 0);
        core::ptr::write_volatile(MPU_RBAR, TEST_ADDR as u32);
        core::ptr::write_volatile(MPU_RASR, 1 | (4 << 1)); // ENABLE | SIZE=4 (32B), AP=000
        core::ptr::write_volatile(MPU_CTRL, 1 | (1 << 2)); // ENABLE | PRIVDEFENA
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        // This load faults (MemManage). The handler disables the MPU and
        // returns; the load then re-executes and reads TEST_VALUE back.
        let recovered = core::ptr::read_volatile(TEST_ADDR);
        mark(5, recovered);

        mark(4, 0x0000_D09E); // done sentinel
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[exception]
unsafe fn SysTick() {
    mark(1, 1);
}

// MemManage handler — records the hit, then disables the MPU so the faulting
// load re-executes successfully on return (clean recovery, no PC surgery).
#[exception]
unsafe fn MemoryManagement() {
    mark(3, 1);
    core::ptr::write_volatile(MPU_CTRL, 0);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

#[interrupt]
unsafe fn TIM2() {
    mark(2, 1);
    cortex_m::peripheral::NVIC::unpend(pac::Interrupt::TIM2);
}
