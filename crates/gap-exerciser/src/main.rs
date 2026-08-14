#![no_std]
#![no_main]

//! Verifies Renode's `GapGuard`. Writes a STARTED marker, then a word to the
//! first **unmapped** address past DTCM (`0x2002_0000`, the start of the DTCM→AXI
//! hole — exactly where a startup loop walking off the end of DTCM lands), then a
//! DONE sentinel.
//!
//! On silicon that gap write is a bus fault, so DONE is never written. With the
//! `GapGuard` provisioned, Renode faults too — the robot asserts `STARTED == 1`
//! and `DONE != 0xD09E`. Under *stock* Renode the write is silently swallowed
//! and DONE would be set — exactly the false-pass the guard exists to prevent.

use core::ptr::write_volatile;
use cortex_m_rt::entry;
use panic_halt as _;

const M: *mut u32 = 0x2001_F000 as *mut u32; // DTCM markers (same block the other exercisers use)
const M_STARTED: isize = 0;
const M_DONE: isize = 1;
const GAP_ADDR: *mut u32 = 0x2002_0000 as *mut u32; // first address past DTCM = start of the gap
const DONE: u32 = 0x0000_D09E;

#[entry]
fn main() -> ! {
    unsafe {
        write_volatile(M.offset(M_STARTED), 1);
        write_volatile(M.offset(M_DONE), 0);
        // Bus-faults on hardware / under the GapGuard; DONE stays 0.
        write_volatile(GAP_ADDR, 0xDEAD_BEEF);
        cortex_m::asm::dsb(); // force an imprecise fault to surface before DONE
        write_volatile(M.offset(M_DONE), DONE); // reached only if the gap access did NOT fault
    }
    loop {
        cortex_m::asm::wfi();
    }
}
