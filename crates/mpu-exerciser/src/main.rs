#![no_std]
#![no_main]

//! Cortex-M7 MPU exerciser.
//!
//! Drives the ARMv7-M MPU (DDI 0403E §B3.5 / PM0253) through its behaviours so
//! Renode's `tlib` PMSAv7 model can be verified against the architecture, end to
//! end. Each sub-test programs MPU regions, performs one access, and records
//! whether it faulted (MemManage) in a DTCM marker; the handler flags the fault
//! and disables the MPU so the access re-executes and the sequence continues.
//! A Renode test reads the markers and asserts the fault/no-fault pattern the
//! ARM ARM requires.
//!
//! Covered: MPU_TYPE.DREGION; AP=000 no-access → fault; AP=110 RO → write
//! faults / read succeeds; region priority (highest-numbered wins); subregion
//! disable (SRD) removes coverage; PRIVDEFENA background map (off → fault, on →
//! succeed); MPU disabled → no enforcement; and MMFSR/MMFAR (DACCVIOL +
//! MMARVALID + faulting address) on a data violation.

use cortex_m_rt::{entry, exception};
use panic_halt as _;

// --- Cortex-M7 MPU + SCB registers (raw MMIO, ARMv7-M arch ref) ---
const MPU_TYPE: *const u32 = 0xE000_ED90 as *const u32;
const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32 = 0xE000_EDA0 as *mut u32;
const SHCSR: *mut u32 = 0xE000_ED24 as *mut u32; // MEMFAULTENA = bit 16
const CFSR: *mut u32 = 0xE000_ED28 as *mut u32; // MMFSR = low byte
const MMFAR: *const u32 = 0xE000_ED34 as *const u32;

// MPU_CTRL bits.
const CTRL_ENABLE: u32 = 1 << 0;
const CTRL_PRIVDEFENA: u32 = 1 << 2;

// Marker array + fault flag in DTCM (see memory.x).
const MARKERS: *mut u32 = 0x2001_F000 as *mut u32;
const FAULT_FLAG: *mut u32 = 0x2001_F100 as *mut u32;

// A backed AXI-SRAM cell used as the protected target.
const TEST_ADDR: u32 = 0x2400_0000;
const TEST_VALUE: u32 = 0x00C0_FFEE;

// Marker slot indices.
const M_DONE: isize = 0;
const M_DREGION: isize = 1;
const M_MMFSR: isize = 2;
const M_MMFAR: isize = 3;
const M_T1_NOACCESS: isize = 4; // AP=000 read → 1
const M_T2_RO_WRITE: isize = 5; // AP=110 write → 1
const M_T3_RO_READ: isize = 6; // AP=110 read → 0
const M_T4_PRIORITY: isize = 7; // higher region full over no-access → 0
const M_T5_SRD_OFF: isize = 8; // disabled subregion → 0
const M_T5_SRD_ON: isize = 9; // enabled subregion → 1
const M_T6_BG_FAULT: isize = 10; // PRIVDEFENA=0 background → 1
const M_T7_BG_OK: isize = 11; // PRIVDEFENA=1 background → 0
const M_T8_DISABLED: isize = 12; // MPU disabled → 0

#[inline(always)]
unsafe fn mark(index: isize, value: u32) {
    core::ptr::write_volatile(MARKERS.offset(index), value);
}

// Region size field: region = 2^(SIZE+1) bytes, so SIZE = log2(bytes) - 1.
const fn rasr(ap: u32, size_log2: u32, srd: u32, xn: u32) -> u32 {
    1 | (((size_log2 - 1) & 0x1F) << 1) | ((srd & 0xFF) << 8) | ((ap & 7) << 24) | ((xn & 1) << 28)
}

unsafe fn program_region(n: u32, base: u32, rasr_val: u32) {
    core::ptr::write_volatile(MPU_RNR, n);
    core::ptr::write_volatile(MPU_RBAR, base);
    core::ptr::write_volatile(MPU_RASR, rasr_val);
}

unsafe fn disable_all_regions() {
    for n in 0..16u32 {
        core::ptr::write_volatile(MPU_RNR, n);
        core::ptr::write_volatile(MPU_RASR, 0);
    }
}

unsafe fn set_ctrl(val: u32) {
    core::ptr::write_volatile(MPU_CTRL, val);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Begin a sub-test: MPU off, clear the sticky fault status, arm the fault flag,
/// and clear any leftover regions.
unsafe fn begin() {
    set_ctrl(0);
    core::ptr::write_volatile(CFSR, core::ptr::read_volatile(CFSR)); // W1C the MMFSR byte
    core::ptr::write_volatile(FAULT_FLAG, 0);
    disable_all_regions();
}

/// True if the volatile read of `addr` took a MemManage fault.
unsafe fn read_faulted(addr: u32) -> u32 {
    let _ = core::ptr::read_volatile(addr as *const u32);
    core::ptr::read_volatile(FAULT_FLAG)
}

/// True if the volatile write to `addr` took a MemManage fault.
unsafe fn write_faulted(addr: u32, value: u32) -> u32 {
    core::ptr::write_volatile(addr as *mut u32, value);
    core::ptr::read_volatile(FAULT_FLAG)
}

// Program the flash (code), DTCM (stack + markers) and PPB (system) regions
// full-access, so exception entry and the handler work even with PRIVDEFENA=0
// (Renode does not exempt the PPB from the MPU the way real silicon does).
unsafe fn program_infrastructure_regions() {
    program_region(0, 0x0800_0000, rasr(0b011, 20, 0, 0)); // 1 MB flash, RWX
    program_region(1, 0x2000_0000, rasr(0b011, 17, 0, 0)); // 128 KB DTCM, RWX
    program_region(2, 0xE000_0000, rasr(0b011, 20, 0, 1)); // 1 MB PPB, RW, XN
}

#[entry]
fn main() -> ! {
    unsafe {
        for i in 0..16 {
            mark(i, 0);
        }
        // Enable MemManage so violations don't escalate straight to HardFault.
        core::ptr::write_volatile(SHCSR, core::ptr::read_volatile(SHCSR) | (1 << 16));
        core::ptr::write_volatile(TEST_ADDR as *mut u32, TEST_VALUE);

        // T0: MPU_TYPE.DREGION — number of regions the core reports.
        let dregion = (core::ptr::read_volatile(MPU_TYPE) >> 8) & 0xFF;
        mark(M_DREGION, dregion);

        // T1: AP=000 (no access) region over TEST_ADDR → read faults.
        begin();
        program_region(0, TEST_ADDR, rasr(0b000, 5, 0, 0)); // 32 B, no access
        set_ctrl(CTRL_ENABLE | CTRL_PRIVDEFENA);
        mark(M_T1_NOACCESS, read_faulted(TEST_ADDR));
        // Capture the fault status the violation left (still sticky — the
        // handler recovered by disabling the MPU, not by clearing CFSR).
        mark(M_MMFSR, core::ptr::read_volatile(CFSR) & 0xFF);
        mark(M_MMFAR, core::ptr::read_volatile(MMFAR));

        // T2: AP=110 (RO) region → write faults.
        begin();
        program_region(0, TEST_ADDR, rasr(0b110, 5, 0, 0)); // 32 B, RO
        set_ctrl(CTRL_ENABLE | CTRL_PRIVDEFENA);
        mark(M_T2_RO_WRITE, write_faulted(TEST_ADDR, 0xDEAD_BEEF));

        // T3: AP=110 (RO) region → read succeeds.
        begin();
        program_region(0, TEST_ADDR, rasr(0b110, 5, 0, 0));
        set_ctrl(CTRL_ENABLE | CTRL_PRIVDEFENA);
        mark(M_T3_RO_READ, read_faulted(TEST_ADDR));

        // T4: region priority — region 1 (full access) overlaps region 0 (no
        // access) over TEST_ADDR; the higher-numbered region wins → no fault.
        begin();
        program_region(0, TEST_ADDR, rasr(0b000, 5, 0, 0)); // no access
        program_region(1, TEST_ADDR, rasr(0b011, 5, 0, 0)); // full access, higher #
        set_ctrl(CTRL_ENABLE | CTRL_PRIVDEFENA);
        mark(M_T4_PRIORITY, read_faulted(TEST_ADDR));

        // T5: subregion disable — a 256 B no-access region with subregion 0
        // (bytes 0..32) disabled. The disabled subregion falls through to the
        // default map → no fault; an enabled subregion still faults.
        begin();
        program_region(0, TEST_ADDR, rasr(0b000, 8, 0x01, 0)); // 256 B, SRD bit 0 set
        set_ctrl(CTRL_ENABLE | CTRL_PRIVDEFENA);
        mark(M_T5_SRD_OFF, read_faulted(TEST_ADDR)); // subregion 0 (disabled)
        mark(M_T5_SRD_ON, read_faulted(TEST_ADDR + 32)); // subregion 1 (enabled)

        // T6: PRIVDEFENA=0 — a privileged access outside every region should
        // fault (ARM ARM §B3.5.3). NOTE: Renode's Cortex-M core does not model
        // this (no ARM_FEATURE_PMSA) and never faults a privileged background
        // access — see mpu.robot. Cover code/stack/PPB so entry+handler work;
        // leave TEST_ADDR uncovered.
        begin();
        program_infrastructure_regions();
        set_ctrl(CTRL_ENABLE); // PRIVDEFENA = 0
        mark(M_T6_BG_FAULT, read_faulted(TEST_ADDR));

        // T7: PRIVDEFENA=1 — same layout, but the privileged default map now
        // backs the uncovered access → no fault.
        begin();
        program_infrastructure_regions();
        set_ctrl(CTRL_ENABLE | CTRL_PRIVDEFENA);
        mark(M_T7_BG_OK, read_faulted(TEST_ADDR));

        // T8: MPU disabled — a no-access region has no effect when CTRL.ENABLE=0.
        begin();
        program_region(0, TEST_ADDR, rasr(0b000, 5, 0, 0));
        set_ctrl(0); // disabled
        mark(M_T8_DISABLED, read_faulted(TEST_ADDR));

        // Leave the MPU off.
        set_ctrl(0);
        mark(M_DONE, 0x00C0_FFEE);
    }
    loop {
        cortex_m::asm::nop();
    }
}

// MemManage handler: flag the fault and disable the MPU so the faulting access
// re-executes successfully on return (clean recovery, no PC surgery).
#[exception]
unsafe fn MemoryManagement() {
    core::ptr::write_volatile(FAULT_FLAG, 1);
    core::ptr::write_volatile(MPU_CTRL, 0);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}
