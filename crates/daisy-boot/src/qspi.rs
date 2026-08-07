//! QUADSPI driver for the Daisy Seed's IS25LP064A.
//!
//! stm32h7xx-hal only supports indirect QSPI mode, so this file drives the
//! peripheral directly through the PAC. Provides three operating modes:
//!
//!   * `init_memory_mapped` — one-shot init at boot. QE bit is set on the
//!     flash chip and QUADSPI is configured for memory-mapped Fast Read
//!     Quad I/O (0xEB). After this returns, 0x9000_0000..0x907F_FFFF is
//!     directly executable.
//!   * `exit_memory_mapped` — cancel the memory-mapped mode via ABORT so
//!     indirect-mode commands can run. Used by the DFU service before
//!     erase/program.
//!   * Indirect-mode primitives: `erase_sector_4k`, `program_page`. All
//!     handle the WEL / WIP polling. Callers only need to line up
//!     addresses on 4 KiB / 256 B boundaries.
//!
//! Refs: RM0433 §22 (QUADSPI); IS25LP064A datasheet §8 (commands),
//! §7.5 (Quad Enable bit), Table 7.1 (memory organisation).

use crate::hal::gpio::{gpiof, gpiog, Speed};
use crate::hal::pac;
use crate::hal::rcc::{rec, ResetEnable};

// IS25LP064A commands.
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_READ_STATUS_REGISTER: u8 = 0x05;
const CMD_WRITE_STATUS_REGISTER: u8 = 0x01;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_SECTOR_ERASE_4K: u8 = 0x20;
const CMD_FAST_READ_QUAD_IO: u8 = 0xEB;
const CMD_ENABLE_RESET: u8 = 0x66;
const CMD_RESET_MEMORY: u8 = 0x99;
const CMD_SET_READ_PARAMETERS: u8 = 0xC0;

const STATUS_QE_BIT: u8 = 1 << 6;
const STATUS_WIP_BIT: u8 = 1 << 0;

/// Timeout for a short QSPI command to finish (BUSY/TCF/FTF) — µs.
const CMD_TIMEOUT_US: u32 = 20_000; // 20 ms; commands actually take µs
/// Timeout for a flash Write-In-Progress (erase/program) to clear — µs.
/// IS25LP064A worst-case sector erase is well under 1 s; give ample margin.
const WIP_TIMEOUT_US: u32 = 3_000_000; // 3 s

/// Spin until `cond()` returns true, bounded by BOTH a DWT-cycle deadline and a
/// hard iteration cap. Returns `true` if the condition was met, `false` on
/// timeout. **A bootloader must never wedge forever on a misbehaving flash** —
/// on timeout every caller here continues best-effort, and the boot decision in
/// `main` falls back to DFU service mode if XIP then reads back implausible.
/// The iteration cap is a backstop in case DWT CYCCNT isn't running (it is, by
/// the time QSPI init runs, but we don't rely on it).
#[must_use = "ignoring a QSPI timeout hides a stuck flash"]
fn spin_until(timeout_us: u32, mut cond: impl FnMut() -> bool) -> bool {
    use cortex_m::peripheral::DWT;
    let start = DWT::cycle_count();
    // DWT runs at sys_ck; the bootloader pins that to 400 MHz (see delay_us).
    let deadline_cycles = timeout_us.saturating_mul(400);
    let mut iters: u32 = 0;
    const MAX_ITERS: u32 = 400_000_000; // absolute backstop if DWT is dead
    loop {
        if cond() {
            return true;
        }
        iters = iters.wrapping_add(1);
        if iters >= MAX_ITERS
            || DWT::cycle_count().wrapping_sub(start) >= deadline_cycles
        {
            return false;
        }
    }
}

/// Read Parameters register value (IS25LP064A Table 6.7). Bits P4:P3 = 10
/// select 8 dummy cycles for 0xEB Fast Read Quad I/O (Table 6.10, and those
/// 8 include the 2 AX/mode-bit cycles); P7:P5 = 111 sets max drive strength.
/// Value = 0xF0, identical to libDaisy's `DummyCyclesConfig` for this part.
/// Required so the flash's dummy count matches the 8 pre-data cycles the
/// controller clocks (8-bit alternate byte = 2 cycles + DCYC=6). See
/// `set_read_parameters`.
const READ_PARAMETERS_8_DUMMY: u8 = 0xF0;

/// FSIZE = log2(bytes) - 1. 8 MiB -> FSIZE = 22.
const FSIZE_8MIB: u8 = 22;

/// IS25LP064A sub-sector size (smallest erasable unit).
pub const SECTOR_SIZE: u32 = 4 * 1024;
/// IS25LP064A page size (largest programmable unit per PP command).
pub const PAGE_SIZE: usize = 256;

/// Configure GPIOF / GPIOG alt functions for QUADSPI Bank 1.
pub fn configure_pins(gpiof: gpiof::Parts, gpiog: gpiog::Parts) {
    // Bank 1: IO0=PF8, IO1=PF9, IO2=PF7, IO3=PF6, CLK=PF10, NCS=PG6.
    // AF10 for IO0/IO1/NCS, AF9 for IO2/IO3/CLK on the H7. Set VeryHigh
    // speed on all signals — XIP reads race at PLL-derived clock rates.
    let _ = gpiof.pf6.into_alternate::<9>().speed(Speed::VeryHigh);
    let _ = gpiof.pf7.into_alternate::<9>().speed(Speed::VeryHigh);
    let _ = gpiof.pf8.into_alternate::<10>().speed(Speed::VeryHigh);
    let _ = gpiof.pf9.into_alternate::<10>().speed(Speed::VeryHigh);
    let _ = gpiof.pf10.into_alternate::<9>().speed(Speed::VeryHigh);
    let _ = gpiog.pg6.into_alternate::<10>().speed(Speed::VeryHigh);
}

/// Initialise QUADSPI and enter memory-mapped mode.
///
/// The peripheral is left running so 0x9000_0000 reads flash. Nothing to
/// return — the CPU accesses the flash purely through the memory bus after
/// this returns.
pub fn init_memory_mapped(qspi: pac::QUADSPI, prec: rec::Qspi) {
    // Kick the peripheral's clock. We don't drive it from a specific PLL
    // here; the reset default (rcc_hclk3) is fine for bring-up. TODO: move
    // to a dedicated PLL kernel clock once we characterise timing margin.
    prec.enable();

    // Program the control register while the peripheral is disabled.
    // Prescaler=7 (÷8 of kernel clock ≈ 12.5 MHz) and SSHIFT=0 (no sample shift).
    // Conservative on purpose while we re-validate our quad path from a clean
    // flash state: a wide timing margin removes signal integrity as a variable so
    // any remaining corruption is unambiguously a config/state bug, not clocking.
    // (The board's quad hardware itself is GOOD — a 51,200-read libDaisy quad
    // stress test passed with zero errors — so once the clean-state read is
    // confirmed this can be raised toward libDaisy's ~100 MHz.) XIP reads are
    // I-cached, so a low clock costs ~nothing in practice. SSHIFT=0 matches
    // libDaisy (SAMPLE_SHIFTING_NONE); SSHIFT=1 sampled the data line at the
    // wrong instant.
    unsafe {
        qspi.cr.write(|w| {
            w.prescaler()
                .bits(7)
                .sshift()
                .clear_bit()
                .fthres()
                .bits(0)
                .en()
                .clear_bit()
        });
        qspi.dcr.write(|w| {
            w.fsize()
                .bits(FSIZE_8MIB)
                .csht()
                .bits(2)
                .ckmode()
                .clear_bit()
        });
        qspi.cr.modify(|_, w| w.en().set_bit());
    }

    // Full flash recovery + memory-mapped setup. `enter_memory_mapped` is
    // self-contained and robust to any prior flash state — see its docs.
    enter_memory_mapped(&qspi);
}

/// Recover the flash to a clean single-line state so indirect erase/program
/// commands are framed correctly. MUST be called after leaving memory-mapped
/// mode and before any indirect Page-Program / Sector-Erase, because those are
/// single-line instruction-led commands: if the flash is still in 0xEB
/// AX/continuous mode (which it is after our XIP reads), it ignores the
/// instruction byte and the write lands corrupt. Verified root cause of sparse
/// DFU-flash corruption — the write path used to abort the controller but never
/// reset the flash chip, so programming ran against a continuous-mode flash.
pub fn recover_flash_to_single_line(qspi: &pac::QUADSPI) {
    exit_memory_mapped(qspi); // abort controller (ES0392 errata dance)
    exit_continuous_read(qspi); // break the flash's continuous-read framing
    single_byte_command(qspi, CMD_ENABLE_RESET);
    single_byte_command(qspi, CMD_RESET_MEMORY);
    crate::delay_us(100); // tRSTP ≈ 30 µs; be generous
}

/// Force the flash out of AX / continuous ("performance-enhance") read mode.
///
/// A prior firmware that used Fast Read Quad I/O (0xEB) with the 0xA0 mode bits
/// + SIOO leaves the flash expecting `[addr][mode-bits][data]` with **no**
/// instruction on the next access. In that state it ignores every normal
/// (instruction-led) command — including our software reset — so single-line
/// reads come back garbage. We break out by issuing one read *in that same
/// no-instruction quad format* but with the mode bits = 0x00 (not the 0xAx
/// continuous pattern), which tells the flash to resume expecting an
/// instruction. Only the mode-bit **outputs** matter here (driven to a static
/// 0), so this works even on a board whose quad *data* reads are unreliable. If
/// the flash was NOT in continuous mode, this is a harmless throwaway read that
/// the ABORT + software reset immediately clean up.
fn exit_continuous_read(qspi: &pac::QUADSPI) {
    unsafe {
        qspi.dlr.write(|w| w.dl().bits(0)); // read 1 byte
        qspi.abr.write(|w| w.bits(0x0000_0000)); // mode bits = 0 → leave continuous
        qspi.ccr.write(|w| {
            w.fmode()
                .bits(0b01) // indirect read
                .dmode()
                .bits(0b11) // data on 4 lines
                .dcyc()
                .bits(6)
                .absize()
                .bits(0b00) // 8-bit mode byte
                .abmode()
                .bits(0b11) // mode bits on 4 lines
                .adsize()
                .bits(0b10) // 24-bit address
                .admode()
                .bits(0b11) // address on 4 lines
                .imode()
                .bits(0b00) // NO instruction — matches continuous-mode framing
        });
        qspi.ar.write(|w| w.bits(0));
    }
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.sr.read().tcf().bit_is_set());
    let _ = qspi.dr.read().bits();
    qspi.fcr.write(|w| w.ctcf().set_bit());
    // Cancel the transaction so the following instruction-led commands start clean.
    qspi.cr.modify(|_, w| w.abort().set_bit());
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.cr.read().abort().bit_is_clear());
}

/// Set the flash's volatile Read Parameters register (SRP, 0xC0).
///
/// The IS25LP064A powers up with P4:P3=00 → 6 dummy cycles for 0xEB
/// (datasheet Table 6.10, the 6 including the 2 AX/mode-bit cycles). Our
/// memory-mapped config clocks an 8-bit alternate-byte phase (2 cycles,
/// carrying the AX mode bits) followed by DCYC=6 = 8 cycles between address
/// and data. Left at the default, the flash begins driving data two cycles
/// (one quad byte) before the controller samples, so every fetch shifts by a
/// byte and XIP code is corrupt on silicon. Writing 0xF0 (P4:P3=10 → 8 cycles)
/// aligns the two, exactly as libDaisy does. The volatile SRP write needs no WREN.
fn set_read_parameters(qspi: &pac::QUADSPI, value: u8) {
    unsafe {
        qspi.dlr.write(|w| w.dl().bits(0));
        qspi.ccr.write(|w| {
            w.fmode()
                .bits(0b00) // indirect write
                .dmode()
                .bits(0b01) // data on 1 line
                .admode()
                .bits(0b00) // no address
                .imode()
                .bits(0b01) // instruction on 1 line
                .instruction()
                .bits(CMD_SET_READ_PARAMETERS)
        });
        qspi.dr.write(|w| w.data().bits(value as u32));
    }
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.sr.read().busy().bit_is_clear());
    qspi.fcr.write(|w| w.ctcf().set_bit());
}

/// Send a bare command byte (no address, no data). Used for one-shot
/// commands like Enable Reset / Reset Memory / Write Enable that only
/// take the command opcode.
fn single_byte_command(qspi: &pac::QUADSPI, cmd: u8) {
    unsafe {
        qspi.ccr.write(|w| {
            w.fmode()
                .bits(0b00)
                .dmode()
                .bits(0b00)
                .admode()
                .bits(0b00)
                .imode()
                .bits(0b01)
                .instruction()
                .bits(cmd)
        });
    }
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.sr.read().busy().bit_is_clear());
    qspi.fcr.write(|w| w.ctcf().set_bit());
}

/// Recover the flash and enter memory-mapped Fast Read Quad I/O mode.
///
/// SELF-CONTAINED and robust to any prior flash state — safe to call at init
/// and every time we return to XIP after indirect programming. It (1) breaks
/// any inherited AX/continuous read framing, (2) software-resets the flash,
/// (3) ensures the Quad-Enable bit, (4) re-applies the volatile 8-dummy-cycle
/// read parameters (the reset clears them to the 6-cycle POR default, which
/// would byte-shift every XIP fetch), then (5) programs the 0xEB continuous
/// memory-mapped read. Assumes EN=1 and DCR already programmed.
///
/// **BUSY must be 0 before writing CCR** (ST's HAL waits on it too), and a
/// ~1 ms settle after the CCR write is needed empirically or the first AHB
/// read to 0x9000_0000 deadlocks.
pub fn enter_memory_mapped(qspi: &pac::QUADSPI) {
    // (1) Break any inherited continuous-read framing so the instruction-led
    // recovery commands below are actually seen by the flash. Verified over
    // SWD: without this, a flash left in 0xEB continuous mode ignores every
    // instruction (JEDEC reads 0xFF, XIP reads 0x88888888) and even the reset
    // is dropped. The board's quad lines are good (51,200-read libDaisy stress
    // test, 0 errors); a no-op if the flash is already single-line.
    exit_continuous_read(qspi);
    // (2) Software-reset the flash — the STM32 reset does not reach the chip,
    // so a warm boot can inherit mid-operation/quad-mode state. RSTEN (0x66)
    // must immediately precede RST (0x99); tRSTP ≈ 30 µs (datasheet §8).
    single_byte_command(qspi, CMD_ENABLE_RESET);
    single_byte_command(qspi, CMD_RESET_MEMORY);
    crate::delay_us(100);
    // (3) Quad-Enable (non-volatile; set only if a fresh chip lacks it).
    if status_register(qspi) & STATUS_QE_BIT == 0 {
        write_enable(qspi);
        write_status_register(qspi, STATUS_QE_BIT);
        wait_while_wip(qspi);
    }
    // (4) Re-apply the VOLATILE read parameters (8 dummy cycles) — the reset in
    // (2) cleared them to the 6-cycle default. Controller drives 8 (alt-byte 2
    // + DCYC 6); mismatch byte-shifts every fetch. MUST precede the CCR write.
    set_read_parameters(qspi, READ_PARAMETERS_8_DUMMY);

    // (5) Program the memory-mapped read.
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.sr.read().busy().bit_is_clear());
    // IS25LP064A Fast Read Quad I/O (0xEB) — per datasheet §8.7 and
    // Figure 8.8:
    //
    //   IMODE=01  instruction (0xEB) on 1 line   — 8 clocks
    //   ADMODE=11 24-bit address on 4 lines      — 6 clocks
    //   ABMODE=11 8-bit MODE BITS on 4 lines     — 2 clocks   (REQUIRED)
    //   ABSIZE=00 mode bits = 1 byte (8 bits)
    //   DCYC=6    6 dummy cycles
    //   DMODE=11  data on 4 lines
    //   FMODE=11  memory-mapped mode
    //   SIOO=1    send instruction only once (per libDaisy pattern)
    //
    // The alternate-bytes / mode-bits phase is CRITICAL. If ABMODE=0
    // the STM32 QUADSPI skips it and the flash chip's state machine
    // never sees the required 2-clock mode-bits window between address
    // and dummies. On real silicon this causes subsequent reads to
    // return garbage and instruction fetches from XIP to hang the CPU.
    // Renode's `Memory.MappedMemory` doesn't emulate any of this — it
    // returns bytes unconditionally — which is why our Renode tests
    // passed for weeks while the hardware silently failed.
    //
    // Mode-bits value 0xA0 = M[7:4]=1010b enables AX Continuous Read
    // mode on the IS25LP064A (datasheet §8.7 last paragraph). Combined
    // with SIOO=1 this lets subsequent reads skip the 0xEB command
    // decode phase entirely. This is libDaisy's EnableMemoryMappedMode
    // config verbatim (per/qspi.cpp): ABR=0xA0, DCYC=6, SIOO=INST_ONLY_FIRST.
    // A read config of ABR=0x00 / SIOO=0 (send instruction every command)
    // does NOT read correctly on this board's flash — verified over SWD it
    // returns uniform 0x88 — so we match libDaisy's continuous-read framing
    // exactly. `exit_continuous_read` at the top of init makes this safe
    // across warm resets by breaking any inherited continuous mode first.
    unsafe {
        qspi.abr.write(|w| w.bits(0x0000_00A0)); // mode bits 0xA0 → AX continuous read
        qspi.ccr.write(|w| {
            w.fmode()
                .bits(0b11) // memory-mapped
                .dmode()
                .bits(0b11) // data on 4 lines
                .dcyc()
                .bits(6) // 6 dummy cycles (+2 mode-bit = 8)
                .absize()
                .bits(0b00) // 8-bit mode byte
                .abmode()
                .bits(0b11) // mode bits on 4 lines
                .adsize()
                .bits(0b10) // 24-bit address
                .admode()
                .bits(0b11) // address on 4 lines
                .imode()
                .bits(0b01) // instruction on 1 line
                .instruction()
                .bits(CMD_FAST_READ_QUAD_IO)
                .sioo()
                .set_bit() // send instruction only on first command (continuous)
        });
    }
    // Small settling window. Not on the boot critical path.
    crate::delay_ms(1);
}

/// Cancel any in-flight (or memory-mapped) transaction so CCR can be
/// rewritten with a new indirect-mode command. Call once before a batch
/// of erase/program operations; the indirect primitives themselves do not
/// re-abort between calls.
///
/// Applies the ES0392 §2.7.4 errata workaround before ABORT — the errata
/// explicitly states "apply upon reset AND upon switching from
/// memory-mapped mode to any other mode." Real silicon has been known to
/// wedge on the mem-mapped→indirect transition without this dance. The
/// workaround does a full peripheral reset (CR=0xFF000001 → CCR write
/// twice → CR=0), so we snapshot CR beforehand and restore it after to
/// keep prescaler / sshift / fthres / EN intact for the subsequent
/// indirect commands.
pub fn exit_memory_mapped(qspi: &pac::QUADSPI) {
    let saved_cr = qspi.cr.read().bits();
    crate::qspi_errata_2_7_4_workaround();
    // Restore CR sans the self-clearing ABORT bit (bit 1). This
    // re-enables the peripheral with the same prescaler etc. `init`
    // configured.
    unsafe {
        qspi.cr.write(|w| w.bits(saved_cr & !0x2));
    }
    // Belt-and-braces ABORT poll — the workaround already cleared BUSY,
    // so this returns immediately, but keeps the invariant "after
    // exit_memory_mapped, no transaction is in flight."
    qspi.cr.modify(|_, w| w.abort().set_bit());
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.cr.read().abort().bit_is_clear());
    // Clear any latched status flags so subsequent polls start from zero.
    qspi.fcr
        .write(|w| w.ctef().set_bit().ctcf().set_bit().csmf().set_bit());
}

/// Erase the 4 KiB sub-sector containing `addr`. Blocks until the flash's
/// Write-In-Progress bit clears (~45 ms typical on the IS25LP064A). The
/// address may point anywhere inside the sector; the chip ignores the low
/// 12 bits for this command.
pub fn erase_sector_4k(qspi: &pac::QUADSPI, addr: u32) {
    write_enable(qspi);
    unsafe {
        qspi.ccr.write(|w| {
            w.fmode()
                .bits(0b00) // indirect write
                .dmode()
                .bits(0b00) // no data phase
                .dcyc()
                .bits(0)
                .adsize()
                .bits(0b10) // 24-bit address
                .admode()
                .bits(0b01) // addr on 1 line
                .imode()
                .bits(0b01) // instr on 1 line
                .instruction()
                .bits(CMD_SECTOR_ERASE_4K)
        });
        qspi.ar.write(|w| w.bits(addr));
    }
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.sr.read().busy().bit_is_clear());
    qspi.fcr.write(|w| w.ctcf().set_bit());
    wait_while_wip(qspi);
}

/// Program up to 256 bytes into a single 256-byte page starting at `addr`.
/// The caller must ensure `data.len() <= 256` and that the range does not
/// cross a page boundary (`addr` and `addr + data.len() - 1` share the top
/// 24-8 = 16 bits above the page offset). Blocks until WIP clears.
pub fn program_page(qspi: &pac::QUADSPI, addr: u32, data: &[u8]) {
    assert!(data.len() <= PAGE_SIZE, "PP is limited to 256 bytes/page");
    assert!(
        !data.is_empty(),
        "PP with empty buffer is a nop, not a command"
    );
    assert_eq!(
        addr & !(PAGE_SIZE as u32 - 1),
        (addr + data.len() as u32 - 1) & !(PAGE_SIZE as u32 - 1),
        "PP must not cross a 256-byte page boundary",
    );

    write_enable(qspi);
    unsafe {
        qspi.dlr.write(|w| w.dl().bits((data.len() - 1) as u32));
        qspi.ccr.write(|w| {
            w.fmode()
                .bits(0b00) // indirect write
                .dmode()
                .bits(0b01) // data on 1 line
                .dcyc()
                .bits(0)
                .adsize()
                .bits(0b10) // 24-bit address
                .admode()
                .bits(0b01) // addr on 1 line
                .imode()
                .bits(0b01) // instr on 1 line
                .instruction()
                .bits(CMD_PAGE_PROGRAM)
        });
        qspi.ar.write(|w| w.bits(addr));
    }
    // Push data in u32 words. The QUADSPI packs each store's bytes into the
    // FIFO in little-endian order. DLR bounds how many bytes the peripheral
    // actually sends over SPI, so trailing bytes in the final partial word
    // are ignored — no separate byte-write path needed. Padding with zeros
    // is safe: the flash never sees those bytes.
    for chunk in data.chunks(4) {
        let mut buf = [0u8; 4];
        buf[..chunk.len()].copy_from_slice(chunk);
        let _ = spin_until(CMD_TIMEOUT_US, || qspi.sr.read().ftf().bit_is_set());
        unsafe {
            qspi.dr.write(|w| w.bits(u32::from_le_bytes(buf)));
        }
    }
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.sr.read().busy().bit_is_clear());
    qspi.fcr.write(|w| w.ctcf().set_bit());
    wait_while_wip(qspi);
}

fn status_register(qspi: &pac::QUADSPI) -> u8 {
    // Indirect read, 1 byte, no address, no dummy cycles, data on 1 line,
    // instruction on 1 line. FMODE=01 (indirect read).
    unsafe {
        qspi.dlr.write(|w| w.dl().bits(0));
        qspi.ccr.write(|w| {
            w.fmode()
                .bits(0b01)
                .dmode()
                .bits(0b01)
                .dcyc()
                .bits(0)
                .admode()
                .bits(0b00)
                .imode()
                .bits(0b01)
                .instruction()
                .bits(CMD_READ_STATUS_REGISTER)
        });
    }
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.sr.read().tcf().bit_is_set());
    let value = qspi.dr.read().bits() as u8;
    qspi.fcr.write(|w| w.ctcf().set_bit());
    value
}

fn write_enable(qspi: &pac::QUADSPI) {
    // Command only, no address, no data.
    unsafe {
        qspi.ccr.write(|w| {
            w.fmode()
                .bits(0b00)
                .dmode()
                .bits(0b00)
                .admode()
                .bits(0b00)
                .imode()
                .bits(0b01)
                .instruction()
                .bits(CMD_WRITE_ENABLE)
        });
    }
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.sr.read().busy().bit_is_clear());
    qspi.fcr.write(|w| w.ctcf().set_bit());
}

fn write_status_register(qspi: &pac::QUADSPI, value: u8) {
    // Indirect write, 1 byte on 1 line.
    unsafe {
        qspi.dlr.write(|w| w.dl().bits(0));
        qspi.ccr.write(|w| {
            w.fmode()
                .bits(0b00)
                .dmode()
                .bits(0b01)
                .admode()
                .bits(0b00)
                .imode()
                .bits(0b01)
                .instruction()
                .bits(CMD_WRITE_STATUS_REGISTER)
        });
        qspi.dr.write(|w| w.data().bits(value as u32));
    }
    let _ = spin_until(CMD_TIMEOUT_US, || qspi.sr.read().busy().bit_is_clear());
    qspi.fcr.write(|w| w.ctcf().set_bit());
}

fn wait_while_wip(qspi: &pac::QUADSPI) {
    let _ = spin_until(WIP_TIMEOUT_US, || {
        status_register(qspi) & STATUS_WIP_BIT == 0
    });
}
