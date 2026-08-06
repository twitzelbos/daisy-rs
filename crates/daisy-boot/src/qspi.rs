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
    // Prescaler=1 (divide-by-2 of kernel clock) is conservative. We'll
    // raise it later once QSPI signal integrity has been checked on scope.
    unsafe {
        qspi.cr.write(|w| {
            w.prescaler()
                .bits(1)
                .sshift()
                .set_bit()
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

    // Software-reset the flash chip. The STM32 reset does not propagate to
    // the QSPI chip, so on a warm boot we may be inheriting mid-operation
    // state (partial writes, quad-mode confusion) from the previous run.
    // libDaisy does the same via `ResetMemory()` in per/qspi.cpp. IS25LP064A
    // datasheet §8: RSTEN (0x66) must immediately precede RST (0x99);
    // tRSTP ≈ 30 µs before the next command.
    single_byte_command(&qspi, CMD_ENABLE_RESET);
    single_byte_command(&qspi, CMD_RESET_MEMORY);
    crate::delay_us(100); // generous vs the 30 µs tRSTP the datasheet specifies

    // Set the QE bit if it's not already set. Read status register first
    // so we don't burn a write cycle on flash we don't need to touch.
    if status_register(&qspi) & STATUS_QE_BIT == 0 {
        write_enable(&qspi);
        write_status_register(&qspi, STATUS_QE_BIT);
        wait_while_wip(&qspi);
    }

    // Program the flash's dummy-cycle count to match the pre-data cycles the
    // controller clocks in memory-mapped mode. MUST precede
    // enter_memory_mapped. Without it the flash stays at its 6-cycle POR
    // default while the controller drives 8 (alt-byte 2 + DCYC 6), so every
    // XIP fetch is shifted one byte and the app's vector table / code reads
    // back corrupt on hardware. See set_read_parameters / READ_PARAMETERS_8_DUMMY.
    set_read_parameters(&qspi, READ_PARAMETERS_8_DUMMY);

    enter_memory_mapped(&qspi);
}

/// Set the flash's volatile Read Parameters register (SRP, 0xC0).
///
/// The IS25LP064A powers up with P4:P3=00 → 6 dummy cycles for 0xEB
/// (datasheet Table 6.10, the 6 including the 2 AX/mode-bit cycles). Our
/// memory-mapped config clocks an 8-bit alternate-byte phase (2 cycles,
/// carrying the 0xA0 AX mode bits) followed by DCYC=6 = 8 cycles between
/// address and data. Left at the default, the flash begins driving data two
/// cycles (one quad byte) before the controller samples, so every fetch
/// shifts by a byte and XIP code is corrupt on silicon. Writing 0xF0
/// (P4:P3=10 → 8 cycles) aligns the two, exactly as libDaisy does. The
/// volatile SRP write needs no WREN.
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
    while qspi.sr.read().busy().bit_is_set() {}
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
    while qspi.sr.read().busy().bit_is_set() {}
    qspi.fcr.write(|w| w.ctcf().set_bit());
}

/// Program QUADSPI for the memory-mapped-mode Fast Read Quad I/O command.
/// Assumes the peripheral is otherwise ready (EN=1, DCR programmed).
///
/// **BUSY must be 0 before writing CCR.** ST's own HAL_QSPI_MemoryMapped
/// waits on QSPI_FLAG_BUSY before configuring memory-mapped mode. Without
/// it, the last indirect transaction may still be finishing when we
/// rewrite CCR — the peripheral latches the write while BUSY, ends up
/// in an undefined state, and the first AHB read to 0x9000_0000 hangs.
///
/// **~1 ms settling required after CCR write.** RM0433 §23.3.7 does not
/// document any delay requirement, but empirically the first AHB read
/// deadlocks unless we give the peripheral time to enter memory-mapped
/// mode. Symptom: with no delay, the bootloader hangs after
/// `init_memory_mapped()` before the vector-table read in `main`.
/// TODO: root-cause this against the IS25LP064A datasheet and any
/// STM32H7 errata; if a specific status bit signals mem-mapped ready,
/// poll that instead of blind waiting.
pub fn enter_memory_mapped(qspi: &pac::QUADSPI) {
    while qspi.sr.read().busy().bit_is_set() {}
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
    // decode phase entirely, matching libDaisy's config exactly.
    unsafe {
        qspi.abr.write(|w| w.bits(0x0000_00A0));
        qspi.ccr.write(|w| {
            w.fmode()
                .bits(0b11) // memory-mapped
                .dmode()
                .bits(0b11) // data on 4 lines
                .dcyc()
                .bits(6) // 6 dummy cycles
                .absize()
                .bits(0b00) // 8-bit alternate bytes
                .abmode()
                .bits(0b11) // alt bytes on 4 lines
                .adsize()
                .bits(0b10) // 24-bit address
                .admode()
                .bits(0b11) // address on 4 lines
                .imode()
                .bits(0b01) // instruction on 1 line
                .instruction()
                .bits(CMD_FAST_READ_QUAD_IO)
                .sioo()
                .set_bit() // instruction once
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
    while qspi.cr.read().abort().bit_is_set() {}
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
    while qspi.sr.read().busy().bit_is_set() {}
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
        while qspi.sr.read().ftf().bit_is_clear() {}
        unsafe {
            qspi.dr.write(|w| w.bits(u32::from_le_bytes(buf)));
        }
    }
    while qspi.sr.read().busy().bit_is_set() {}
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
    while qspi.sr.read().tcf().bit_is_clear() {}
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
    while qspi.sr.read().busy().bit_is_set() {}
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
    while qspi.sr.read().busy().bit_is_set() {}
    qspi.fcr.write(|w| w.ctcf().set_bit());
}

fn wait_while_wip(qspi: &pac::QUADSPI) {
    while status_register(qspi) & STATUS_WIP_BIT != 0 {}
}
