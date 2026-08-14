//! In-process ELF → raw firmware binary conversion.
//!
//! Equivalent to `arm-none-eabi-objcopy -O binary`. We walk the loadable
//! program headers (PT_LOAD), place their data at their physical addresses
//! into one contiguous buffer starting from the lowest paddr, and pad any
//! gaps with 0xFF (matching NOR flash's erased state — writing 0xFF is a
//! no-op on both internal flash and QSPI).

use anyhow::{anyhow, Context, Result};
use object::elf::PT_LOAD;
use object::read::elf::{ElfFile32, ProgramHeader};
use object::{Endianness, Object, ObjectSymbol};
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

/// Read an ELF and return `(base_address, bin_bytes)`.
pub fn elf_to_bin(path: &Path) -> Result<(u32, Vec<u8>)> {
    let bytes = fs::read(path).with_context(|| format!("read {}", path.display()))?;
    let elf: ElfFile32<Endianness> =
        ElfFile32::parse(bytes.as_slice()).context("parse ELF header")?;

    let endian = elf.endian();
    let headers = elf
        .elf_program_headers()
        .iter()
        .filter(|ph| ph.p_type(endian) == PT_LOAD && ph.p_filesz(endian) > 0)
        .collect::<Vec<_>>();

    if headers.is_empty() {
        return Err(anyhow!("ELF has no loadable segments"));
    }

    let base = headers
        .iter()
        .map(|ph| ph.p_paddr(endian))
        .min()
        .expect("non-empty");
    let end = headers
        .iter()
        .map(|ph| ph.p_paddr(endian) + ph.p_filesz(endian))
        .max()
        .expect("non-empty");
    let size = (end - base) as usize;

    let mut out = vec![0xFFu8; size];
    for ph in &headers {
        let data = ph
            .data(endian, bytes.as_slice())
            .map_err(|_| anyhow!("read PT_LOAD segment data"))?;
        let offset = (ph.p_paddr(endian) - base) as usize;
        out[offset..offset + data.len()].copy_from_slice(data);
    }

    Ok((base, out))
}

/// The STM32H750's on-chip RAM regions, each a **separate contiguous window**
/// with unmapped gaps between them (mirrors `daisy_bsp::boot_check::is_valid_sram`).
/// Bounds are inclusive of the one-past-the-end address (a stack pointer / section
/// end legitimately points there).
const RAM_REGIONS: &[(u64, u64, &str)] = &[
    (0x2000_0000, 0x2002_0000, "DTCM (128 KiB)"),
    (0x2400_0000, 0x2408_0000, "AXI SRAM (512 KiB)"),
    (0x3000_0000, 0x3005_0000, "D2 SRAM1..3 (288 KiB)"),
];

fn region_of(addr: u64) -> Option<usize> {
    RAM_REGIONS
        .iter()
        .position(|&(lo, hi, _)| addr >= lo && addr <= hi)
}

/// Read the ELF symbol table as `name -> address`.
pub fn read_symbols(path: &Path) -> Result<BTreeMap<String, u64>> {
    let bytes = fs::read(path).with_context(|| format!("read {}", path.display()))?;
    let elf: ElfFile32<Endianness> =
        ElfFile32::parse(bytes.as_slice()).context("parse ELF header")?;
    let mut map = BTreeMap::new();
    for sym in elf.symbols() {
        if let Ok(name) = sym.name() {
            if !name.is_empty() {
                map.insert(name.to_string(), sym.address());
            }
        }
    }
    Ok(map)
}

/// Verify cortex-m-rt's startup-critical linker symbols keep RAM initialisation
/// within a **single contiguous region**. The startup code zeroes `.bss`
/// (`__sbss..__ebss`) and copies `.data` (`__sdata..__edata`) with tight loops
/// that assume contiguous memory — if a symbol is dragged into a *different*
/// region (e.g. a `#[link_section]` NOLOAD block `INSERT AFTER .bss` in D2 SRAM
/// pushing `__ebss` to `0x3000_0600`), the loop runs across the unmapped
/// DTCM→D2 gap and the M7 locks up before `main` (this passes Renode, which
/// backs the gap, but faults on silicon). Returns the list of violations.
pub fn check_startup_ram_invariant(symbols: &BTreeMap<String, u64>) -> Result<(), Vec<String>> {
    let mut errs = Vec::new();
    let get = |name: &str| symbols.get(name).copied();

    // Ranges that a startup loop walks end-to-end; both ends must be in the SAME
    // region, or the loop crosses an unmapped gap.
    for (label, lo_name, hi_name) in [
        (".bss zero", "__sbss", "__ebss"),
        (".data copy (dest)", "__sdata", "__edata"),
    ] {
        if let (Some(lo), Some(hi)) = (get(lo_name), get(hi_name)) {
            match (region_of(lo), region_of(hi)) {
                (Some(a), Some(b)) if a != b => errs.push(format!(
                    "{label}: {lo_name}=0x{lo:08x} ({}) and {hi_name}=0x{hi:08x} ({}) are in \
                     DIFFERENT RAM regions — startup init would cross the unmapped gap and lock up",
                    RAM_REGIONS[a].2, RAM_REGIONS[b].2,
                )),
                _ => {}
            }
        }
    }

    // Every startup symbol must land in *some* valid RAM region.
    for name in [
        "__sbss",
        "__ebss",
        "__sdata",
        "__edata",
        "__sheap",
        "_stack_start",
        "_stack_end",
    ] {
        if let Some(a) = get(name) {
            if region_of(a).is_none() {
                errs.push(format!(
                    "{name}=0x{a:08x} is not in any valid on-chip RAM region"
                ));
            }
        }
    }

    if errs.is_empty() {
        Ok(())
    } else {
        Err(errs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn syms(pairs: &[(&str, u64)]) -> BTreeMap<String, u64> {
        pairs.iter().map(|&(n, a)| (n.to_string(), a)).collect()
    }

    #[test]
    fn contiguous_dtcm_layout_passes() {
        // Healthy XIP app: everything inside DTCM.
        let s = syms(&[
            ("__sdata", 0x2000_0000),
            ("__edata", 0x2000_05c4),
            ("__sbss", 0x2000_05c4),
            ("__ebss", 0x2000_2794),
            ("__sheap", 0x2000_2794),
            ("_stack_end", 0x2000_2794),
            ("_stack_start", 0x2002_0000), // one-past-DTCM-top, inclusive
        ]);
        assert!(check_startup_ram_invariant(&s).is_ok());
    }

    #[test]
    fn ebss_dragged_into_d2_is_rejected() {
        // The exact `.sram_d2 INSERT AFTER .bss` bug: __ebss pushed to D2 SRAM.
        let s = syms(&[
            ("__sbss", 0x2000_05c4), // DTCM
            ("__ebss", 0x3000_0600), // D2 SRAM — .bss zero crosses the gap
        ]);
        let errs = check_startup_ram_invariant(&s).unwrap_err();
        assert!(errs
            .iter()
            .any(|e| e.contains(".bss zero") && e.contains("DIFFERENT")));
    }

    #[test]
    fn symbol_in_a_hole_is_rejected() {
        let s = syms(&[("_stack_start", 0x2200_0000)]); // in the DTCM→AXI gap
        assert!(check_startup_ram_invariant(&s).is_err());
    }
}
