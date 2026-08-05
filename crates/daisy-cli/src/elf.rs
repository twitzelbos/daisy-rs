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
use object::Endianness;
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
