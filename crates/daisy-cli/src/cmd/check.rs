//! `daisy check-elf` — static memory-safety checks on a firmware ELF.
//!
//! Currently: the startup RAM invariant (cortex-m-rt's `.bss`/`.data` init stays
//! within one contiguous RAM region). Run it in CI on every linked app ELF; it
//! turns the "startup RAM init faults on unmapped memory" class of bug — a
//! section in a foreign region dragging `__ebss` across a RAM-region gap, which
//! boots on Renode (it backs the gap) but bus-faults before `main` on silicon —
//! into a red check.

use anyhow::{bail, Result};
use std::path::PathBuf;

#[derive(clap::Args, Debug)]
pub struct Args {
    /// ELF firmware image(s) to check.
    #[arg(required = true)]
    pub elfs: Vec<PathBuf>,
}

pub fn run(args: Args) -> Result<()> {
    let mut failed = false;
    for elf in &args.elfs {
        let symbols = crate::elf::read_symbols(elf)?;
        match crate::elf::check_startup_ram_invariant(&symbols) {
            Ok(()) => println!("\u{2713} {} — startup RAM invariant OK", elf.display()),
            Err(errs) => {
                failed = true;
                eprintln!(
                    "\u{2717} {} — startup RAM invariant VIOLATED:",
                    elf.display()
                );
                for e in errs {
                    eprintln!("    - {e}");
                }
            }
        }
    }
    if failed {
        bail!("startup RAM invariant check failed (see above)");
    }
    Ok(())
}
