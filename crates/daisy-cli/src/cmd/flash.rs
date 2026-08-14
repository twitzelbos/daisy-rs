use anyhow::{anyhow, Context, Result};
use clap::Args as ClapArgs;
use std::path::PathBuf;

use crate::{dfu, elf};

/// Flash a firmware image over DFU.
///
/// Default: builds and flashes `daisy-app-template` to QSPI (0x9000_0000).
/// With `--bootloader`: builds and flashes `daisy-boot` to internal flash
/// (0x0800_0000). Users pass their own binaries via `--elf` to bypass the
/// package/build path.
#[derive(ClapArgs, Debug)]
pub struct Args {
    /// Package to build and flash. Defaults to `daisy-app-template`, or
    /// `daisy-boot` if `--bootloader` is set.
    #[arg(short, long)]
    package: Option<String>,

    /// Flash the bootloader to internal flash instead of the app to QSPI.
    #[arg(long)]
    bootloader: bool,

    /// Skip the build step; flash this ELF file directly. The image's
    /// entry point address is taken from the ELF's own program headers.
    #[arg(long, value_name = "PATH")]
    elf: Option<PathBuf>,

    /// Override the base address the image is written to. Rarely needed —
    /// the ELF already carries its target address via memory.x.
    #[arg(long, value_name = "ADDR", value_parser = parse_hex_u32)]
    address: Option<u32>,

    /// Release build (default). Pass `--profile dev` for a debug build.
    #[arg(long, default_value = "release")]
    profile: String,

    /// Cargo features to enable when building, e.g. `--features seed3` for the
    /// Seed 3 TAC5242 codec. Comma-separated or repeated; ignored with `--elf`.
    #[arg(long, value_delimiter = ',')]
    features: Vec<String>,

    /// Skip the startup-RAM-invariant check before flashing. Not recommended —
    /// the check refuses images whose `.bss`/`.data` init would fault on
    /// unmapped memory during startup (a `.bss`/`.data` range crossing a
    /// RAM-region gap).
    #[arg(long)]
    no_check: bool,
}

fn parse_hex_u32(s: &str) -> Result<u32, String> {
    let s = s.trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(s, 16).map_err(|e| format!("invalid hex address `{s}`: {e}"))
}

#[cfg(test)]
mod tests {
    use super::parse_hex_u32;

    #[test]
    fn parses_bare_hex() {
        assert_eq!(parse_hex_u32("9000_0000").ok(), None); // underscores rejected by u32::from_str_radix
        assert_eq!(parse_hex_u32("90000000"), Ok(0x9000_0000));
    }

    #[test]
    fn parses_0x_prefix() {
        assert_eq!(parse_hex_u32("0x08000000"), Ok(0x0800_0000));
        assert_eq!(parse_hex_u32("0X08000000"), Ok(0x0800_0000));
    }

    #[test]
    fn rejects_non_hex() {
        assert!(parse_hex_u32("xyz").is_err());
        assert!(parse_hex_u32("").is_err());
    }
}

pub fn run(args: Args) -> Result<()> {
    let elf_supplied = args.elf.is_some();
    let elf_path = match args.elf {
        Some(p) => p,
        None => {
            let pkg = args
                .package
                .clone()
                .unwrap_or_else(|| default_package(args.bootloader).to_string());
            crate::cmd::build::build_firmware(&pkg, &args.profile, &args.features)?
        }
    };

    // Refuse to flash an ELF whose startup RAM init would fault on unmapped
    // memory (a `.bss`/`.data` range crossing a RAM-region gap) — so the fault
    // never reaches a board.
    if !args.no_check {
        if let Ok(symbols) = elf::read_symbols(&elf_path) {
            if let Err(errs) = elf::check_startup_ram_invariant(&symbols) {
                eprintln!(
                    "\u{2717} {} — startup RAM invariant VIOLATED:",
                    elf_path.display()
                );
                for e in &errs {
                    eprintln!("    - {e}");
                }
                return Err(anyhow!(
                    "refusing to flash: this image's startup RAM init would fault on \
                     unmapped memory before `main` ({} issue(s)). Pass --no-check to override.",
                    errs.len()
                ));
            }
        }
    }

    let (elf_base, image) = elf::elf_to_bin(&elf_path)
        .with_context(|| format!("convert {} to bin", elf_path.display()))?;
    let address = args.address.unwrap_or(elf_base);

    println!(
        "Flashing {} bytes @ 0x{:08X} from {}",
        image.len(),
        address,
        elf_path.display()
    );

    // Sanity check: catch obvious address mixups (e.g. accidentally flashing
    // an app-template ELF to internal flash when --bootloader was intended).
    if args.bootloader && !(0x0800_0000..=0x0801_FFFF).contains(&address) {
        return Err(anyhow!(
            "--bootloader was set but image is linked for 0x{:08X}. \
             Bootloaders must live in internal flash (0x0800_0000..).",
            address
        ));
    }
    if !args.bootloader && !(0x9000_0000..=0x907F_FFFF).contains(&address) && !elf_supplied {
        return Err(anyhow!(
            "Image is linked for 0x{:08X} but the app is expected in QSPI \
             (0x9000_0000..). Pass --bootloader if this is the bootloader, \
             or --address to override.",
            address
        ));
    }

    dfu::download(address, &image)
}

fn default_package(bootloader: bool) -> &'static str {
    if bootloader {
        "daisy-boot"
    } else {
        "daisy-app-template"
    }
}
