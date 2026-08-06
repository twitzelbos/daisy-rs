use anyhow::{anyhow, Context, Result};
use clap::Args as ClapArgs;
use std::path::PathBuf;
use std::process::Command;

const FIRMWARE_TARGET: &str = "thumbv7em-none-eabihf";

#[derive(ClapArgs, Debug)]
pub struct Args {
    /// Package to build. Defaults to building the whole firmware workspace.
    #[arg(short, long)]
    package: Option<String>,

    /// Cargo profile (dev / release / a custom profile from Cargo.toml).
    #[arg(long, default_value = "release")]
    profile: String,
}

pub fn run(args: Args) -> Result<()> {
    let mut cmd = Command::new("cargo");
    cmd.args(["build", "--target", FIRMWARE_TARGET]);
    cmd.args(["--profile", &args.profile]);
    if let Some(pkg) = args.package {
        cmd.args(["-p", &pkg]);
    } else {
        // Building `--workspace` would drag in daisy-cli which is host-only.
        // Enumerate firmware crates instead.
        for pkg in FIRMWARE_PACKAGES {
            cmd.args(["-p", pkg]);
        }
    }
    let status = cmd.status().context("spawn cargo")?;
    if !status.success() {
        return Err(anyhow!("cargo build failed with status {status}"));
    }
    Ok(())
}

/// Build one firmware package and return the path to its ELF artifact.
///
/// Used by the flash subcommand so `daisy flash` also builds. Consolidating
/// here keeps the target triple in exactly one place.
pub fn build_firmware(package: &str, profile: &str) -> Result<PathBuf> {
    let status = Command::new("cargo")
        .args([
            "build",
            "--target",
            FIRMWARE_TARGET,
            "--profile",
            profile,
            "-p",
            package,
        ])
        .status()
        .context("spawn cargo")?;
    if !status.success() {
        return Err(anyhow!("cargo build -p {package} failed with {status}"));
    }
    // Cargo names the profile directory after the profile except for `dev`
    // (which maps to `debug`). Translate here rather than call `cargo metadata`.
    let profile_dir = if profile == "dev" { "debug" } else { profile };
    let path = PathBuf::from("target")
        .join(FIRMWARE_TARGET)
        .join(profile_dir)
        .join(package);
    if !path.exists() {
        return Err(anyhow!(
            "expected artifact {} not found — is the binary name the same as the package?",
            path.display()
        ));
    }
    Ok(path)
}

const FIRMWARE_PACKAGES: &[&str] = &[
    "daisy-boot",
    "daisy-bsp",
    "daisy-audio",
    "daisy-dsp",
    "daisy-app-template",
];
