use anyhow::Result;
use clap::{Parser, Subcommand};

mod cmd;
mod dfu;
mod elf;

/// Host-side dev tool for the daisy-rs environment.
#[derive(Parser, Debug)]
#[command(
    name = "daisy",
    version,
    about = "Build, flash, and monitor Daisy Seed firmware written entirely in Rust."
)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Create a new Daisy Rust project from the template.
    New(cmd::new::Args),
    /// Build the workspace or a specific package for the Daisy target.
    Build(cmd::build::Args),
    /// Flash a firmware image to a connected Daisy over DFU.
    Flash(cmd::flash::Args),
    /// Static memory-safety checks on a firmware ELF (startup RAM invariant).
    CheckElf(cmd::check::Args),
    /// Enumerate connected Daisy devices (DFU + run mode).
    List,
    /// Stream logs from a running Daisy over USB serial (or RTT via probe).
    Monitor(cmd::monitor::Args),
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::New(a) => cmd::new::run(a),
        Cmd::Build(a) => cmd::build::run(a),
        Cmd::Flash(a) => cmd::flash::run(a),
        Cmd::CheckElf(a) => cmd::check::run(a),
        Cmd::List => cmd::list::run(),
        Cmd::Monitor(a) => cmd::monitor::run(a),
    }
}
