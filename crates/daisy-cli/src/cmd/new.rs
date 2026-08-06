use anyhow::{anyhow, Result};
use clap::Args as ClapArgs;
use std::path::PathBuf;

#[derive(ClapArgs, Debug)]
pub struct Args {
    /// Directory to create.
    pub path: PathBuf,
}

pub fn run(_args: Args) -> Result<()> {
    // TODO: copy the daisy-app-template contents into `_args.path`, rewrite
    // the package name in Cargo.toml, and add relative paths to daisy-bsp /
    // daisy-audio / daisy-dsp (or point at git tags once we publish).
    Err(anyhow!("`daisy new` is not implemented yet"))
}
