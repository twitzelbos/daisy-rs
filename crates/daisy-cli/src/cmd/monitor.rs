use anyhow::{anyhow, Result};
use clap::{Args as ClapArgs, ValueEnum};

#[derive(ClapArgs, Debug)]
pub struct Args {
    #[arg(long, value_enum, default_value_t = Via::Serial)]
    via: Via,
}

#[derive(ValueEnum, Debug, Clone)]
pub enum Via {
    /// USB CDC-ACM serial exposed by the firmware.
    Serial,
    /// RTT via probe-rs (requires an ST-Link/CMSIS-DAP probe).
    Rtt,
}

pub fn run(_args: Args) -> Result<()> {
    // TODO: serialport-rs for CDC-ACM, probe-rs-rtt for RTT. Both are pure
    // Rust so they fit the "no C++" rule.
    Err(anyhow!("`daisy monitor` is not implemented yet"))
}
