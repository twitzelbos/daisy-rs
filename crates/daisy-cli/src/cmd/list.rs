use anyhow::Result;

use crate::dfu;

pub fn run() -> Result<()> {
    let devices = dfu::list()?;
    if devices.is_empty() {
        println!(
            "No Daisy in DFU mode found. Hold BOOT and press RESET, then run again.\n\
             Looking for VID:PID {:04x}:{:04x} (STM32 ROM bootloader).",
            dfu::ST_DFU_VID,
            dfu::ST_DFU_PID
        );
        return Ok(());
    }
    println!("DFU-mode devices:");
    for (i, d) in devices.iter().enumerate() {
        println!(
            "  [{i}] bus {}, address {}  serial={}",
            d.info.bus_id(),
            d.info.device_address(),
            d.serial.as_deref().unwrap_or("<none>"),
        );
    }
    Ok(())
}
