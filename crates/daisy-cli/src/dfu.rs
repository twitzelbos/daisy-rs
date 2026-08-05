//! Thin wrapper over `nusb` + `dfu-nusb` for STM32 DFUse (ST's address-based
//! DFU extension). Handles device discovery, address setting, and the
//! blocking download.

use anyhow::{anyhow, Context, Result};
use dfu_nusb::DfuNusb;
use indicatif::{ProgressBar, ProgressStyle};
use nusb::MaybeFuture;
use std::time::Duration;

/// STM32's ROM DFU bootloader identifier.
pub const ST_DFU_VID: u16 = 0x0483;
pub const ST_DFU_PID: u16 = 0xDF11;

/// One connected DFU-capable device.
pub struct DfuDevice {
    pub info: nusb::DeviceInfo,
    pub serial: Option<String>,
}

/// Return all connected STM32 ROM-DFU devices.
pub fn list() -> Result<Vec<DfuDevice>> {
    let mut out = Vec::new();
    for info in nusb::list_devices().wait().context("enumerate USB")? {
        if info.vendor_id() == ST_DFU_VID && info.product_id() == ST_DFU_PID {
            let serial = info.serial_number().map(str::to_owned);
            out.push(DfuDevice { info, serial });
        }
    }
    Ok(out)
}

/// Download `image` to `address` on the first DFU device found. Blocks the
/// current thread on a single-threaded tokio runtime for the async
/// discovery/handshake, then uses the sync DFU API for the actual transfer
/// so we can drive an indicatif progress bar.
pub fn download(address: u32, image: &[u8]) -> Result<()> {
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .context("build tokio runtime")?;

    let dfu = rt.block_on(async {
        let info = nusb::list_devices()
            .await
            .context("enumerate USB")?
            .find(|d| d.vendor_id() == ST_DFU_VID && d.product_id() == ST_DFU_PID)
            .ok_or_else(|| {
                anyhow!(
                    "no STM32 DFU device found (VID:PID {:04x}:{:04x}). \
                     Hold BOOT and press RESET on the Daisy, then re-run.",
                    ST_DFU_VID,
                    ST_DFU_PID
                )
            })?;
        let device = info.open().await.context("open USB device")?;
        let interface = device
            .claim_interface(0)
            .await
            .context("claim DFU interface 0")?;
        DfuNusb::open(device, interface, 0)
            .await
            .context("initialise DFU on interface 0 / alt 0")
    })?;

    let mut sync = dfu.into_sync_dfu();
    sync.override_address(address);

    let pb = ProgressBar::new(image.len() as u64);
    pb.set_style(
        ProgressStyle::with_template(
            "  {msg:<8} {bar:32.cyan/blue} {bytes:>8}/{total_bytes:<8} {eta}",
        )
        .unwrap()
        .progress_chars("=> "),
    );
    pb.set_message(format!("0x{:08X}", address));
    pb.enable_steady_tick(Duration::from_millis(100));

    let pb_clone = pb.clone();
    sync.with_progress(move |written| {
        pb_clone.inc(written as u64);
    });

    sync.download_from_slice(image)
        .map_err(|e| anyhow!("DFU download failed: {e}"))?;

    pb.finish_with_message("done");
    Ok(())
}
