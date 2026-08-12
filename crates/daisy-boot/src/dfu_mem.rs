//! `DFUMemIO` implementation backing our USB DFU service with the Daisy's
//! IS25LP064A QSPI flash.
//!
//! The `usbd-dfu` crate drives the DFU state machine and the DFUse
//! address-pointer extension for us; we plug in the actual flash operations.
//! Every `DFUMemIO` method runs in USB interrupt context, so we keep them
//! small: `store_write_buffer` just memcpys, and `erase`/`program` delegate
//! to `qspi.rs` where the busy-waits live. The whole thing is single-threaded
//! at the CPU level — `usbd-dfu` guarantees no calls overlap.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::hal::pac;
use crate::qspi;
use usbd_dfu::{DFUManifestationError, DFUMemError, DFUMemIO};

/// Set the first time a host issues a real flash-mutating DFU operation this
/// boot (buffer store, erase, or program). The boot-window loop in `main.rs`
/// keys "does a host actually want to flash?" off THIS — not the DFUse address
/// pointer. A host merely *enumerating* the DFU interface (notably macOS, which
/// has no DFU driver) was observed to move the address pointer and pin us in
/// service mode, so the app never booted while plugged into a Mac. Enumeration
/// never calls the write paths below, so it can never set this.
static DFU_SAW_WRITE: AtomicBool = AtomicBool::new(false);

/// True once a host has issued a real flash write over DFU this boot.
pub fn dfu_saw_write() -> bool {
    DFU_SAW_WRITE.load(Ordering::Relaxed)
}

/// IS25LP064A layout constants, denormalised here so the trait's `const`s
/// stay `const`-evaluable.
const FLASH_BASE: u32 = 0x9000_0000;
const FLASH_SIZE: u32 = 8 * 1024 * 1024;
const FLASH_END: u32 = FLASH_BASE + FLASH_SIZE;

/// Transfer size used for DFU download blocks. Must be `<=` the
/// `usb-device` control-endpoint buffer size, and small enough that the
/// worst-case program time in `PROGRAM_TIME_MS` still bounds it. 128 is
/// the crate default and works with dfu-util / dfu-nusb out of the box.
///
/// TODO: raise to 2048 once we increase the control endpoint buffer,
/// bringing whole-chip flash time from ~10 min down to under a minute.
const TRANSFER_SIZE: u16 = 128;

pub struct QspiDfuMem {
    qspi: pac::QUADSPI,
    buffer: [u8; TRANSFER_SIZE as usize],
    buffer_len: usize,
    /// True while the QUADSPI peripheral is currently in memory-mapped mode.
    /// We enter this mode lazily on the first `read()` and stay in it
    /// across successive reads to avoid the ~1 ms per-call settling delay,
    /// which caused USB control-transfer timeouts. `program()`/`erase()`
    /// drop back to indirect mode when needed.
    in_mm_mode: bool,
}

impl QspiDfuMem {
    /// Take ownership of QUADSPI, recover the flash to a clean single-line
    /// state (so indirect erase/program are framed correctly), and prepare an
    /// empty write buffer.
    pub fn new(qspi: pac::QUADSPI) -> Self {
        qspi::recover_flash_to_single_line(&qspi);
        Self {
            qspi,
            buffer: [0; TRANSFER_SIZE as usize],
            buffer_len: 0,
            in_mm_mode: false,
        }
    }

    fn ensure_mm_mode(&mut self) {
        if !self.in_mm_mode {
            qspi::enter_memory_mapped(&self.qspi);
            self.in_mm_mode = true;
        }
    }

    fn ensure_indirect_mode(&mut self) {
        if self.in_mm_mode {
            qspi::recover_flash_to_single_line(&self.qspi);
            self.in_mm_mode = false;
        }
    }

    /// Release the QUADSPI peripheral without touching mode. Callers that
    /// want memory-mapped mode back must invoke `qspi::enter_memory_mapped`
    /// themselves.
    pub fn release(self) -> pac::QUADSPI {
        self.qspi
    }

    fn contains(addr: u32) -> bool {
        (FLASH_BASE..FLASH_END).contains(&addr)
    }

    fn contains_range(addr: u32, len: u32) -> bool {
        (FLASH_BASE..=FLASH_END).contains(&addr) && len <= FLASH_END - addr
    }
}

impl DFUMemIO for QspiDfuMem {
    const INITIAL_ADDRESS_POINTER: u32 = FLASH_BASE;

    // 2048 sectors × 4 KiB each = 8 MiB. `g` = readable, erasable, writable.
    const MEM_INFO_STRING: &'static str = "@External Flash /0x90000000/2048*04Kg";

    const HAS_DOWNLOAD: bool = true;
    // Upload (device -> host reads) enables byte-exact verification of
    // what actually landed in QSPI flash after a DFU download. `read()`
    // re-enters memory-mapped mode, copies out of the XIP window, and
    // drops back to indirect so subsequent program/erase still works.
    const HAS_UPLOAD: bool = true;
    // We stay in DFU mode after each flash so the host can send more
    // firmware without a reset. The service-mode loop in main.rs decides
    // when to reboot into the app.
    const MANIFESTATION_TOLERANT: bool = true;

    // IS25LP064A page program: typ 240 µs, max 750 µs.
    const PROGRAM_TIME_MS: u32 = 2;
    // 4 KiB sub-sector erase: typ 45 ms, max 300 ms.
    const ERASE_TIME_MS: u32 = 50;
    // Chip erase: typ 40 s, max 90 s. We report the max plus slack so the
    // host doesn't time out.
    const FULL_ERASE_TIME_MS: u32 = 95_000;

    const TRANSFER_SIZE: u16 = TRANSFER_SIZE;

    fn store_write_buffer(&mut self, src: &[u8]) -> Result<(), ()> {
        DFU_SAW_WRITE.store(true, Ordering::Relaxed);
        if src.len() > self.buffer.len() {
            return Err(());
        }
        self.buffer[..src.len()].copy_from_slice(src);
        self.buffer_len = src.len();
        Ok(())
    }

    fn read(&mut self, address: u32, length: usize) -> Result<&[u8], DFUMemError> {
        if !Self::contains_range(address, length as u32) {
            return Err(DFUMemError::Address);
        }
        let length = length.min(self.buffer.len());
        self.ensure_mm_mode();
        unsafe {
            let src = address as *const u8;
            for i in 0..length {
                self.buffer[i] = core::ptr::read_volatile(src.add(i));
            }
        }
        Ok(&self.buffer[..length])
    }

    fn program(&mut self, address: u32, length: usize) -> Result<(), DFUMemError> {
        DFU_SAW_WRITE.store(true, Ordering::Relaxed);
        if !Self::contains_range(address, length as u32) {
            return Err(DFUMemError::Address);
        }
        if length > self.buffer_len {
            return Err(DFUMemError::Prog);
        }
        self.ensure_indirect_mode();

        // Program page-by-page. Each Page Program command targets at most
        // one 256-byte page and must not cross a page boundary. The DFU
        // block (up to TRANSFER_SIZE) may span multiple pages.
        let flash_offset = address - FLASH_BASE;
        let mut written = 0usize;
        while written < length {
            let this_flash = flash_offset + written as u32;
            // Distance to the next page boundary in the flash.
            let page_bound = qspi::PAGE_SIZE as u32 - (this_flash & (qspi::PAGE_SIZE as u32 - 1));
            let this_len = core::cmp::min(page_bound as usize, length - written);
            qspi::program_page(
                &self.qspi,
                this_flash,
                &self.buffer[written..written + this_len],
            );
            written += this_len;
        }
        Ok(())
    }

    fn erase(&mut self, address: u32) -> Result<(), DFUMemError> {
        DFU_SAW_WRITE.store(true, Ordering::Relaxed);
        if !Self::contains(address) {
            return Err(DFUMemError::Address);
        }
        self.ensure_indirect_mode();
        qspi::erase_sector_4k(&self.qspi, address - FLASH_BASE);
        Ok(())
    }

    fn erase_all(&mut self) -> Result<(), DFUMemError> {
        DFU_SAW_WRITE.store(true, Ordering::Relaxed);
        // No dedicated chip-erase command wired yet; sweep sectors.
        // At ~50 ms/sector × 2048 sectors this is ~100 s.
        // TODO: implement 0xC7 Chip Erase for a ~40 s path.
        self.ensure_indirect_mode();
        let sector_size = qspi::SECTOR_SIZE;
        let mut offset = 0u32;
        while offset < FLASH_SIZE {
            qspi::erase_sector_4k(&self.qspi, offset);
            offset += sector_size;
        }
        Ok(())
    }

    fn manifestation(&mut self) -> Result<(), DFUManifestationError> {
        self.ensure_mm_mode();
        Ok(())
    }
}
