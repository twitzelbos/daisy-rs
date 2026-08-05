"""
Renode Robot Framework helpers for the Daisy Seed platform.

Adds two capabilities Renode's stock library doesn't cover for our tests:

  1. `Soft Reset` — resets the machine while preserving Backup SRAM contents.
     Real STM32 hardware retains the backup domain across SYSRESETREQ if
     VBAT is present, but Renode's Memory.MappedMemory zeros on machine
     reset. We snapshot the region into a Python variable, do the reset,
     and restore.

  2. `Backup Sram Read/Write Word` — small conveniences so tests can plant
     a "stay in bootloader" magic value from the outside, simulating what
     an app would do before requesting reset.

Load with:

  Library    ${CURDIR}/daisy_seed_helpers.py
"""

# STM32H750 Backup SRAM region.
BACKUP_SRAM_BASE = 0x38800000
BACKUP_SRAM_SIZE = 0x1000  # 4 KiB


def _sysbus(monitor):
    return monitor.Machine["sysbus"]


def _snapshot_backup_sram(monitor):
    bus = _sysbus(monitor)
    return [bus.ReadByte(BACKUP_SRAM_BASE + i) for i in range(BACKUP_SRAM_SIZE)]


def _restore_backup_sram(monitor, snapshot):
    bus = _sysbus(monitor)
    for i, byte in enumerate(snapshot):
        bus.WriteByte(BACKUP_SRAM_BASE + i, byte)


def soft_reset(monitor):
    """Reset the machine but preserve Backup SRAM, simulating a soft reset
    on real hardware where VBAT keeps the backup domain alive across
    SYSRESETREQ."""
    snapshot = _snapshot_backup_sram(monitor)
    monitor.Parse("machine Reset")
    _restore_backup_sram(monitor, snapshot)


def backup_sram_write_word(monitor, offset, value):
    """Write a 32-bit word into Backup SRAM. `offset` is bytes from the
    region base (0x38800000). Robot passes value as a string, so accept
    hex-prefixed forms too."""
    if isinstance(value, str):
        value = int(value, 0)
    if isinstance(offset, str):
        offset = int(offset, 0)
    _sysbus(monitor).WriteDoubleWord(BACKUP_SRAM_BASE + offset, value)


def backup_sram_read_word(monitor, offset):
    if isinstance(offset, str):
        offset = int(offset, 0)
    return _sysbus(monitor).ReadDoubleWord(BACKUP_SRAM_BASE + offset)
