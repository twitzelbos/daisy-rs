/* gap-exerciser — runs standalone from internal flash. Markers + stack in DTCM.
 * main() deliberately touches 0x2100_0000 (the unmapped DTCM→AXI gap) to verify
 * Renode's GapGuard bus-faults on it like silicon. */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM */
}
