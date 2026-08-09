/* cache-coherency-exerciser — runs standalone from internal flash. The shared
 * buffer lives at 0x3000_0000 (D2 SRAM), which the Renode CacheCoherencyChecker
 * overlays; markers/stack are in DTCM. */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM */
}
