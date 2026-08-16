/* dma-cache-exerciser — runs standalone from internal flash. The cacheable
 * buffer under test (BUF) plus the DMA source/sink live at 0x3000_0000 (D2
 * SRAM), reachable by DMA1 and cacheable + write-back under the default memory
 * map. Markers and the stack are in DTCM (0x2000_0000), which is tightly
 * coupled and NEVER cached — so a probe-rs read of a marker is always coherent
 * with what the CPU wrote, exactly what the read-out relies on. */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM */
}
