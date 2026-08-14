/* sdram-exerciser — runs standalone from internal flash (like the other
 * Renode exercisers). Markers + stack live in DTCM; the SDRAM window
 * (0xC0000000) is accessed by raw pointer after daisy_bsp::sdram::init(). */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM */
}
