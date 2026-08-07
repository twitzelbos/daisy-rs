/* Fault-exerciser memory map — STM32H750, runs standalone from internal
 * flash (no bootloader / no QSPI). Marker area is a fixed DTCM address
 * (0x2001F000) the Renode test reads back; it sits well above the tiny
 * .data/.bss and below the stack top, so nothing else touches it. */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM */
}
