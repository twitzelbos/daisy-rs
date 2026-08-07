/* adc-exerciser memory map — STM32H750, standalone from internal flash (no
 * bootloader / no QSPI). The Renode test reads back DTCM markers at 0x2001_0000
 * (progress) and 0x2001_0004 (ADC result); both sit above .data/.bss and below
 * the stack top. */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM */
}
