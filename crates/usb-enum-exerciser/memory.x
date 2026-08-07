/* usb-init-exerciser memory map — STM32H750, standalone from internal flash.
 * DTCM markers at 0x2001_0000 (stage) and 0x2001_0004 (poll count). */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM */
}
