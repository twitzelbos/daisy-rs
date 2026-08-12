/* pod-midi — Daisy Pod USB-MIDI input, standalone from internal flash. */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM */
}
