/* dsp-bench memory map — STM32H750, runs standalone from internal flash (no
 * bootloader / no QSPI), on the RESET clock (HSI 64 MHz, flash 0 wait states,
 * no cache). All DSP scratch fits in DTCM (the fastest, zero-wait, core-coupled
 * RAM — the cleanest core-bound baseline); the results array is a fixed DTCM
 * address the Renode test (and probe-rs on hardware) reads back. No custom
 * NOLOAD sections: a zero-initialised section in AXI would extend __ebss across
 * the DTCM→AXI hole and make cortex-m-rt's startup zero-loop run away. */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM: stack + .data/.bss + results */
}
