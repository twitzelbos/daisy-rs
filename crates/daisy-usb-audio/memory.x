/* Application memory map — STM32H750, executing from QSPI XIP.
 *
 * The bootloader configures OCTOSPI in memory-mapped mode so the QSPI flash
 * (IS25LP064A, 8 MiB) appears at 0x90000000 as read-only executable memory.
 * DTCM/AXI SRAM hold runtime data; AXI SRAM is used because DTCM is too small
 * for typical audio buffers + DSP state.
 *
 * When the app grows, split .rodata off into a QSPI region and keep
 * hot-path DSP tables in AXI SRAM via `#[link_section = ".sram1"]`.
 */
/* Diagnostic: move the initial stack to DTCM. cortex-m-rt's default
 * `_stack_start = ORIGIN(RAM) + LENGTH(RAM)` points at the top of AXI
 * SRAM, and on hardware the very-first `push` in __pre_init produces a
 * PRECISERR/IMPRECISERR HardFault (data bus error) at that address.
 * DTCM is tightly-coupled memory on the CPU's private bus, always
 * clocked, never has any AXI arbitration concerns — if the fault goes
 * away with the stack on DTCM, we've localised the bug to AXI SRAM
 * access after the bootloader→app transition.
 */
MEMORY
{
    FLASH  : ORIGIN = 0x90000000, LENGTH = 8M
    RAM    : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM, used for stack */
    AXI    : ORIGIN = 0x24000000, LENGTH = 512K   /* AXI SRAM, unused for now */
    SDRAM  : ORIGIN = 0xC0000000, LENGTH = 64M    /* AS4C16M32MSA, live only after daisy_bsp::sdram::init() */
}

/* To place data in external SDRAM once daisy_bsp::sdram::init() has run,
 * add a NOLOAD output section mapped to the SDRAM region and zero/copy it
 * yourself AFTER init (cortex-m-rt's startup runs before SDRAM exists, so
 * it must not auto-initialise this section):
 *
 *   SECTIONS {
 *     .sdram (NOLOAD) : ALIGN(4) {
 *       *(.sdram .sdram.*);
 *     } > SDRAM
 *   } INSERT AFTER .bss;
 *
 * Reference a buffer with #[link_section = ".sdram"]. */

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);
