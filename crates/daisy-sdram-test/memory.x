/* SDRAM March-test application memory map — STM32H750, executing from QSPI XIP.
 *
 * The bootloader configures OCTOSPI memory-mapped mode so the QSPI flash appears
 * at 0x90000000 as read-only executable memory. The stack lives in DTCM (never
 * cached, always clocked, no AXI arbitration) — the app-template proved the
 * initial `push` in __pre_init bus-faults with the stack at the top of AXI SRAM.
 *
 * The external SDRAM (0xC0000000, 64 MiB) is declared here for reference but is
 * NOT placed into by the linker — this app accesses it directly by raw pointer
 * after `daisy_bsp::sdram::init()` brings the FMC controller up, and tests it
 * non-cacheably. cortex-m-rt's startup must never auto-init it (it runs before
 * the SDRAM exists), so there is deliberately no `.sdram` output section.
 */
MEMORY
{
    FLASH  : ORIGIN = 0x90000000, LENGTH = 8M
    RAM    : ORIGIN = 0x20000000, LENGTH = 128K   /* DTCM, used for stack */
    AXI    : ORIGIN = 0x24000000, LENGTH = 512K   /* AXI SRAM, unused */
    SDRAM  : ORIGIN = 0xC0000000, LENGTH = 64M    /* AS4C16M32SB-6BCN, live only after sdram::init() */
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);
