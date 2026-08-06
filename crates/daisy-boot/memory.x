/* Bootloader memory map — STM32H750
 *
 * The H750 has 128 KiB of internal flash (one 128 KiB sector at 0x08000000)
 * and multiple SRAM regions. The bootloader lives entirely in internal flash
 * and executes from DTCM/AXI SRAM. The application is stored in QSPI at
 * 0x90000000 and executed via memory-mapped XIP after the bootloader
 * configures OCTOSPI and jumps to it.
 *
 * Refs: RM0433 §3.3 (flash organisation), §4.3 (SRAM), §22 (OCTOSPI).
 */
MEMORY
{
    FLASH  : ORIGIN = 0x08000000, LENGTH = 128K
    DTCM   : ORIGIN = 0x20000000, LENGTH = 128K
    RAM    : ORIGIN = 0x24000000, LENGTH = 512K
}

/* cortex-m-rt places .text/.rodata in FLASH and .bss/.data in RAM by default. */
REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", DTCM);
