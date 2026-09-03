/*
 * Flat linker script for the bare-metal core1 image. Link address equals
 * load address: core0 copies this image's raw bytes verbatim into place
 * before launching core1, so only .bss needs runtime zeroing.
 *
 * Region must match the reserved SRAM carve-out in the app's devicetree
 * overlay (top 128KB of SRAM, 0x20062000-0x20082000).
 */
MEMORY
{
    CORE1_RAM (rwx) : ORIGIN = 0x20062000, LENGTH = 128K
}

ENTRY(core1_reset_handler)

SECTIONS
{
    .vectors : {
        . = ALIGN(512);
        KEEP(*(.vectors))
    } > CORE1_RAM

    .text : {
        *(.text*)
    } > CORE1_RAM

    .rodata : {
        *(.rodata*)
    } > CORE1_RAM

    .data : {
        *(.data*)
    } > CORE1_RAM

    .bss (NOLOAD) : {
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        __bss_end__ = .;
    } > CORE1_RAM

    . = ALIGN(8);
    __core1_stack_top = ORIGIN(CORE1_RAM) + LENGTH(CORE1_RAM);

    /DISCARD/ : { *(.comment) *(.ARM.attributes) }
}
