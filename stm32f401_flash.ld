/*
 ******************************************************************************
 * @file    stm32f401_flash.ld
 * @brief   Linker script for STM32F401CC
 *
 * Memory Layout for STM32F401CC:
 *   FLASH : 256KB (0x08000000 - 0x0803FFFF)
 *   RAM   : 64KB  (0x20000000 - 0x2000FFFF)
 ******************************************************************************
 */

MEMORY
{
    FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 256K
    RAM (rwx)       : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Define output sections */
SECTIONS
{
    /* ISR Vector table goes first in FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* Program code and read-only data */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /* ARM attributes */
    .ARM.attributes :
    {
        *(.ARM.attributes)
    } >FLASH

    /* Initialized data will be stored in FLASH and copied to RAM at startup */
    _sidata = LOADADDR(.data);

    /* Initialized data section */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT >FLASH

    /* Uninitialized data section (BSS) */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >RAM

    /* Provide end-of-RAM symbols required by newlib (sbrk) */
    . = ALIGN(4);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    PROVIDE(__end__ = .);

    /* Set stack top */
    _estack = ORIGIN(RAM) + LENGTH(RAM);

    .ARM.exidx :
    {
        *(.ARM.exidx*)
    } >FLASH
}
