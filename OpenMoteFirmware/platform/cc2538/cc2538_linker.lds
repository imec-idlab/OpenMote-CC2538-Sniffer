/*
 * Copyright 2013 OpenMote Technologies, S.L.
 */

/**
 * Indicate the architecture and format
 */
OUTPUT_ARCH(arm)
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")

/**
 * Indicate the entry point to the linker.
 */
ENTRY(reset_handler)

/**
 * Linker script for Texas Instruments CC2538SF53 (32K RAM, 512K Flash)
 * FLASH stores the program code
 * FLASH_CCA stores the backdoor bootloader configuration
 * SRAM consists of 16K NO-RETENTION and 16K RETENTION
 */
MEMORY
{
    FLASH     (RX)  : ORIGIN = 0x00200000, LENGTH = 512K
    FLASH_CCA (RX)  : ORIGIN = 0x0027FFD4, LENGTH = 44
    SRAM      (RWX) : ORIGIN = 0x20000000, LENGTH = 32K
}

SECTIONS
{
    /* Holds the code in FLASH */
    .text :
    {
        _text_start = .;
         /* Interrupt vector in Flash */
        . = ALIGN(4);
        KEEP(*(.vectors))
        
        . = ALIGN(4);
        *(.text*)
        *(.rodata*)
        
        . = ALIGN(4);
        KEEP(*(.init))

        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;

        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;

        . = ALIGN(4);
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*crtend.o(.ctors))

        . = ALIGN(4);
        KEEP(*(.fini))

        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;

        . = ALIGN(4);
        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*crtend.o(.dtors))
        
        _text_end = .;
    } > FLASH = 0

    /* Put UDMA Channel Control Table at start of SRAM to avoid large padding as address must be aligned on a 1024-byte boundary */
    .socdata (NOLOAD) :
    {
        *(.udma_channel_control_table)
    } > SRAM

    /* Holds the stack at the start of SRAM */
    .stack (NOLOAD) :
    {
        . = ALIGN(4);
        KEEP(*(.stack))
    } > SRAM

    /* Holds variables stored in FLASH and copied to SRAM upon initialization */
    .data :
    {
        . = ALIGN(4);
        _data_start = .;
        /* Interrupt vector in SRAM */
        *(vtable)
        *(.data*)
        _data_end = .;
    } > SRAM AT > FLASH

    /* Holds unitialized variables stored in RAM */
    .bss :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss*)
        *(COMMON)
        _bss_end = .;
    } > SRAM

    /* Hods the CCA section of the CC2538 */
    .flashcca :
    {
        . = ALIGN(4);
        KEEP(*(.flashcca))
    } > FLASH_CCA
    
    /* Contains information to unwind the stack for debugging purposes */
    PROVIDE_HIDDEN (__exidx_start = .);
    .ARM.exidx :
    {
        . = ALIGN(4);
        *(.ARM.exidx*)
    } > FLASH
    PROVIDE_HIDDEN (__exidx_end = .);
    
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
}
