MEMORY {
    /*
     * gen4-RP2350-70CT-CLB ships with 16 MiB external QSPI flash (W25Q128).
     */
    FLASH : ORIGIN = 0x10000000, LENGTH = 16384K
    RAM : ORIGIN = 0x20000000, LENGTH = 512K
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
    /* APS6404L PSRAM on QMI CS1 — used for Graphics4D DMA bounce buffers at link time. */
    PSRAM : ORIGIN = 0x11000000, LENGTH = 8192K
}

SECTIONS {
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH

} INSERT AFTER .vector_table;

_stext = ADDR(.start_block) + SIZEOF(.start_block);

SECTIONS {
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
    } > FLASH

} INSERT AFTER .uninit;

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);

/* Pico SDK / libgcc references when linking Graphics4D static lib */
PROVIDE(__exidx_start = 0);
PROVIDE(__exidx_end = 0);

/* Graphics4D RGB DMA bounce buffers (~125 KiB each) — keep out of 512 KiB SRAM. */
SECTIONS {
    .graphics4d_bounce (NOLOAD) : ALIGN(4) {
        *(.bss._ZL12bounce_buff0)
        *(.bss._ZL12bounce_buff1)
        *(.bss._ZZN15GraphicsMedia4D16LoadImageControlEPKcjE3fil)
    } > PSRAM

    /* Graphics4D.cpp.o remaining static state */
    .graphics4d_bss (NOLOAD) : ALIGN(4) {
        KEEP(*(COMMON))
        *libgraphics4d_rp2350_embassy.a:Graphics4D.cpp.o(.bss .bss.*)
    } > PSRAM
}
