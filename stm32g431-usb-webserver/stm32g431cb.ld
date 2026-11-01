ENTRY(Reset_Handler)
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 22K
    CCM (xrw)  : ORIGIN = 0x10000000, LENGTH = 10K
}
/* Keep the CPU stack in CCM. USB uses PMA, not DMA into CCM. */
_estack = ORIGIN(CCM) + 4096;
SECTIONS
{
    .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) } > FLASH
    .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); } > FLASH
    .ARM.extab : { *(.ARM.extab*) } > FLASH
    .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
    .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH
    .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH
    .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH
    _sidata = LOADADDR(.data);
    .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
    .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
    .noinit (NOLOAD) : { . = ALIGN(4); *(.noinit*) . = ALIGN(4); } > RAM
    /* lwIP uses its own static heap; there is no libc heap. */
    end = .; _end = .;
    .stack (NOLOAD) : { . = ALIGN(8); _stack_bottom = .; . += 4096; } > CCM
    .ccm (NOLOAD) : { . = ALIGN(8); *(.ccm*) . = ALIGN(8); } > CCM
    ASSERT(_ebss <= ORIGIN(RAM) + LENGTH(RAM), "SRAM overflow")
    ASSERT(SIZEOF(.stack) <= LENGTH(CCM), "CCM stack overflow")
}
