ENTRY(_reset);
MEMORY {
  flash1(rx): ORIGIN = 0x08000000, LENGTH = 1024k
  /* flash2(rx): ORIGIN = 0x08100000, LENGTH = 1024k */
  sram_axi(rwx): ORIGIN = 0x24000000, LENGTH = 512k
  dtcm(rwx) : ORIGIN = 0x20000000, LENGTH = 128k
}
/* _estack     = ORIGIN(dtcm) + LENGTH(dtcm); */ /* doesn't work with DMA */
_estack     = ORIGIN(sram_axi) + LENGTH(sram_axi);

SECTIONS {
  .vectors  : {
    KEEP(*(.vectors))
  } > flash1

  .init_array : {
    _sinit_array = .;
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array)))
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array*)))
    _einit_array = .;
  } > flash1

  .text : {
    *(.text)
    *(.text*)
    . = ALIGN(4);
  } > flash1

  .rodata : {
    *(.rodata*)
  } > flash1

  .data : {
    _sdata = .;
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;
  } > sram_axi AT > flash1
  _sidata = LOADADDR(.data);

  .bss(NOLOAD) : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram_axi

  /* . = ALIGN(8); */
  /* _end = .; */    /* for cmsis_gcc.h  */
}
