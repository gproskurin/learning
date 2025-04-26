ENTRY(_reset);
MEMORY {
  flash1(rx): ORIGIN = 0x08000000, LENGTH = 1024k
  flash2(rx): ORIGIN = 0x08100000, LENGTH = 1024k
  sram1(rwx): ORIGIN = 0x30000000, LENGTH = 128k
  sram2(rwx): ORIGIN = 0x30020000, LENGTH = 128k
  sram3(rwx): ORIGIN = 0x30040000, LENGTH = 32k
  sram4(rwx): ORIGIN = 0x38000000, LENGTH = 64k
}
_estack     = ORIGIN(sram1) + LENGTH(sram1);

SECTIONS {
  .vectors  : {
    KEEP(*(.vectors))
  } > flash2

  .init_array : {
    _sinit_array = .;
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array)))
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array*)))
    _einit_array = .;
  } > flash2

  .text : {
    *(.text)
    *(.text*)
    . = ALIGN(4);
  } > flash2

  .rodata : {
    *(.rodata*)
  } > flash2

  .data : {
    _sdata = .;
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;
  } > sram1 AT > flash2
  _sidata = LOADADDR(.data);

  .bss(NOLOAD) : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram1

  /* . = ALIGN(8); */
  /* _end = .; */    /* for cmsis_gcc.h  */
}
