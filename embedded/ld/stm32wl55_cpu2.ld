ENTRY(_reset);
MEMORY {
  flash1(rx): ORIGIN = 0x08000000, LENGTH = 128k
  flash2(rx): ORIGIN = 0x08020000, LENGTH = 128k
  sram1(rwx) : ORIGIN = 0x20000000, LENGTH = 32k
  sram2(rwx) : ORIGIN = 0x20008000, LENGTH = 32k
}
_estack     = ORIGIN(sram2) + LENGTH(sram2);    /* stack points to end of SRAM */

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
  } > sram2 AT > flash2
  _sidata = LOADADDR(.data);

  .bss(NOLOAD) : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram2

  /* . = ALIGN(8); */
  /* _end = .; */    /* for cmsis_gcc.h  */
}
