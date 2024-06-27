/**/

ENTRY(_reset);

MEMORY
{
  flash (rx) : ORIGIN = 0x00000000, LENGTH = 0x80000
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x10000
  code_ram (rwx) : ORIGIN = 0x800000, LENGTH = 0x10000
}

_estack     = ORIGIN(ram) + LENGTH(ram);    /* stack points to end of SRAM */

SECTIONS {
  .vectors  : {
    KEEP(*(.vectors))
  } > flash

  .init_array : {
    _sinit_array = .;
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array)))
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array*)))
    _einit_array = .;
  } > flash

  .text : {
    *(.text)
    *(.text*)
    . = ALIGN(4);
  } > flash

  .rodata : {
    *(.rodata*)
  } > flash

  .data : {
    _sdata = .;
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;
  } > ram AT > flash
  _sidata = LOADADDR(.data);

  .bss(NOLOAD) : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > ram

  /* . = ALIGN(8); */
  /* _end = .; */    /* for cmsis_gcc.h  */
}
