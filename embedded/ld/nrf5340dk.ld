/**/

ENTRY(_reset);

MEMORY
{
  flash (rx) : ORIGIN = 0x00000000, LENGTH = 0x100000
  /* extflash (rx) : ORIGIN = 0x10000000, LENGTH = 0x8000000 */
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x40000
  /* ram1 (rwx) : ORIGIN = 0x20040000, LENGTH = 0x3F000 */
}

_estack     = ORIGIN(ram) + LENGTH(ram);    /* stack points to end of SRAM */

SECTIONS {
  .vectors  : {
    KEEP(*(.vectors))
    . = 0x200; /* skip remaining vectors. FIXME: better? */
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
