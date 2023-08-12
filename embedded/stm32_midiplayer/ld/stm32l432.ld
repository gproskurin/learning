ENTRY(_reset);
MEMORY {
  flash(rx): ORIGIN = 0x08000000, LENGTH = 256k
  sram(rwx): ORIGIN = 0x20000000, LENGTH = 64k
}
_estack     = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */

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
  } > sram AT > flash
  _sidata = LOADADDR(.data);

  .bss(NOLOAD) : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram

  /* . = ALIGN(8); */
  /* _end = .; */    /* for cmsis_gcc.h  */
}
