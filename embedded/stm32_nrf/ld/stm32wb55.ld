ENTRY(_reset);
MEMORY {
  /* FIXME */
  flash(rx): ORIGIN = 0x08000000, LENGTH = 32k
  sram1(rwx) : ORIGIN = 0x20000000, LENGTH = 20k
}
_estack     = ORIGIN(sram1) + LENGTH(sram1);    /* stack points to end of SRAM */

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
  } > sram1 AT > flash
  _sidata = LOADADDR(.data);

  .bss(NOLOAD) : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram1

  /* . = ALIGN(8); */
  /* _end = .; */    /* for cmsis_gcc.h  */
}
