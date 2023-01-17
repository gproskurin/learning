ENTRY(_reset);
MEMORY {
  flash(rx) : ORIGIN = 0x08000000, LENGTH = 512k
  sram(rwx) : ORIGIN = 0x20000000, LENGTH = 80k
}
_estack     = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */

SECTIONS {
  .vectors  : {
    KEEP(*(.vectors))
    . = ALIGN(0x200); /* skip remaining vectors. FIXME: better? */
  } > flash

  . = ALIGN(4);

  .text : {
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
  } > flash AT > sram
  _sidata = LOADADDR(.data);

  .bss(NOLOAD) : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram

  /* . = ALIGN(8); */
  /* _end = .; */    /* for cmsis_gcc.h  */
}
