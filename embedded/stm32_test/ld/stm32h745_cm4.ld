ENTRY(_reset);
MEMORY {
  flash1(rx): ORIGIN = 0x08000000, LENGTH = 1024k
  flash2(rx): ORIGIN = 0x08100000, LENGTH = 1024k
  sram1(rwx): ORIGIN = 0x24000000, LENGTH = 256k
  sram2(rwx): ORIGIN = 0x24040000, LENGTH = 384k
  sram3(rwx): ORIGIN = 0x240A0000, LENGTH = 384k
}
_estack     = ORIGIN(dtcm) + LENGTH(dtcm);    /* stack points to end of DTCM */

SECTIONS {
  .vectors  : {
    KEEP(*(.vectors))
    . = 0x200; /* skip remaining vectors. FIXME: better? */
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
  } > dtcm AT > flash1
  _sidata = LOADADDR(.data);

  .bss(NOLOAD) : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > dtcm

  /* . = ALIGN(8); */
  /* _end = .; */    /* for cmsis_gcc.h  */
}
