ENTRY(_reset);
MEMORY {
  sram5(rwx) : ORIGIN = 0x20040000, LENGTH = 256-4
}
/* _estack     = ORIGIN(sram5) + LENGTH(sram5); */
_estack = 0x20001000;

SECTIONS {
  .vectors  : {
    KEEP(*(.vectors))
  } > sram5

  .text : {
    *(.text)
    *(.text*)
    . = ALIGN(4);
  } > sram5

}
