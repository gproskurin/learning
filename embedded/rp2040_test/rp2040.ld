MEMORY {
  xip(rwx) : ORIGIN = 0x10000000, LENGTH = 16M
  sram4(rwx) : ORIGIN = 0x20040000, LENGTH = 256-4
  sram5(rwx) : ORIGIN = 0x20041000, LENGTH = 256-4
}
_estack = 0x20001000;

SECTIONS {
    /*
  .vectors  : {
    KEEP(*(.vectors))
  } > sram5 AT > xip
  */

  .text : {
    *(.text)
    *(.text*)
    . = ALIGN(4);
  } > sram5 AT > xip

}
