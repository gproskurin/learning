#include <stdint.h>


extern int main();
extern void _estack(void);


__attribute__((naked, noreturn)) void _reset(void)
{
	extern long _sbss, _ebss; // bss
	extern long _sdata, _edata, _sidata; // data
	//extern long _stext_flash_to_sram, _etext_flash_to_sram, _stext_sram; // text

	// zero bss
	for (long *src = &_sbss; src < &_ebss; src++) *src = 0;

	// copy data to sram
	for (long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;

	// copy text to sram
	//for (long *src = &_stext_flash_to_sram, *dst = &_stext_sram; src < &_etext_flash_to_sram;)
	//	*src++ = *dst++;

	main();
	for (;;);
}


__attribute__((section(".vectors"))) void (*tab[2 /*16 + 68*/])(void) =
{
	_estack, _reset
};

