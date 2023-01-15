#include <stdint.h>


extern int main();
extern void _estack(void);
extern void IntHandler_Tim6();


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


__attribute__((section(".vectors"))) void (*tab[])(void) =
{
	_estack,
	_reset,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	&IntHandler_Tim6 // 59 = 16 + 43
};

