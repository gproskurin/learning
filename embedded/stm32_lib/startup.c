#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern void main();


__attribute__((naked, noreturn)) void _reset(void)
{
	// zero bss
	extern long _sbss, _ebss;
	memset(&_sbss, 0, (const char*)&_ebss - (const char *)&_sbss);

	// copy data to sram
	extern long _sdata, _edata, _sidata;
	memcpy(&_sdata, &_sidata, (const char*)&_edata - (const char*)&_sdata);

	// copy text to sram
	//extern long _stext_flash_to_sram, _etext_flash_to_sram, _stext_sram;
	//for (long *src = &_stext_flash_to_sram, *dst = &_stext_sram; src < &_etext_flash_to_sram;)
	//	*src++ = *dst++;

	// call constructors of C++ static objects
	{
		extern void (*_sinit_array [])();
		extern void (*_einit_array [])();
		const size_t init_array_size = &(_einit_array[0]) - &(_sinit_array[0]);
		for (size_t i = 0; i < init_array_size; ++i) {
			_sinit_array[i]();
		}
	}

	main();
}

