#include <stdint.h>
#include <string.h>


extern void main();
extern void _estack(void);
extern void IntHandler_Timer();


__attribute__((naked, noreturn)) void _reset(void)
{
	extern long _sbss, _ebss; // bss
	extern long _sdata, _edata, _sidata; // data
	//extern long _stext_flash_to_sram, _etext_flash_to_sram, _stext_sram; // text

	// zero bss
	//for (long *src = &_sbss; src < &_ebss; src++) *src = 0;
	memset(&_sbss, 0, (const char*)&_ebss - (const char *)&_sbss);

	// copy data to sram
	//for (long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;
	memcpy(&_sdata, &_sidata, (const char*)&_edata - (const char*)&_sdata);

	// copy text to sram
	//for (long *src = &_stext_flash_to_sram, *dst = &_stext_sram; src < &_etext_flash_to_sram;)
	//	*src++ = *dst++;

	main();
}

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

#if defined TARGET_STM32L152
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
	&vPortSVCHandler, // svc
	0,
	0,
	&xPortPendSVHandler, // PendSV
	&xPortSysTickHandler, // systick

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
	&IntHandler_Timer, // TIM9
	0,
	0, // TIM11 43 = 27 + 16
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
	0 // &IntHandler_Timer // TIM6 59 = 16 + 43
};
#elif defined TARGET_STM32F103
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
	&vPortSVCHandler, // svc
	0,
	0,
	&xPortPendSVHandler, // PendSV
	&xPortSysTickHandler, // systick

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
	&IntHandler_Timer, // TIM2
	0
};
#endif
