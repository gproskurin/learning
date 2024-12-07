extern void _reset();
extern void _estack();

extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

#if defined TARGET_STM32L072
__attribute__((section(".vectors"))) void (*tab[16+32])(void) =
{
	_estack,
	_reset,
	[11] = &SVC_Handler,
	[14] = &PendSV_Handler,
	[15] = &SysTick_Handler,
};
#endif

