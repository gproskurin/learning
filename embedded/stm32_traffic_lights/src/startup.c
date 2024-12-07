extern void _reset(void);
extern void _estack(void);

extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);


#if defined TARGET_STM32G030
__attribute__((section(".vectors"))) void (*tab[16+32])(void) =
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
	&SVC_Handler,
	0,
	0,
	&PendSV_Handler,
	&SysTick_Handler,
};
#endif

