extern void _reset(void);
extern void _estack(void);


extern void IntHandler_Timer();


#if defined TARGET_STM32L072
extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);
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

	[16 + 25] = &IntHandler_Timer, // TIM9
	// [16 + 27] = 0, // TIM11
	// [16 + 43] = 0, // TIM6
};
#elif defined TARGET_STM32F103
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
__attribute__((section(".vectors"))) void (*tab[16+60])(void) =
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
	&vPortSVCHandler,
	0,
	0,
	&xPortPendSVHandler,
	&xPortSysTickHandler,

	[16 + 28] = &IntHandler_Timer, // TIM2
};
#endif

