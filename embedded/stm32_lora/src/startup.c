extern void _reset(void);
extern void _estack(void);

extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

//extern void IntHandler_EXTI23();
extern void IntHandler_EXTI4_15();
extern void IntHandler_DMA1_Channel4_5_6_7();
extern void IntHandler_DMA1_Channel2_3();


#if defined TARGET_STM32L072
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

	//[16 + 6] = IntHandler_EXTI23,
	[16 + 7] = IntHandler_EXTI4_15,
	[16 + 10] = IntHandler_DMA1_Channel2_3,
	[16 + 11] = IntHandler_DMA1_Channel4_5_6_7,
};
#endif

