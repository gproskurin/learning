extern void _reset(void);
extern void _estack(void);

extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);


#if defined TARGET_STM32L072
extern void IntHandler_DMA1_Channel2_3();
extern void IntHandler_DMA1_Channel4_5_6_7();
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

	[16 + 10] = IntHandler_DMA1_Channel2_3,
	[16 + 11] = IntHandler_DMA1_Channel4_5_6_7,
};
#endif

#if defined TARGET_STM32WL55_CPU1
extern void IntHandler_Dma1Ch1();
__attribute__((section(".vectors"))) void (*tab[16+62])(void) =
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

	[16+11] = IntHandler_Dma1Ch1,
};
#endif

