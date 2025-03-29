extern void _reset();
extern void _estack();


#if defined TARGET_STM32L072

extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

extern void IntHandler_DMA1_Channel4_5_6_7();
extern void IntHandler_USB();

__attribute__((section(".vectors"))) void (*tab[16+32])(void) =
{
	_estack,
	_reset,
	[11] = &SVC_Handler,
	[14] = &PendSV_Handler,
	[15] = &SysTick_Handler,

	[16 + 11] = IntHandler_DMA1_Channel4_5_6_7,
	[16 + 31] = IntHandler_USB,
};

#elif defined TARGET_STM32H745_CM4

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

extern void IntHandler_Dma1S1(void);

__attribute__((section(".vectors"))) void (*tab[16 + 150])(void) =
{
	_estack,
	_reset,
	[11] = &vPortSVCHandler,
	[14] = &xPortPendSVHandler,
	[15] = &xPortSysTickHandler,

	[16 + 12] = IntHandler_Dma1S1,
};
#endif

