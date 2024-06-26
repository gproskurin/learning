extern void _reset(void);
extern void _estack(void);

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

extern void IntHandler_DMA1_Channel2_3();
extern void IntHandler_DMA1_Channel4_5_6_7();


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
	&vPortSVCHandler,
	0,
	0,
	&xPortPendSVHandler,
	&xPortSysTickHandler,

	[16 + 10] = IntHandler_DMA1_Channel2_3,
	[16 + 11] = IntHandler_DMA1_Channel4_5_6_7,
};
#endif

