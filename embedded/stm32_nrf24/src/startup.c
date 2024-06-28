extern void _reset(void);
extern void _estack(void);

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

extern void IntHandler_EXTI2();
extern void IntHandler_EXTI5_9();
extern void IntHandler_Dma1Ch1();


#if defined TARGET_STM32WB55
// 16+32 interrupts in cpu2 (M0+)
__attribute__((section(".vectors"))) void (*tab[16+63])(void) =
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

	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	IntHandler_EXTI2,
	0,
	0,
	IntHandler_Dma1Ch1,
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
	IntHandler_EXTI5_9,
	0
};
#endif

