extern void _reset(void);
extern void _estack(void);

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

extern void IntHandler_Dma1S0(void);


__attribute__((section(".vectors"))) void (*tab[16 + 150])(void) =
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

	[16 + 11] = IntHandler_Dma1S0,
};

