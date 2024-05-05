extern void _reset(void);
extern void _estack(void);

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);


#if defined TARGET_STM32WL55_CPU1
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

	0
};
#endif

