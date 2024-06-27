extern void _reset(void);
extern void _estack(void);

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);


#if defined TARGET_NRF5340DK_APP
#define NVIC_SIZE (16+240)
#elif defined TARGET_NRF5340DK_NET
#define NVIC_SIZE (16+129)
#endif
__attribute__((section(".vectors"))) void (*tab[NVIC_SIZE])(void) =
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

