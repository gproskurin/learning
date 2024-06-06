extern void _reset(void);
extern void _estack(void);

extern void IntHandler_Lptim1();

#if defined TARGET_STM32L072
__attribute__((section(".vectors"))) void (*tab[16+32])(void) =
{
	_estack,
	_reset,
	[16 + 13] = IntHandler_Lptim1,
};
#elif defined TARGET_STM32WL55_CPU1
__attribute__((section(".vectors"))) void (*tab[16+62])(void) =
{
	_estack,
	_reset,
	[16 + 39] = IntHandler_Lptim1,
};
#elif defined TARGET_STM32WL55_CPU2
__attribute__((section(".vectors"))) void (*tab[16+32])(void) =
{
	_estack,
	_reset,
	[16 + 13] = IntHandler_Lptim1,
};
#elif defined TARGET_STM32F103
__attribute__((section(".vectors"))) void (*tab[16+60])(void) =
{
	_estack,
	_reset,
};
#endif

