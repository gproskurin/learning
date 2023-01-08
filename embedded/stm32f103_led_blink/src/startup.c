extern int main();
extern void _estack(void);

__attribute__((naked, noreturn)) void _reset(void)
{
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *src = &_sbss; src < &_ebss; src++) *src = 0;
  for (long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;

  main();
  for (;;) (void) 0;
}


__attribute__((section(".vectors"))) void (*tab[2 /*16 + 68*/])(void) = {
	_estack, _reset
};

