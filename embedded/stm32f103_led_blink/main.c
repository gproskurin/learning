#include <stdbool.h>
#include <stdint.h>

#define GPIOB_BASE 0x40010C00
#define GPIO_LED 12 // PB12

// push-pull mode, output
#define CNF 0b00
#define MODE 0b10

#define RCC_BASE 0x40021000
#define RCC_APB2ENR_offset 0x18


typedef struct {
	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
} gpio_t;


void gpio_set_mode(uint32_t gpio_base_addr, int reg, uint32_t mode, uint32_t cnf)
{
	gpio_t* const gpio = (gpio_t*) gpio_base_addr;

	const uint32_t reg_lo = ((reg < 8) ? reg : reg - 8);
	uint32_t bits = (cnf << 2) | mode;
	const uint32_t mask = 0b1111 << (reg_lo * 4);
	const uint32_t mask_value = bits << (reg_lo * 4);
	volatile uint32_t* const cr = ((reg < 8) ? &gpio->CRL : &gpio->CRH);
	*cr = (*cr & ~mask) | mask_value;
}


void gpio_set(uint32_t gpio_base_addr, int reg, bool high)
{
	uint32_t const mask = (high ? (1U << reg) : (1U << reg) << 16);
	((gpio_t*) gpio_base_addr)->BSRR = mask;
}


void delay(int val)
{
	for (volatile int i = 0; i<val; ++i) {
	}
}

// enable APB2 for GPIO to work
void gpio_bus_enable()
{
	// byte access is supported
	volatile unsigned char* const RCC_APB2ENR = (unsigned char*) (RCC_BASE + RCC_APB2ENR_offset);
	*RCC_APB2ENR |= 0b1000; // enable port B
}


int main()
{
	gpio_bus_enable();

	gpio_set_mode(GPIOB_BASE, GPIO_LED, MODE, CNF);

	bool s = true;
	for (;;) {
		gpio_set(GPIOB_BASE, GPIO_LED, s);
		delay(s ? 500000 : 10000);
		s = !s;
	}

}


__attribute__((naked, noreturn)) void _reset(void) {
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *src = &_sbss; src < &_ebss; src++) *src = 0;
  for (long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;

  main();
  for (;;) (void) 0;
}

extern void _estack(void);

__attribute__((section(".vectors"))) void (*tab[2 /*16 + 68*/])(void) = {
	_estack, _reset
};

