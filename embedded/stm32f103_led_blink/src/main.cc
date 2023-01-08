#include <stdbool.h>
#include <stdint.h>

#if defined TARGET_STM32F100
	#define GPIOB_BASE 0x40010C00
	#define GPIO_LED 12 // PB12
#elif defined TARGET_STM32L152
	#define GPIOA_BASE 0x40020000
	//#define GPIOB_BASE 0x40020400
	//#define GPIO_LED 13 // PB13
	#define GPIO_LED 5 // PA5
#else
	#error
#endif

// push-pull mode, output
#define CNF 0b00
#define MODE 0b10

#if defined TARGET_STM32F100
	#define RCC_BASE 0x40021000
	#define RCC_APB2ENR_offset 0x18
#elif defined TARGET_STM32L152
	#define RCC_BASE 0x40023800
	#define RCC_APBENR_offset 0x1C
#else
	#error
#endif


#if defined TARGET_STM32F100
typedef struct {
	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
} gpio_t;

#elif defined TARGET_STM32L152

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
} gpio_t;

#else
	#error
#endif


#if defined TARGET_STM32L152

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t _reserved_1;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t _reserved_2;
	volatile uint32_t _reserved_3;
	volatile uint32_t _reserved_4;
	volatile uint16_t CNT;
	volatile uint16_t _reserved_5;
	volatile uint16_t PSC;
	volatile uint16_t _reserved_6;
	volatile uint16_t ARR;
	volatile uint16_t _reserved_7;
} basic_timer_t;

#endif

void basic_timer_init(basic_timer_t* t, uint16_t prescaler, uint16_t timeout)
{
	t->PSC = prescaler;
	t->ARR = timeout;
}


#if defined TARGET_STM32F100
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

#elif defined TARGET_STM32L152

void gpio_set_mode(uint32_t gpio_base_addr, int reg, uint32_t mode)
{
	gpio_t* const gpio = (gpio_t*) gpio_base_addr;

	const uint32_t mask = mode << (reg * 2);
	const uint32_t mask_value = mode << (reg * 2);
	gpio->MODER = (gpio->MODER & ~mask) | mask_value;
}
#endif



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

#if defined TARGET_STM32F100
// enable APB2 for GPIO to work
void gpio_bus_enable()
{
	// byte access is supported
	volatile unsigned char* const RCC_APB2ENR = (unsigned char*) (RCC_BASE + RCC_APB2ENR_offset);
	*RCC_APB2ENR |= 0b1000; // enable port B
}

#elif defined TARGET_STM32L152
// enable APB for GPIO to work
void gpio_bus_enable()
{
	// byte access is supported
	volatile unsigned char* const RCC_APBENR = (unsigned char*) (RCC_BASE + RCC_APBENR_offset);
	*RCC_APBENR |= 0b01; // enable port A
}
#endif

int main()
{
	gpio_bus_enable();

#if defined TARGET_STM32F100
	gpio_set_mode(GPIOB_BASE, GPIO_LED, MODE, CNF);
#elif defined TARGET_STM32L152
	gpio_set_mode(GPIOA_BASE, GPIO_LED, 0b01);
#endif

	bool s = true;
	for (;;) {
		gpio_set(GPIOA_BASE, GPIO_LED, s);
		delay(s ? 100000 : 30000);
		s = !s;
	}
	return 0;
}

