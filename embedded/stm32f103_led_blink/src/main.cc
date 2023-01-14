//

#if defined TARGET_STM32L152
#include <cmsis_device_l1/Include/stm32l152xe.h>
#endif

#include <stdbool.h>
#include <stdint.h>


#if defined TARGET_STM32F100
	#define GPIOB_BASE 0x40010C00
	#define GPIO_LED 12 // PB12
#elif defined TARGET_STM32L152
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
#else
	#error
#endif

#if defined (TARGET_STM32L152)
#define NVIC_ADDR_TIM6 0xEC
#define NVIC_POS_TIM6 43
#else
#error
#endif


void basic_timer_init(TIM_TypeDef* const tim, uint16_t prescaler, uint16_t arr)
{
	//tim->EGR |= TIM_EGR_UG; // remove
	tim->PSC = prescaler;
	tim->ARR = arr;
	//tim->DIER |= TIM_DIER_UIE; // enable hardware interrupt
	tim->CR1 = (tim->CR1 & ~(TIM_CR1_UDIS | TIM_CR1_OPM)) | TIM_CR1_CEN;
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

void gpio_set_mode(GPIO_TypeDef* const gpio, int reg, uint32_t mode)
{
	const uint32_t mask = mode << (reg * 2);
	const uint32_t mask_value = mode << (reg * 2);
	gpio->MODER = (gpio->MODER & ~mask) | mask_value;
}
#endif


void gpio_set(GPIO_TypeDef* const gpio, int reg, bool high)
{
	uint32_t const mask = (high ? (1U << reg) : (1U << reg) << 16);
	gpio->BSRR = mask;
}


void delay(int val)
{
	for (volatile int i = 0; i<val; ++i) {
	}
}

volatile bool led_high = false;

void toggle_led()
{
	const bool high = led_high;
	gpio_set(GPIOA, GPIO_LED, high);
	led_high = !high;
}

void IntHandler_Tim6()
{
	TIM_TypeDef* const pt = TIM6;
	if (pt->SR & TIM_SR_UIF) {
		pt->SR &= ~TIM_SR_UIF;
		toggle_led();
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
void gpio_bus_enable()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN_Msk; // enable port A
	//RCC->APB1RSTR = RCC_APB1RSTR_TIM6RST_Msk; // reset TIM6
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN_Msk; // enable TIM6 // pos=43 prio=50
}

extern "C" void (*tab[2])(void);
void nvic_init_tim6()
{
	// set interrupt handler
	*(volatile uint32_t*) NVIC_ADDR_TIM6 = (uintptr_t) &IntHandler_Tim6;
	tab[43] = &IntHandler_Tim6;

	// enable interrupt
	//NVIC->ISER[1] = 1U << (NVIC_POS_TIM6 - 32);
	NVIC_SetPriority(TIM6_IRQn, 3);
	NVIC_EnableIRQ(TIM6_IRQn);
}

#endif

void blink(int n)
{
	while (n-- > 0) {
		gpio_set(GPIOA, GPIO_LED, 1);
		delay(50000);
		gpio_set(GPIOA, GPIO_LED, 0);
		delay(50000);
	}
}


int main()
{
	gpio_bus_enable();

#if defined TARGET_STM32F100
	gpio_set_mode(GPIOB_BASE, GPIO_LED, MODE, CNF);
#elif defined TARGET_STM32L152
	gpio_set_mode(GPIOA, GPIO_LED, 0b01);
#endif
	blink(1); delay(300000);

	//nvic_init_tim6();
	//blink(2); delay(300000);
	basic_timer_init(TIM6, 1000, 500);
	blink(1); delay(300000);

	//bool s = true;
	for (;;) {
		//gpio_set(GPIOA_BASE, GPIO_LED, s);
		//delay(s ? 200000 : 50000);
		//s = !s;
		//blink(1); delay(500000);
		IntHandler_Tim6();
	}
	return 0;
}

