//

#if defined TARGET_STM32L152
#include <cmsis_device_l1/Include/stm32l152xe.h>
#elif defined TARGET_STM32F103
#include <cmsis_device_f1/Include/stm32f103xb.h>
#else
#error
#endif

#include <stdbool.h>
#include <stdint.h>


#if defined TARGET_STM32F103
	// PB12
	#define LED_GPIO GPIOB
	#define LED_PIN 12
	#define LED_TIM TIM1
#elif defined TARGET_STM32L152
	// PA5
	#define LED_GPIO GPIOA
	#define LED_PIN 5
	#define LED_TIM TIM9
#endif


void basic_timer_init(TIM_TypeDef* const tim, uint16_t prescaler, uint16_t arr)
{
	//tim->EGR |= TIM_EGR_UG; // remove
	tim->PSC = prescaler;
	tim->ARR = arr;
	tim->DIER |= TIM_DIER_UIE; // enable hardware interrupt
	tim->CR1 = (tim->CR1 & ~(TIM_CR1_UDIS | TIM_CR1_OPM)) | TIM_CR1_CEN;
}


#if defined TARGET_STM32F103
void gpio_set_mode(GPIO_TypeDef* const gpio, int reg)
{
	const uint32_t mode = 0b10; // output mode, max speed 2 MHz
	const uint32_t cnf = 0b00; // output push-pull
	const uint32_t reg_lo = ((reg < 8) ? reg : reg - 8);
	uint32_t bits = (cnf << 2) | mode;
	const uint32_t mask = 0b1111 << (reg_lo * 4);
	const uint32_t mask_value = bits << (reg_lo * 4);
	auto const cr = ((reg < 8) ? &gpio->CRL : &gpio->CRH);
	*cr = (*cr & ~mask) | mask_value;
}

#elif defined TARGET_STM32L152

void gpio_set_mode(GPIO_TypeDef* const gpio, int reg)
{
	const uint32_t mask = 0b11 << (reg * 2);
	const uint32_t mask_value = 0b01 << (reg * 2); // general-purpose output
	gpio->MODER = (gpio->MODER & ~mask) | mask_value;
	gpio->OTYPER &= ~(1U << reg); // push-pull
	gpio->OSPEEDR &= ~(0b11 << (reg*2)) ; // 0b00 = low speed
	gpio->PUPDR &= (0b11 << (reg*2)); // 0b00 = no pull up/down
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

void toggle_led()
{
	//const bool high = led_high;
	LED_GPIO->ODR ^= (1U << LED_PIN);
	//gpio_set(GPIOA, GPIO_LED, high);
	//led_high = !high;
}

extern "C" __attribute__ ((interrupt)) void IntHandler_Timer()
{
	if (LED_TIM->SR & TIM_SR_UIF) {
		LED_TIM->SR = ~TIM_SR_UIF;
		toggle_led();
	}
}


void gpio_bus_enable()
{

#if defined TARGET_STM32F103
	// enable timer and port B
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN_Msk | RCC_APB2ENR_IOPBEN_Msk;

	// reset TIM1
	RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST_Msk;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST_Msk;

#elif defined TARGET_STM32L152
	// enable port A
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN_Msk;

if (0) {
	// TIM6
	// enable TIM
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN_Msk;

	// reset TIM
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST_Msk;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM6RST_Msk;
}

if (1) {
	// TIM9
	// enable TIM
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN_Msk;

	// reset TIM
	RCC->APB2RSTR |= RCC_APB2RSTR_TIM9RST_Msk;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM9RST_Msk;
}
#endif
}


void nvic_init_tim()
{
	// enable interrupt
#if defined TARGET_STM32F103
	NVIC_SetPriority(TIM1_UP_IRQn, 3);
	NVIC_EnableIRQ(TIM1_UP_IRQn);
#elif defined TARGET_STM32L152
	NVIC_SetPriority(TIM9_IRQn, 3);
	NVIC_EnableIRQ(TIM9_IRQn);
#endif
}


void blink(int n)
{
	while (n-- > 0) {
		gpio_set(LED_GPIO, LED_PIN, 1);
		delay(50000);
		gpio_set(LED_GPIO, LED_PIN, 0);
		delay(50000);
	}
}


__attribute__ ((noreturn)) int main()
{
	gpio_bus_enable();
	gpio_set_mode(LED_GPIO, LED_PIN);
	blink(1); delay(300000);

	nvic_init_tim();
	blink(2); delay(300000);
	basic_timer_init(LED_TIM, 100-1, 2000-1);
	//blink(3); delay(300000);

	for (;;) {
		__WFI();
	}
}

