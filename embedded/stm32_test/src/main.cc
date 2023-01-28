#include "lib_stm32.h"

#include <stdbool.h>
#include <stdint.h>


#if defined TARGET_STM32F103
	// PB12
	#define LED_GPIO GPIOB
	#define LED_PIN 12
	#define LED_TIM TIM2

	// USART1, tx(PA9)
	#define USART_LOG USART1
	#define USART_LOG_GPIO GPIOA
	#define USART_LOG_PIN_TX 9
#elif defined TARGET_STM32L152
	// PA5
	#define LED_GPIO GPIOA
	#define LED_PIN 5
	#define LED_TIM TIM9

	// USART1, tx(PA9)
	#define USART_LOG USART1
	#define USART_LOG_GPIO GPIOA
	#define USART_LOG_PIN_TX 9
	//#define USART_LOG_PIN_RX 10
#endif

#define USART_CON_BAUDRATE 115200
#if defined (TARGET_STM32F103)
#define CLOCK_SPEED 48000000 // FIXME
#elif defined (TARGET_STM32L152)
#define CLOCK_SPEED 2000000 // FIXME
#endif


void usart_init(USART_TypeDef* const usart)
{
#ifdef TARGET_STM32F103
	stm32_lib::gpio::set_mode_af_lowspeed_pu(USART_LOG_GPIO, USART_LOG_PIN_TX, 7/*FIXME*/);
	//usart->BRR = ((div / 16) << USART_BRR_DIV_MANTISSA_Pos) | ((div % 16) << USART_BRR_DIV_FRACTION_Pos);
#elif defined TARGET_STM32L152
	stm32_lib::gpio::set_mode_af_lowspeed_pu(USART_LOG_GPIO, USART_LOG_PIN_TX, 7);
	usart->BRR = 2000000 / USART_CON_BAUDRATE;
	usart->CR1 = USART_CR1_UE | USART_CR1_TE;
	//usart->CR3 = USART_CR3_DMAT;
#endif
}


void usart_tx(USART_TypeDef* const usart, const char* s)
{
	while (*s) {
		while (! (usart->SR & USART_SR_TXE)) {
		}
		usart->DR = *s;
		++s;
	}
}


void log_str(const char* s)
{
	usart_tx(USART_LOG, s);
}


void basic_timer_init(TIM_TypeDef* const tim, uint16_t prescaler, uint16_t arr)
{
	tim->PSC = prescaler;
	tim->ARR = arr;
	tim->CCR1 = arr / 5 * 4;
	tim->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE; // enable hardware interrupt
	tim->CR1 = (tim->CR1 & ~(TIM_CR1_UDIS | TIM_CR1_OPM)) | TIM_CR1_CEN;
}


void delay(int val)
{
	for (volatile int i = 0; i<val; ++i) {
	}
}


extern "C" __attribute__ ((interrupt)) void IntHandler_Timer()
{
	const uint32_t sr = LED_TIM->SR;
	if (sr & TIM_SR_UIF) {
		// new cycle starts, led on
		LED_TIM->SR = ~TIM_SR_UIF;
		stm32_lib::gpio::set_state(LED_GPIO, LED_PIN, 1);
	} else if (sr & TIM_SR_CC1IF) {
		// compare event triggered, part of cycle finished, led off
		LED_TIM->SR = ~TIM_SR_CC1IF;
		stm32_lib::gpio::set_state(LED_GPIO, LED_PIN, 0);
	}
}


void bus_init()
{

#if defined TARGET_STM32F103
	// enable timer and port B
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN_Msk;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN_Msk;

	// reset TIM2
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST_Msk;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST_Msk;

#elif defined TARGET_STM32L152
	// enable port A
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN_Msk;

	// USART
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;
	RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST_Msk;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST_Msk;

	// TIM9
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN_Msk;
	RCC->APB2RSTR |= RCC_APB2RSTR_TIM9RST_Msk;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM9RST_Msk;
#endif
}


void nvic_init_tim()
{
	// enable interrupt
#if defined TARGET_STM32F103
	NVIC_SetPriority(TIM2_IRQn, 3);
	NVIC_EnableIRQ(TIM2_IRQn);
#elif defined TARGET_STM32L152
	//NVIC_SetVector(TIM9_IRQn, (uint32_t) &IntHandler_Timer);
	NVIC_SetPriority(TIM9_IRQn, 3);
	NVIC_EnableIRQ(TIM9_IRQn);
#endif
}


void blink(int n)
{
	while (n-- > 0) {
		stm32_lib::gpio::set_state(LED_GPIO, LED_PIN, 1);
		delay(50000);
		stm32_lib::gpio::set_state(LED_GPIO, LED_PIN, 0);
		delay(10000);
	}
}


__attribute__ ((noreturn)) void main()
{
	bus_init();
	usart_init(USART_LOG);
	log_str("\r\n");

	log_str("Setting mode for LED\r\n");
	stm32_lib::gpio::set_mode_output_lowspeed_pushpull(LED_GPIO, LED_PIN);
	log_str("Switching off LED\r\n");
	stm32_lib::gpio::set_state(LED_GPIO, LED_PIN, 0);

	log_str("Initializing interrupts\r\n");
	nvic_init_tim();

	log_str("Initializing timer\r\n");
	basic_timer_init(LED_TIM, 2000-1, 1000-1);

	log_str("Initialization done\r\n");

	log_str("Starting WFI loop\r\n");
	for (;;) {
		__WFI();
	}
}

