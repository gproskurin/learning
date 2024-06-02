#include "lib_stm32.h"
#include "bsp.h"
#include "cmsis_device.h"

#include <stdint.h>


#ifdef TARGET_STM32F103
constexpr auto pin_led_green2 = bsp::pin_led;
#else
constexpr auto pin_led_green2 = bsp::pin_led_green2;
#endif


static
void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


static
void init_periph()
{
#ifdef TARGET_STM32L072
	// flash
	FLASH->ACR |= FLASH_ACR_LATENCY;

	// clock: switch from MSI to HSI
	// turn on HSI & wait
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY)) {}
	// switch to HSI
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (0b01 << RCC_CFGR_SW_Pos);
	// switch off MSI & wait
	RCC->CR &= ~RCC_CR_MSION;
	while (RCC->CR & RCC_CR_MSIRDY) {}

	// LSI
	RCC->CSR |= RCC_CSR_LSION;
	while (!(RCC->CSR & RCC_CSR_LSIRDY)) {}

	// GPIOs
	RCC->IOPENR = RCC_IOPENR_IOPAEN_Msk /*| RCC_IOPENR_IOPBEN_Msk | RCC_IOPENR_IOPCEN_Msk*/;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_IOPARST /*| RCC_IOPRSTR_IOPBRST | RCC_IOPRSTR_IOPCRST*/);

	// USART2 & & LPTIM1
	RCC->APB1ENR |= /*RCC_APB1ENR_USART2EN_Msk |*/ RCC_APB1ENR_LPTIM1EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, /*RCC_APB1RSTR_USART2RST_Msk |*/ RCC_APB1RSTR_LPTIM1RST_Msk);
	NVIC_SetPriority(LPTIM1_IRQn, 20);
	NVIC_EnableIRQ(LPTIM1_IRQn);
#endif
}


#ifdef TARGET_STM32L072
extern "C" __attribute__ ((interrupt)) void IntHandler_Lptim1()
{
	auto const isr = LPTIM1->ISR;
	if (isr & LPTIM_ISR_CMPM) {
		LPTIM1->ICR = LPTIM_ICR_CMPMCF;
		pin_led_green2.set_state(1);
		return;
	}
	if (isr & LPTIM_ISR_ARRM) {
		LPTIM1->ICR = LPTIM_ICR_ARRMCF;
		pin_led_green2.set_state(0);
		return;
	}
}
#endif


template <typename Pin>
void init_pin_led(const Pin& pin)
{
	pin.set_state(0);
	pin.set_mode_output_lowspeed_pushpull();
}


void init_lptim()
{
#ifdef TARGET_STM32L072
	constexpr uint32_t lptim_cr_en = LPTIM_CR_ENABLE;
	constexpr uint32_t lptim_cr_en_start = lptim_cr_en | LPTIM_CR_CNTSTRT;
	RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_LPTIM1SEL_Msk) | (0b01/*LSI*/ << RCC_CCIPR_LPTIM1SEL_Pos);
	LPTIM1->CR = 0;
	LPTIM1->CR = lptim_cr_en;
	LPTIM1->CFGR = (0b000 << LPTIM_CFGR_PRESC_Pos);

	{
		constexpr uint16_t arr = 20000;
		constexpr uint16_t cmp = 19000;
		constexpr uint32_t clear = LPTIM_ICR_ARROKCF | LPTIM_ICR_CMPOKCF;
		constexpr uint32_t wait = LPTIM_ISR_ARROK | LPTIM_ISR_CMPOK;
		LPTIM1->ICR = clear;
		LPTIM1->ARR = arr - 1;
		LPTIM1->CMP = cmp - 1;
		while ((LPTIM1->ISR & wait) != wait) {}
	}

	LPTIM1->IER = 0;
	LPTIM1->ICR = 0xFFFFFFFF;
	LPTIM1->IER = LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE;
	LPTIM1->CR = lptim_cr_en_start;
#endif
}


__attribute__ ((noreturn)) void main()
{
	init_periph();
	init_pin_led(pin_led_green2);
	init_lptim();

	for(;;) {
		__WFI();
	}
}

