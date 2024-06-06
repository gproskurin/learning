#include "lib_stm32.h"
#include "bsp.h"
#include "cmsis_device.h"

#include <stdint.h>


#if defined TARGET_STM32L072
constexpr auto pin_led = bsp::pin_led_green2;
#elif defined TARGET_STM32F103
constexpr auto pin_led = bsp::pin_led;
#elif defined TARGET_STM32WL55_CPU1
constexpr auto pin_led = bsp::pin_led_green;
#elif defined TARGET_STM32WL55_CPU2
constexpr auto pin_led = bsp::pin_led_red;
#endif


static
void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


template <bool High>
void freq_flash()
{
	constexpr uint32_t ws = (High ? 0b010/*2WS*/ : 0b000/*0WS*/);
	FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | (ws << FLASH_ACR_LATENCY_Pos);
	// wait for latency to be applied
	while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != (ws << FLASH_ACR_LATENCY_Pos)) {}
}

template <bool High>
void freq_msi()
{
	constexpr uint32_t msi_range = (High ? 0b1011/*48MHz*/ : 0b0100/*1MHz*/);
	uint32_t cr = RCC->CR;
	while (!(cr & RCC_CR_MSIRDY)) {
		cr = RCC->CR;
	}
	RCC->CR = (cr & ~RCC_CR_MSIRANGE_Msk) | (msi_range << RCC_CR_MSIRANGE_Pos);
}


void freq_high()
{
	freq_flash<true>();
	freq_msi<true>();
}

void freq_low()
{
	freq_msi<false>();
	freq_flash<false>();
}


static
void init_periph()
{
#if defined TARGET_STM32L072
#if 0
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
#endif
	while (!(RCC->CR & RCC_CR_MSIRDY)) {}
	RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE_Msk) | (0b000 << RCC_ICSCR_MSIRANGE_Pos);

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
#elif defined TARGET_STM32WL55_CPU1
	// FLASH: enable caches & prefetch
	{
		constexpr uint32_t acr_en_cp = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN;
		constexpr uint32_t acr_rst = FLASH_ACR_DCRST | FLASH_ACR_ICRST;
		auto acr = FLASH->ACR;

		// disable caches & prefetch
		acr &= ~acr_en_cp;
		FLASH->ACR = acr;

		// reset caches
		acr |= acr_rst;
		FLASH->ACR = acr;
		acr &= ~acr_rst;
		FLASH->ACR = acr;

		// enable caches & prefetch
		acr |= acr_en_cp;
		FLASH->ACR = acr;
	}

	RCC->CR |= RCC_CR_MSIRGSEL;
	while (!(RCC->CR & RCC_CR_MSIRDY)) {}
	freq_high();

	// LSI
	RCC->CSR |= RCC_CSR_LSION;
	while (!(RCC->CSR & RCC_CSR_LSIRDY)) {}

#if 0
	// PCLK3
	RCC->EXTCFGR = (RCC->EXTCFGR & ~RCC_EXTCFGR_SHDHPRE_Msk)
		| ((0b0000) << RCC_EXTCFGR_SHDHPRE_Pos)
		;
	while (!(RCC->EXTCFGR & RCC_EXTCFGR_SHDHPREF)) {} // wait for prescaler to be applied
#endif

	// switch on HSE
	//RCC->CR |= RCC_CR_HSEON;
	//while (!(RCC->CR & RCC_CR_HSERDY)) {} // TODO

	// AHB1
	RCC->AHB1ENR = 0;
	RCC->AHB1SMENR = 0;

	// AHB2: GPIOs
	RCC->AHB2ENR = RCC_AHB2ENR_GPIOAEN_Msk | RCC_AHB2ENR_GPIOBEN_Msk | RCC_AHB2ENR_GPIOCEN_Msk;
	toggle_bits_10(
		&RCC->AHB2RSTR,
		RCC_AHB2RSTR_GPIOARST_Msk | RCC_AHB2RSTR_GPIOBRST_Msk | RCC_AHB2RSTR_GPIOCRST_Msk
	);
	RCC->AHB2SMENR = RCC_AHB2SMENR_GPIOASMEN_Msk | RCC_AHB2SMENR_GPIOBSMEN_Msk | RCC_AHB2SMENR_GPIOCSMEN_Msk;

	// AHB3
	RCC->AHB3ENR = RCC_AHB3ENR_FLASHEN;
	RCC->AHB3SMENR = RCC_AHB3SMENR_FLASHSMEN | RCC_AHB3SMENR_SRAM1SMEN;

	// APB1: LPTIM1
	RCC->APB1ENR1 = RCC_APB1ENR1_LPTIM1EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR1, RCC_APB1RSTR1_LPTIM1RST_Msk);
	RCC->APB1ENR2 = 0;
	RCC->APB1SMENR1 = RCC_APB1SMENR1_LPTIM1SMEN_Msk;
	RCC->APB1SMENR2 = 0;

	// APB2
	RCC->APB2ENR = 0;
	RCC->APB2SMENR = 0;

	// APB3
	RCC->APB3ENR = 0;
	RCC->APB3SMENR = 0;
#else
#error
#endif
}


#if defined(TARGET_STM32L072) || defined (TARGET_STM32WL55)
volatile int count_led_blinks = 0;
extern "C" __attribute__ ((interrupt)) void IntHandler_Lptim1()
{
	auto const isr = LPTIM1->ISR;
	if (isr & LPTIM_ISR_CMPM) {
		LPTIM1->ICR = LPTIM_ICR_CMPMCF;
		pin_led.set_state(1);
		return;
	}
	if (isr & LPTIM_ISR_ARRM) {
		LPTIM1->ICR = LPTIM_ICR_ARRMCF;
		pin_led.set_state(0);
		++count_led_blinks;
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
#if defined(TARGET_STM32L072) || defined(TARGET_STM32WL55_CPU1)
	constexpr uint32_t lptim_cr_en = LPTIM_CR_ENABLE;
	constexpr uint32_t lptim_cr_en_start = lptim_cr_en | LPTIM_CR_CNTSTRT;
	LPTIM1->CR = 0;
	RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_LPTIM1SEL_Msk) | (0b01/*LSI*/ << RCC_CCIPR_LPTIM1SEL_Pos);
	LPTIM1->IER = LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE;
	LPTIM1->CFGR = (0b010 << LPTIM_CFGR_PRESC_Pos);

	LPTIM1->CR = lptim_cr_en;

	{
		constexpr uint16_t arr = 37000;
		constexpr uint16_t cmp = 22000;
		constexpr uint32_t clear = LPTIM_ICR_ARROKCF | LPTIM_ICR_CMPOKCF;
		constexpr uint32_t wait = LPTIM_ISR_ARROK | LPTIM_ISR_CMPOK;
		LPTIM1->ICR = clear;
		LPTIM1->ARR = arr - 1;
		LPTIM1->CMP = cmp - 1;
		while ((LPTIM1->ISR & wait) != wait) {}
	}

	// wakeup from lptim1
#if defined TARGET_STM32L072
	EXTI->IMR = (1 << 29); // |= ?
#elif defined TARGET_STM32WL55_CPU1
	EXTI->IMR1 = (1 << 29);
#endif

	NVIC_SetPriority(LPTIM1_IRQn, 46);
	NVIC_EnableIRQ(LPTIM1_IRQn);

	LPTIM1->CR = lptim_cr_en_start;
#endif
}


struct mode_run {
	static void prepare_once() {}
	static void unprepare_once() {}
	static void wfi() {}
};

struct mode_sleep {
	static void prepare_once() {}
	static void unprepare_once() {}
	static void wfi() { __WFI(); }
};

struct mode_lprun {
#if defined TARGET_STM32WL55_CPU1
	static void prepare_once()
	{
		//PWR->CR1 |= PWR_CR1_FPDS;
		//PWR->C2CR1 | PWR_C2CR1_FPDS;
		freq_low();
		while (!(PWR->SR2 & PWR_SR2_REGLPS)) {}
		PWR->CR1 |= PWR_CR1_LPR;
	}
	static void unprepare_once()
	{
		//PWR->CR1 &= ~PWR_CR1_FPDS;
		//PWR->C2CR1 &= ~PWR_C2CR1_FPDS;
		PWR->CR1 &= ~PWR_CR1_LPR;
		while (PWR->SR2 & PWR_SR2_REGLPF) {}
		freq_high();
	}
#endif
	static void wfi() {}
};

struct mode_lpsleep : public mode_lprun {
	static void wfi() { __WFI(); }
};

struct mode_stop0 {
#if defined TARGET_STM32WL55_CPU1
	static void prepare_once()
	{
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS_Msk) | (0b000 << PWR_CR1_LPMS_Pos);
	}
	static void unprepare_once()
	{
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	}
#endif
	static void wfi() { __WFI(); }
};

struct mode_stop1 {
#if defined TARGET_STM32WL55_CPU1
	static void prepare_once()
	{
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS_Msk) | (0b001 << PWR_CR1_LPMS_Pos);
	}
	static void unprepare_once()
	{
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	}
#endif
	static void wfi() { __WFI(); }
};

struct mode_stop2 {
#if defined TARGET_STM32WL55_CPU1
	static void prepare_once()
	{
		PWR->CR1 = (PWR->CR1 & ~(PWR_CR1_LPR | PWR_CR1_LPMS_Msk)) | (0b010 << PWR_CR1_LPMS_Pos);
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	}
	static void unprepare_once()
	{
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS_Msk) | (0b000 << PWR_CR1_LPMS_Pos);
	}
#endif
	static void wfi() { __WFI(); }
};


__attribute__ ((noreturn)) void main()
{
	init_periph();
	init_pin_led(pin_led);
	init_lptim();

	//using mode_t = mode_run;
	//using mode_t = mode_sleep;
	//using mode_t = mode_lprun;
	using mode_t = mode_lpsleep;
	//using mode_t = mode_stop0;
	//using mode_t = mode_stop1;
	//using mode_t = mode_stop2;

	bool is_lp = false;
	mode_t::unprepare_once();
	for(;;) {
		const bool want_lp = (count_led_blinks % 4) >= 2;
		if (want_lp && !is_lp) {
			mode_t::prepare_once();
			is_lp = true;
		} else if (!want_lp && is_lp) {
			mode_t::unprepare_once();
			is_lp = false;
		}
		mode_t::wfi();
	}
}

