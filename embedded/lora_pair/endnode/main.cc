#include "lib_stm32.h"
#include "lcd.h"
#include "sx1276.h"
#include "bsp.h"
#include "freertos_utils.h"
#include "logging.h"
#include "logger_fwd.h"

#include "lora.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>

#define PRIO_WWDG 1
#define PRIO_BLINK 2
#define PRIO_LORA 3
#define PRIO_LOGGER 4


freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green2("blink_green2", bsp::pin_led_green2, PRIO_BLINK);


extern "C"
const sx1276::hwconf_t hwc_emb = {
	.spi = SPI1,
	.spi_af = 0,
	.pin_spi_nss = bsp::sx1276::pin_spi_nss,
	.pin_spi_sck = bsp::sx1276::pin_spi_sck,
	.pin_spi_miso = bsp::sx1276::pin_spi_miso,
	.pin_spi_mosi = bsp::sx1276::pin_spi_mosi,
	.pin_dio0 = bsp::sx1276::pin_dio0,
	//.pin_radio_reset{GPIOC_BASE, 0},
	//.pin_sx1276_reset{GPIOA_BASE, 11},
	//.pin_radio_tcxo_vcc{GPIOA_BASE, 12},
};


logging::logger_t<log_dev_t> logger("logger", PRIO_LOGGER);


void log_sync(const char* s)
{
	stm32_lib::usart::send(USART_STLINK, s);
}


void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


void bus_init()
{
#if 1
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

#endif

	// GPIOs
	RCC->IOPENR = RCC_IOPENR_IOPAEN_Msk | RCC_IOPENR_IOPBEN_Msk | RCC_IOPENR_IOPCEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_IOPARST | RCC_IOPRSTR_IOPBRST | RCC_IOPRSTR_IOPCRST);

	// USART2 & !WWDG & LPTIM1
	RCC->APB1ENR = (RCC->APB1ENR & ~RCC_APB1ENR_WWDGEN) | RCC_APB1ENR_USART2EN_Msk | RCC_APB1ENR_LPTIM1EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_USART2RST_Msk | RCC_APB1RSTR_LPTIM1RST_Msk);

	// SPI1 & SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk | RCC_APB2ENR_SYSCFGEN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST_Msk | RCC_APB2RSTR_SYSCFGRST_Msk);

	// DMA
	RCC->AHBENR |= RCC_AHBENR_DMAEN;
	toggle_bits_10(&RCC->AHBRSTR, RCC_AHBRSTR_DMARST_Msk);

	DMA1_CSELR->CSELR =
		(0b0001 << DMA_CSELR_C2S_Pos) // SPI1_RX
		| (0b0001 << DMA_CSELR_C3S_Pos) // SPI1_TX
		| (0b0100 << DMA_CSELR_C4S_Pos); // USART2_TX
	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	// WWDG
	NVIC_SetPriority(WWDG_IRQn, 0);
	NVIC_EnableIRQ(WWDG_IRQn);

	// LPTIM
	NVIC_SetPriority(LPTIM1_IRQn, 20);
	NVIC_EnableIRQ(LPTIM1_IRQn);

#if 0
	// PB2 interrupt TODO: do not hardcode PB2, use bit masks stuff
	{
		auto const cr = &SYSCFG->EXTICR[pin_userbutton.reg / 4];
		*cr = (*cr & ~(SYSCFG_EXTICR1_EXTI2_Msk)) | SYSCFG_EXTICR1_EXTI2_PB;
	}

	// EXTI
	pin_userbutton.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::no_pupd);
	EXTI->IMR |= (1 << pin_userbutton.reg);
	EXTI->RTSR &= ~(1 << pin_userbutton.reg); // disable rising edge interrupt (button release)
	EXTI->FTSR |= (1 << pin_userbutton.reg); // enable falling edge interrupt (button press)
	NVIC_SetPriority(EXTI2_3_IRQn, 13);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
#endif
}


void perif_init_irq_dio0()
{
	// PB4 sx1276/dio0
	auto const cr = &SYSCFG->EXTICR[bsp::sx1276::pin_dio0.reg / 4];
	*cr = (*cr & ~(SYSCFG_EXTICR2_EXTI4_Msk)) | SYSCFG_EXTICR2_EXTI4_PB;

	// EXTI
	EXTI->IMR |= (1 << bsp::sx1276::pin_dio0.reg);
	EXTI->RTSR |= (1 << bsp::sx1276::pin_dio0.reg); // irq on raising edge
	EXTI->FTSR &= ~(1 << bsp::sx1276::pin_dio0.reg);
	NVIC_SetPriority(EXTI4_15_IRQn, 0);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
}


extern "C" __attribute__ ((interrupt)) void IntHandler_DMA1_Channel4_5_6_7()
{
	auto const isr = DMA1->ISR;

	{
		uint32_t ev_log = 0;
		if (isr & DMA_ISR_TCIF4) {
			DMA1->IFCR = DMA_IFCR_CTCIF4;
			ev_log |= stm32_lib::dma::dma_result_t::tc;
		}
		if (isr & DMA_ISR_TEIF4) {
			DMA1->IFCR = DMA_IFCR_CTEIF4;
			ev_log |= stm32_lib::dma::dma_result_t::te;
		}
		if (ev_log) {
			logger.notify_from_isr(ev_log);
		}
	}
}


StaticTask_t xTaskBufferIdle;
freertos_utils::task_stack_t<64> idle_task_stack;
extern "C"
void vApplicationGetIdleTaskMemory(StaticTask_t **tcbIdle, StackType_t **stackIdle, uint32_t *stackSizeIdle)
{
	*tcbIdle = &xTaskBufferIdle;
	*stackIdle = idle_task_stack.data();
	*stackSizeIdle = idle_task_stack.size();
}

extern "C"
void vApplicationIdleHook(void)
{
	__WFI();
}


freertos_utils::task_data_t<128> task_data_wwdg;

void task_function_wwdg(void*)
{
	logger.log_async("WWDG task started\r\n");

	RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_WWDGRST_Msk);

	constexpr uint32_t cr1 = (0b1111111 << WWDG_CR_T_Pos);
	constexpr uint32_t cr2 = cr1 | WWDG_CR_WDGA;
	constexpr uint32_t cr_upd = (cr2 & ~WWDG_CR_T_Msk) | (0b1111111 << WWDG_CR_T_Pos);

	WWDG->CFR = WWDG_CFR_EWI | (0b11 << WWDG_CFR_WDGTB_Pos) | (0b1111111 << WWDG_CFR_W_Pos);
	WWDG->CR = cr1;
	WWDG->CR = cr2;

	logger.log_async("WWDG: wwdg enabled\r\n");
	for (;;) {
		constexpr auto ticks_delay = stm32_lib::wwdg::counts_to_ticks<
			configCPU_CLOCK_HZ,
			configTICK_RATE_HZ,
			8,
			0x7F - 0x40
		>();
		static_assert(ticks_delay == 12);
		vTaskDelay(ticks_delay - 2);
		WWDG->CR = cr_upd;
	}
}

void create_task_wwdg()
{
	task_data_wwdg.task_handle = xTaskCreateStatic(
		&task_function_wwdg,
		"wwdg",
		task_data_wwdg.stack.size(),
		nullptr,
		PRIO_WWDG,
		task_data_wwdg.stack.data(),
		&task_data_wwdg.task_buffer
	);
}


extern "C" __attribute__ ((interrupt)) void IntHandler_WWDG()
{
	if (WWDG->SR & WWDG_SR_EWIF) {
		WWDG->SR = 0;
		logger.log_async_from_isr("IRQ: wwdg\r\n");
	}
}


extern "C" __attribute__ ((interrupt)) void IntHandler_Lptim1()
{
	logger.log_async_from_isr("IRQ: lptim1 ISR_START\r\n");
	if (LPTIM1->ISR & LPTIM_ISR_ARRM) {
		LPTIM1->ICR = LPTIM_ICR_ARRMCF;
		logger.log_async_from_isr("IRQ: lptim1 ARR\r\n");
	}
}


__attribute__ ((noreturn)) void main()
{
	bus_init();
	stm32_lib::usart::init_logger_uart<configCPU_CLOCK_HZ>(
		USART_STLINK,
		bsp::usart_stlink_pin_tx,
		USART_STLINK_PIN_TX_AF
	);

	log_sync("\r\nLogger initialized (sync)\r\n");

	g_pin_green.init_pin();
	g_pin_blue.init_pin();
	g_pin_red.init_pin();
	g_pin_green2.init_pin();
	g_pin_green2.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/25);

	log_sync("Creating LORA task...\r\n");
	lora::create_task("lora", PRIO_LORA, &hwc_emb);
	log_sync("Created LORA task\r\n");

	create_task_wwdg();

	// LPTIM
	constexpr uint32_t lptim_cr_en = LPTIM_CR_ENABLE;
	constexpr uint32_t lptim_cr_en_start = lptim_cr_en | LPTIM_CR_CNTSTRT;
	RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_LPTIM1SEL_Msk) | (0b01/*LSI*/ << RCC_CCIPR_LPTIM1SEL_Pos);
	LPTIM1->CR = 0;
	LPTIM1->CR = lptim_cr_en;
	LPTIM1->CFGR =
		(0b100/*16*/ << LPTIM_CFGR_PRESC_Pos)
		| (0b00 << LPTIM_CFGR_TRIGEN_Pos)
		| (0b000 << LPTIM_CFGR_TRIGSEL_Pos);

	LPTIM1->ICR = LPTIM_ICR_ARROKCF;
	LPTIM1->ARR = 37000-1;
	while (!(LPTIM1->ISR & LPTIM_ISR_ARROK)) {}
	LPTIM1->ICR = LPTIM_ICR_ARROKCF;

	LPTIM1->IER = 0;
	LPTIM1->ICR = 0xFFFFFFFF;
	LPTIM1->IER = LPTIM_IER_ARRMIE;
	LPTIM1->CR = lptim_cr_en_start;

	log_sync("Starting FreeRTOS scheduler\r\n");
	logger.init();
	vTaskStartScheduler();

	log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

