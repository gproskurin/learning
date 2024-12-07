#include "cmsis_device.h"

#include "bsp.h"
#include "freertos_utils.h"
#include "lib_stm32.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"


#include <stdint.h>


#define PRIO_BLINK 1
#define PRIO_USB 3
#define PRIO_LOGGER 2


using log_dev_t = stm32_lib::dma::dev_usart_dma_t<USART2_BASE, DMA1_Channel4_BASE>;
logging::logger_t<log_dev_t> logger("logger", PRIO_LOGGER);

void log_sync(const char* s)
{
	stm32_lib::usart::send(USART_STLINK, s);
}

freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green2("blink_green2", bsp::pin_led_green2, PRIO_BLINK);


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

	// USART2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_USART2RST_Msk);

	// DMA
	RCC->AHBENR |= RCC_AHBENR_DMAEN_Msk;
	toggle_bits_10(&RCC->AHBRSTR, RCC_AHBRSTR_DMARST_Msk);

	DMA1_CSELR->CSELR =
		(0b0100 << DMA_CSELR_C4S_Pos); // USART2_TX
	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
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


freertos_utils::task_data_t<128> task_data_usb;

void task_function_usb(void*)
{
	logger.log_async("USB task started\r\n");

	for (;;) {
		vTaskDelay(configTICK_RATE_HZ * 5);
		logger.log_async("USB task keep-alive\r\n");
	}
}

void create_task_usb()
{
	task_data_usb.task_handle = xTaskCreateStatic(
		&task_function_usb,
		"usb",
		task_data_usb.stack.size(),
		nullptr,
		PRIO_USB,
		task_data_usb.stack.data(),
		&task_data_usb.task_buffer
	);
}



__attribute__ ((noreturn)) void main()
{
	init_periph();

	g_pin_green.init_pin();
	g_pin_blue.init_pin();
	g_pin_red.init_pin();
	g_pin_green2.init_pin();

	g_pin_green.pulse_continuous(configTICK_RATE_HZ/30, configTICK_RATE_HZ/13);
	g_pin_blue.pulse_continuous(configTICK_RATE_HZ/47, configTICK_RATE_HZ/7);
	g_pin_red.pulse_continuous(configTICK_RATE_HZ/5, configTICK_RATE_HZ*4/5);
	g_pin_green2.pulse_continuous(configTICK_RATE_HZ/43, configTICK_RATE_HZ/25);

	create_task_usb();

	logger.init();
	vTaskStartScheduler();

	for (;;) {}
}

