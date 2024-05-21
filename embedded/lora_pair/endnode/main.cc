#include "lib_stm32.h"
#include "lcd.h"
#include "sx1276.h"
#include "bsp.h"
#include "freertos_utils.h"
#include "logging.h"

#include "lora.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>

#define PRIO_BLINK 1
#define PRIO_LORA 6
#define PRIO_LOGGER 8 // FIXME


freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green2("blink_green2", bsp::pin_led_green2, PRIO_BLINK);


lora::task_data_t task_data_lora;


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

usart_logger_t logger(USART_STLINK, "logger", PRIO_LOGGER);

#define USART_CON_BAUDRATE 115200

void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_TE;

	usart->CR3 = USART_CR3_DMAT; // DMA
	stm32_lib::gpio::set_mode_af_lowspeed_pu(bsp::usart_stlink_pin_tx, USART_STLINK_PIN_TX_AF);
	usart->BRR = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;

	usart->CR1 = cr1;
	usart->CR1 = cr1 | USART_CR1_UE;
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
#endif

	// GPIOs
	RCC->IOPENR = RCC_IOPENR_IOPAEN_Msk | RCC_IOPENR_IOPBEN_Msk | RCC_IOPENR_IOPCEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_IOPARST | RCC_IOPRSTR_IOPBRST | RCC_IOPRSTR_IOPCRST);

	// USART2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_USART2RST_Msk);

	// SPI1 & SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk | RCC_APB2ENR_SYSCFGEN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST_Msk | RCC_APB2RSTR_SYSCFGRST_Msk);

	// DMA
	RCC->AHBENR |= RCC_AHBENR_DMAEN;
	toggle_bits_10(&RCC->AHBRSTR, RCC_AHBRSTR_DMARST_Msk);

	// configure DMA: USART2/Channel4
	DMA1_CSELR->CSELR = (0b0100 << DMA_CSELR_C4S_Pos);
	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

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


extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI4_15()
{
	if (EXTI->PR & (1 << bsp::sx1276::pin_dio0.reg)) {
		EXTI->PR = (1 << bsp::sx1276::pin_dio0.reg);

		BaseType_t yield = pdFALSE;
		xTaskNotifyFromISR(task_data_lora.task_handle, 0, eSetBits, &yield);
		portYIELD_FROM_ISR(yield);
	}
}


extern "C" __attribute__ ((interrupt)) void IntHandler_DMA1_Channel4_5_6_7()
{
	using ev = usart_logger_t::events_t;
	uint32_t events = 0;

	auto const isr = DMA1->ISR;
	if (isr & DMA_ISR_TEIF4) {
		events |= ev::dma_te;
	}
	if (isr & DMA_ISR_TCIF4) {
		events |= ev::dma_tc;
	}

	if (events) {
		DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CTEIF4 | DMA_IFCR_CTCIF4;
		xTaskNotifyFromISR(logger.task_handle_, events, eSetBits, nullptr);
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


__attribute__ ((noreturn)) void main()
{
	bus_init();
	logger.init_dma();

	usart_init(USART_STLINK);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	g_pin_green.init_pin();
	g_pin_blue.init_pin();
	g_pin_red.init_pin();
	g_pin_green2.init_pin();
	g_pin_green2.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/25);

	logger.log_sync("Creating LORA task...\r\n");
	lora::create_task("lora", PRIO_LORA, task_data_lora, &hwc_emb);
	logger.log_sync("Created LORA task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

