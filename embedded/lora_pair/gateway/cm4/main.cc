#include "../common/utils.h"
#include "lib_stm32.h"
#include "bsp.h"
#include "freertos_utils.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>

#define PRIO_BLINK 1
#define PRIO_IPCC_RECV 2
#define PRIO_LOGGER 3


constexpr uint8_t ipcc_ch_lora = 3;
constexpr uint32_t mb_ptr_addr = 0x2000807c; // keep it up-to-date, taken from CM0 linker map file

using log_dev_t = stm32_lib::dma::dev_usart_dmamux_t<LPUART1_BASE, DMAMUX1_Channel0_BASE>;
logging::logger_t<log_dev_t> logger("logger_cm4", PRIO_LOGGER);

freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
//freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);


void log_sync(const char* s)
{
	stm32_lib::usart::send(LPUART1, s);
}


void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


void bus_init()
{
#if defined TARGET_STM32WL55_CPU1
	// flash
	FLASH->ACR &= (FLASH_ACR_DCEN | FLASH_ACR_ICEN); // disable caches
	FLASH->ACR |= (FLASH_ACR_DCRST | FLASH_ACR_ICRST); // reset caches
	FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk)
		| (0b010 << FLASH_ACR_LATENCY_Pos) // 2WS
		| (FLASH_ACR_DCEN | FLASH_ACR_ICEN) // enable caches
		| FLASH_ACR_PRFTEN
		;

#if 1
	while (!(RCC->CR & RCC_CR_MSIRDY)) {}
	RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE_Msk)
		| RCC_CR_MSIRGSEL
		| (0b1011 << RCC_CR_MSIRANGE_Pos)
		;
#endif
	// PCLK3(subghzspi)
	RCC->EXTCFGR = (RCC->EXTCFGR & ~RCC_EXTCFGR_SHDHPRE_Msk)
		| ((0b0000) << RCC_EXTCFGR_SHDHPRE_Pos)
		;
	while (!(RCC->EXTCFGR & RCC_EXTCFGR_SHDHPREF)) {} // wait for prescaler to be applied

	// switch on HSE
	//RCC->CR |= RCC_CR_HSEON;
	//while (!(RCC->CR & RCC_CR_HSERDY)) {} // TODO

	// AHB1: DMA1 & DMAMUX1
	RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC->AHB1SMENR |= RCC_AHB1SMENR_DMAMUX1SMEN | RCC_AHB1SMENR_DMA1SMEN | RCC_AHB1SMENR_DMA2SMEN;
	RCC->C2AHB1ENR |= RCC_C2AHB1ENR_DMAMUX1EN | RCC_C2AHB1ENR_DMA1EN | RCC_C2AHB1ENR_DMA2EN;
	RCC->C2AHB1SMENR |= RCC_C2AHB1SMENR_DMAMUX1SMEN | RCC_C2AHB1SMENR_DMA1SMEN | RCC_C2AHB1SMENR_DMA2SMEN;
	toggle_bits_10(&RCC->AHB1RSTR, RCC_AHB1RSTR_DMA1RST_Msk | RCC_AHB1RSTR_DMA2RST_Msk| RCC_AHB1RSTR_DMAMUX1RST_Msk);

	// AHB2: GPIOs
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN_Msk | RCC_AHB2ENR_GPIOBEN_Msk | RCC_AHB2ENR_GPIOCEN_Msk;
	RCC->AHB2SMENR |= RCC_AHB2SMENR_GPIOASMEN_Msk | RCC_AHB2SMENR_GPIOBSMEN_Msk | RCC_AHB2SMENR_GPIOCSMEN_Msk;
	RCC->C2AHB2ENR |= RCC_C2AHB2ENR_GPIOAEN_Msk | RCC_C2AHB2ENR_GPIOBEN_Msk | RCC_C2AHB2ENR_GPIOCEN_Msk;
	RCC->C2AHB2SMENR |= RCC_C2AHB2SMENR_GPIOASMEN_Msk | RCC_C2AHB2SMENR_GPIOBSMEN_Msk | RCC_C2AHB2SMENR_GPIOCSMEN_Msk;
	toggle_bits_10(
		&RCC->AHB2RSTR,
		RCC_AHB2RSTR_GPIOARST_Msk | RCC_AHB2RSTR_GPIOBRST_Msk | RCC_AHB2RSTR_GPIOCRST_Msk
	);

	// AHB3
	RCC->AHB3ENR |= RCC_AHB3ENR_FLASHEN | RCC_AHB3ENR_IPCCEN;
	RCC->AHB3SMENR |= RCC_AHB3SMENR_FLASHSMEN | RCC_AHB3SMENR_SRAM2SMEN | RCC_AHB3SMENR_SRAM1SMEN;
	RCC->C2AHB3ENR |= RCC_C2AHB3ENR_FLASHEN | RCC_C2AHB3ENR_IPCCEN;
	RCC->C2AHB3SMENR |= RCC_C2AHB3SMENR_FLASHSMEN | RCC_C2AHB3SMENR_SRAM2SMEN | RCC_C2AHB3SMENR_SRAM1SMEN;
	toggle_bits_10(&RCC->AHB3RSTR, RCC_AHB3RSTR_IPCCRST);

	// APB1: LPUART1
	RCC->APB1ENR1 |= 0;
	RCC->APB1SMENR1 |= 0;
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
	RCC->APB1SMENR2 |= RCC_APB1SMENR2_LPUART1SMEN;
	RCC->C2APB1ENR1 |= 0;
	RCC->C2APB1SMENR1 |= 0;
	RCC->C2APB1ENR2 |= RCC_C2APB1ENR2_LPUART1EN;
	RCC->C2APB1SMENR2 |= RCC_C2APB1SMENR2_LPUART1SMEN;
	toggle_bits_10(&RCC->APB1RSTR2, RCC_APB1RSTR2_LPUART1RST_Msk);
	RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_LPUART1SEL_Msk))
		| (0b01 << RCC_CCIPR_LPUART1SEL_Pos)
		;

	// APB2
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->APB2SMENR |= RCC_APB2SMENR_USART1SMEN;
	RCC->C2APB2ENR |= RCC_C2APB2ENR_USART1EN;
	RCC->C2APB2SMENR |= RCC_C2APB2SMENR_USART1SMEN;

	// APB3: SUBGHZSPI
	RCC->APB3ENR |= RCC_APB3ENR_SUBGHZSPIEN; // TODO not needed?
	RCC->APB3SMENR |= RCC_APB3SMENR_SUBGHZSPISMEN;
	RCC->C2APB3ENR |= RCC_C2APB3ENR_SUBGHZSPIEN;
	RCC->C2APB3SMENR |= RCC_C2APB3SMENR_SUBGHZSPISMEN;
	toggle_bits_10(&RCC->APB3RSTR, RCC_APB3RSTR_SUBGHZSPIRST);

	NVIC_SetPriority(DMA1_Channel1_IRQn, 11);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#endif
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


void set_pin_debug(const stm32_lib::gpio::pin_t& p)
{
	p.set(/*stm32_lib::gpio::mode_t::output,*/ stm32_lib::gpio::af_t(SUBGHZSPI_DEBUG_AF));
}


freertos_utils::task_data_t<1024> task_data_ipcc_recv;

void task_function_ipcc_recv(void*)
{
	logger.log_async("CM4: IPCC_RECV started\r\n");
	NVIC_SetPriority(IPCC_C1_RX_IRQn, 10); // TODO
	NVIC_EnableIRQ(IPCC_C1_RX_IRQn);
	IPCC->C1CR |= IPCC_C1CR_RXOIE;
	for (;;) {
		stm32_lib::ipcc::c1_unmask_rx_int<ipcc_ch_lora>();
		if (xTaskNotifyWait(0, 0xffffffff, nullptr, portMAX_DELAY) != pdTRUE) {
			logger.log_async("CM4: notify timeout\r\n");
			continue;
		}

		if (! (IPCC->C2TOC1SR & (1 << ipcc_ch_lora))) {
			logger.log_async("CM4: stray notify\r\n");
			continue;
		}

		vTaskDelay(2);
		logger.log_async("CM4: IPCC_RECV has data\r\n");
		// read mem barrier/flush read cache
		__DSB();
		auto const mb_ptr = mailbox::get_mb_ptr(mb_ptr_addr);
		auto const mb_type = mailbox::get_mb_type(mb_ptr);
		switch (mb_type) {
			case mailbox::lora_packet:
				g_pin_blue.pulse_once(configTICK_RATE_HZ/2);
				logger.log_async("CM4: mailbox/lora_packet\r\n");
				{
					const auto* const mp = mailbox::mb_cast<mailbox::lora_packet>(mb_ptr);
					logger.log_async(reinterpret_cast<const char*>(mp->data.data()));
					logger.log_async("RssiPkt SnrPkt SignalRssiPkt\r\n");
					static std::array<char, 16> logbuf;
					auto p = logbuf.data();
					p = printf_byte(mp->rssi_pkt, p);
					*p++ = ' ';
					p = printf_byte(mp->snr_pkt, p);
					*p++ = ' ';
					p = printf_byte(mp->signal_rssi_pkt, p);
					*p++ = '\r';
					*p++ = '\n';
					*p = 0;
					logger.log_async(logbuf.data());
				}
				vTaskDelay(configTICK_RATE_HZ);
				break;
			case mailbox::string0:
				g_pin_blue.pulse_once(configTICK_RATE_HZ/20);
				logger.log_async("CM4: mailbox/string0\r\n");
				{
					const auto* const mp = mailbox::mb_cast<mailbox::string0>(mb_ptr);
					logger.log_async(mp->s);
				}
				vTaskDelay(configTICK_RATE_HZ);
				break;
			default:
				logger.log_async("CM4: mailbox: unknown type\r\n");
		}
		stm32_lib::ipcc::c1_mark_received<ipcc_ch_lora>();
	}
}

void create_task_ipcc_recv()
{
	task_data_ipcc_recv.task_handle = xTaskCreateStatic(
		&task_function_ipcc_recv,
		"ipcc_recv_cm4",
		task_data_ipcc_recv.stack.size(),
		nullptr,
		PRIO_IPCC_RECV,
		task_data_ipcc_recv.stack.data(),
		&task_data_ipcc_recv.task_buffer
	);
}


extern "C" __attribute__ ((interrupt)) void IntHandler_IPCC_CPU1_RX()
{
	auto const sr = IPCC->C2TOC1SR;
	if (sr & (1 << ipcc_ch_lora)) {
		stm32_lib::ipcc::c1_mask_rx_int<ipcc_ch_lora>();

		BaseType_t yield = pdFALSE;
		xTaskNotifyFromISR(task_data_ipcc_recv.task_handle, 0, eSetBits, &yield);
		portYIELD_FROM_ISR(yield);
	}
}


extern "C" __attribute__ ((interrupt)) void IntHandler_Dma1Ch1()
{
	auto const isr = DMA1->ISR;
	uint32_t events = 0;
	if (isr & DMA_ISR_TCIF1) {
		DMA1->IFCR = DMA_IFCR_CTCIF1;
		events |= stm32_lib::dma::dma_result_t::tc;
	}
	if (isr & DMA_ISR_TEIF1) {
		DMA1->IFCR = DMA_IFCR_CTEIF1;
		events |= stm32_lib::dma::dma_result_t::te;
	}
	if (events) {
		logger.notify_from_isr(events);
	}
}


__attribute__ ((noreturn)) void main()
{
	bus_init();
	stm32_lib::usart::init_logger_lpuart<configCPU_CLOCK_HZ>(
		USART_STLINK,
		bsp::usart_stlink_pin_tx,
		USART_STLINK_PIN_TX_AF
	);

	log_sync("\r\nLogger initialized (sync)\r\n");

#if 0
	set_pin_debug(bsp::pin_debug_subghzspi_nss);
	set_pin_debug(bsp::pin_debug_subghzspi_sck);
	set_pin_debug(bsp::pin_debug_subghzspi_miso);
	set_pin_debug(bsp::pin_debug_subghzspi_mosi);
#endif

	g_pin_blue.init_pin();
	g_pin_green.init_pin();
	g_pin_green.pulse_continuous(configTICK_RATE_HZ/30, configTICK_RATE_HZ/10);
	//g_pin_red.init_pin();
	PWR->CR4 |= PWR_CR4_C2BOOT;

	create_task_ipcc_recv();

	log_sync("CM4: Starting FreeRTOS scheduler\r\n");
	logger.init();
	vTaskStartScheduler();

	log_sync("CM4: Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

