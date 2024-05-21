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
#define PRIO_IPCC_RECV 5
#define PRIO_LOGGER 8 // FIXME


constexpr uint8_t ipcc_ch_lora = 3;
constexpr uint32_t mb_ptr_addr = 0x20008088; // keep it up-to-date, taken from CM0 linker map file

#define USART_CON_BAUDRATE 115200


usart_logger_t logger(USART_STLINK, "logger_cm4", PRIO_LOGGER);

freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
//freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_FIFOEN | USART_CR1_TE;

	stm32_lib::gpio::set_mode_af_lowspeed_pu(bsp::usart_stlink_pin_tx, USART_STLINK_PIN_TX_AF);

	{
		constexpr uint64_t baud = USART_CON_BAUDRATE;
		constexpr uint64_t psc = 4;
		constexpr uint32_t psc_reg_val = 0b0010;
		constexpr uint64_t clk = configCPU_CLOCK_HZ / psc;
		constexpr uint64_t brr = clk*256 / baud;
		static_assert(brr >= 0x300);
		static_assert(brr < (1 << 20));
		static_assert(3*baud < clk);
		static_assert(clk < 4096*baud);
		constexpr uint32_t brr_reg_val = brr;

		usart->PRESC = psc_reg_val;
		usart->BRR = brr_reg_val;
	}

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
	// PCLK3
	RCC->EXTCFGR = (RCC->EXTCFGR & ~RCC_EXTCFGR_SHDHPRE_Msk)
		| ((0b0000) << RCC_EXTCFGR_SHDHPRE_Pos)
		;
	while (!(RCC->EXTCFGR & RCC_EXTCFGR_SHDHPREF)) {} // wait for prescaler to be applied

	// switch on HSE
	//RCC->CR |= RCC_CR_HSEON;
	//while (!(RCC->CR & RCC_CR_HSERDY)) {} // TODO

	// GPIOs
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN_Msk | RCC_AHB2ENR_GPIOBEN_Msk | RCC_AHB2ENR_GPIOCEN_Msk;
	toggle_bits_10(
		&RCC->AHB2RSTR,
		RCC_AHB2RSTR_GPIOARST_Msk | RCC_AHB2RSTR_GPIOBRST_Msk | RCC_AHB2RSTR_GPIOCRST_Msk
	);

	// LPUART1
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR2, RCC_APB1RSTR2_LPUART1RST_Msk);
	RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_LPUART1SEL_Msk))
		| (0b01 << RCC_CCIPR_LPUART1SEL_Pos)
		;

	// SUBGHZSPI
	RCC->APB3ENR |= RCC_APB3ENR_SUBGHZSPIEN;
	toggle_bits_10(&RCC->APB3RSTR, RCC_APB3RSTR_SUBGHZSPIRST);

	// IPCC
	RCC->C2AHB3ENR |= RCC_C2AHB3ENR_IPCCEN;
	RCC->AHB3ENR |= RCC_AHB3ENR_IPCCEN;
	toggle_bits_10(&RCC->AHB3RSTR, RCC_AHB3RSTR_IPCCRST);
	//toggle_bits_10(&RCC->C2AHB3RSTR, RCC_C2AHB3RSTR_IPCCRST);
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


__attribute__ ((noreturn)) void main()
{
	bus_init();

#if 0
	set_pin_debug(bsp::pin_debug_subghzspi_nss);
	set_pin_debug(bsp::pin_debug_subghzspi_sck);
	set_pin_debug(bsp::pin_debug_subghzspi_miso);
	set_pin_debug(bsp::pin_debug_subghzspi_mosi);
#endif

	usart_init(USART_STLINK);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	g_pin_blue.init_pin();
	g_pin_green.init_pin();
	g_pin_green.pulse_continuous(configTICK_RATE_HZ/30, configTICK_RATE_HZ/10);
	//g_pin_red.init_pin();
	PWR->CR4 |= PWR_CR4_C2BOOT;

	create_task_ipcc_recv();

	logger.log_sync("CM4: Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("CM4: Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

