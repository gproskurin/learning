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
#define PRIO_TEST1 2
#define PRIO_LOGGER 8 // FIXME


#define USART_CON_BAUDRATE 115200


usart_logger_t logger(USART_STLINK, "logger", PRIO_LOGGER);


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


freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);

freertos_utils::task_data_t<1024> task_data_test1;

void task_function_test1(void*)
{
	logger.log_async("TEST-1 task started\r\n");
	for(;;) {
		vTaskDelay(configTICK_RATE_HZ);
		//logger.log_async("TEST-1 task keep-alive\r\n");
	}
}

void create_task_test1()
{
	task_data_test1.task_handle = xTaskCreateStatic(
		&task_function_test1,
		"TEST-1",
		task_data_test1.stack.size(),
		nullptr,
		PRIO_TEST1,
		task_data_test1.stack.data(),
		&task_data_test1.task_buffer
	);
}


__attribute__ ((noreturn)) void main()
{
	bus_init();

	usart_init(USART_STLINK);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	g_pin_blue.init_pin();
	g_pin_blue.pulse_continuous(configTICK_RATE_HZ/2, configTICK_RATE_HZ/2);
	g_pin_green.init_pin();
	g_pin_green.pulse_continuous(configTICK_RATE_HZ/30, configTICK_RATE_HZ/10);
	g_pin_red.init_pin();
	g_pin_red.pulse_continuous(configTICK_RATE_HZ/3, configTICK_RATE_HZ*5);

	logger.log_sync("Creating TEST-1 task...\r\n");
	create_task_test1();
	logger.log_sync("Created TEST-1 task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

