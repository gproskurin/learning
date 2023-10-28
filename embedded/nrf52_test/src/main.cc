#include "FreeRTOS.h"
#include "task.h"

#include "freertos_utils.h"
//#include "logging.h"
#include "lib_nrf5.h"
#include "bsp.h"


#include <stdint.h>
#include <string.h>

#define PRIO_BLINK 1
#define PRIO_NRF52 2


//#define USART_CON_BAUDRATE 115200


auto g_pin_led1 = freertos_utils::make_pin_toggle_task("blink_led1", bsp::pin_led_1, PRIO_BLINK);
auto g_pin_led2 = freertos_utils::make_pin_toggle_task("blink_led2", bsp::pin_led_2, PRIO_BLINK);
auto g_pin_led3 = freertos_utils::make_pin_toggle_task("blink_led3", bsp::pin_led_3, PRIO_BLINK);
auto g_pin_led4 = freertos_utils::make_pin_toggle_task("blink_led4", bsp::pin_led_4, PRIO_BLINK);


#if 0
usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_FIFOEN | USART_CR1_TE;

	stm32_lib::gpio::set_mode_af_lowspeed_pu(bsp::usart_stlink_pin_tx, USART_STLINK_PIN_TX_AF);
	usart->BRR = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;

	usart->CR1 = cr1;
	usart->CR1 = cr1 | USART_CR1_UE;
}
#endif


#if 0
void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}
#endif


void periph_init()
{
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
	//__WFI();
}


void nrf52_task_function(void* arg)
{
	//logger.log_async("NRF-52 task started\r\n");

	g_pin_led1.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/50*49);
	g_pin_led2.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ);
	g_pin_led3.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ);
	g_pin_led4.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/5);

	for (;;) {
		vTaskDelay(configTICK_RATE_HZ);
	}
}


freertos_utils::task_data_t<128> nrf52_task_data;

void create_nrf52_task(const char* task_name, UBaseType_t prio)
{
	nrf52_task_data.task_handle = xTaskCreateStatic(
		&nrf52_task_function,
		task_name,
		nrf52_task_data.stack.size(),
		nullptr,
		prio,
		nrf52_task_data.stack.data(),
		&nrf52_task_data.task_buffer
	);
}


__attribute__ ((noreturn)) void main()
{
	periph_init();
	g_pin_led1.init_pin();
	g_pin_led2.init_pin();
	g_pin_led3.init_pin();
	g_pin_led4.init_pin();

	//logger.log_sync("Creating NRF-52 task...\r\n");
	create_nrf52_task("nrf52_task", PRIO_NRF52);
	//logger.log_sync("Created NRF-52 task\r\n");

	//logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	//logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

