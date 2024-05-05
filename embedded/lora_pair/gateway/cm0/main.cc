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
#define PRIO_LORA_RECV 5
#define PRIO_LOGGER 8 // FIXME


#define USART_CON_BAUDRATE 115200


usart_logger_t logger(USART_STLINK, "logger_cm0", PRIO_LOGGER);


void bus_init()
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
	__WFI();
}


freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);


freertos_utils::task_data_t<1024> task_data_lora_recv;

void task_function_lora_recv(void*)
{
	logger.log_async("CM0: LORA_RECV task started\r\n");
	for(;;) {
		vTaskDelay(configTICK_RATE_HZ*10);
		logger.log_async("CM0: keep-alive\r\n");
	}
}

void create_task_lora_recv()
{
	task_data_lora_recv.task_handle = xTaskCreateStatic(
		&task_function_lora_recv,
		"LORA_RECV",
		task_data_lora_recv.stack.size(),
		nullptr,
		PRIO_LORA_RECV,
		task_data_lora_recv.stack.data(),
		&task_data_lora_recv.task_buffer
	);
}


__attribute__ ((noreturn)) void main()
{
	bus_init();

	logger.log_sync("\r\nLogger_cm0 initialized (sync)\r\n");

	g_pin_red.init_pin();
	g_pin_red.pulse_continuous(configTICK_RATE_HZ/10, configTICK_RATE_HZ*9/10);

	logger.log_sync("Creating LORA_RECV task...\r\n");
	create_task_lora_recv();
	logger.log_sync("Created LORA_RECV task\r\n");

	logger.log_sync("CM0: Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

