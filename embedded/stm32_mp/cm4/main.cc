#include "lib_stm32.h"
#include "logging.h"
#include "freertos_utils.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>


#define PRIO_BLINK 1
#define PRIO_LOGGER 2

const stm32_lib::gpio::gpio_pin_t pin_led_red(GPIOI, 13);
const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOJ, 2);

// USART1, tx(PB6)
#define USART_LOG USART1
#define USART_LOG_AF 7
const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOB, 6);


#define CLOCK_SPEED configCPU_CLOCK_HZ
#define USART_CON_BAUDRATE 115200


usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_FIFOEN | USART_CR1_TE;

	stm32_lib::gpio::set_mode_af_lowspeed_pu(usart_log_pin_tx, USART_LOG_AF);
	usart->BRR = CLOCK_SPEED / USART_CON_BAUDRATE;

	usart->CR1 = cr1;
	usart->CR1 = cr1 | USART_CR1_UE;
}


StaticTask_t xTaskBufferIdle;
using idle_task_stack_t = freertos_utils::task_stack_t<128>;
idle_task_stack_t idle_task_stack;
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


struct blink_tasks_t {
	std::array<freertos_utils::blink_task_data_t, 1> tasks = {
		freertos_utils::blink_task_data_t(
			"blink_red",
			pin_led_red,
			configTICK_RATE_HZ/2,
			configTICK_RATE_HZ/2
		)
	};
} blink_tasks;


void str_cpy_3(char* dst, const char* s1, const char* s2, const char* s3)
{
	auto p = stpcpy(dst, s1);
	p = stpcpy(p, s2);
	stpcpy(p, s3);
}


__attribute__ ((noreturn)) void main()
{
#if 0
	// clear event
	__SEV();
	for (volatile int i=0; i<1000000; ++i) {}
	__WFE();
	for (volatile int i=0; i<1000000; ++i) {}

	// goto STOP mode and wait for CM7 to init periph and wakeup us
	if (1) {
		__SEV(); // signal "START" event to CM7
		__WFE(); // clear pending events

		//PWR->CR1 &= ~PWR_CR1_LPDS;
		//PWR->CPU2CR &= ~PWR_CPUCR_PDDS_D2_Msk; // keep STOP mode when in DEEPSLEEP
		//SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		//__DSB();
		//__ISB();
		//__WFE(); // sleep
		//SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	}
#endif
	for (volatile int i=0; i<1000000; ++i) {} // wait for other core to init periph

	usart_init(USART_LOG);
	logger.set_usart(USART_LOG);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");

	logger.log_sync("Creating blink tasks...\r\n");
	for (auto& bt : blink_tasks.tasks) {
		stm32_lib::gpio::set_mode_output_lowspeed_pushpull(bt.pin);
		freertos_utils::create_blink_task(bt, PRIO_BLINK);
	}
	logger.log_sync("Created blink tasks\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	for (;;) {}
}

