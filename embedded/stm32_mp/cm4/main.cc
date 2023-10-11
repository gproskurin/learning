#include "lib_stm32.h"
#include "bsp.h"
#include "logging.h"
#include "freertos_utils.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>


#define PRIO_BLINK 1
#define PRIO_LOGGER 2


#define USART_CON_BAUDRATE 115200


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


StaticTask_t xTaskBufferIdle;
freertos_utils::task_stack_t<128> idle_task_stack;
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
freertos_utils::pin_toggle_task_t g_pin_red_otg("blink_red_otg", bsp::pin_led_red_otg_overcurrent, PRIO_BLINK);


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

	g_pin_red.init_pin();
	g_pin_red.pulse_continuous(configTICK_RATE_HZ/10, configTICK_RATE_HZ);

	g_pin_red_otg.init_pin();
	g_pin_red_otg.pulse_continuous(configTICK_RATE_HZ/10, configTICK_RATE_HZ);

	usart_init(USART_STLINK);
	logger.set_usart(USART_STLINK);
	logger.log_sync("\r\nCM4: Logger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	for (;;) {}
}

