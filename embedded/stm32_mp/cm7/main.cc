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


void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


void periph_init()
{
	RCC->AHB4ENR |=
		RCC_AHB4ENR_GPIOAEN_Msk
		| RCC_AHB4ENR_GPIOBEN_Msk
		| RCC_AHB4ENR_GPIODEN_Msk
		| RCC_AHB4ENR_GPIOHEN_Msk
		| RCC_AHB4ENR_GPIOIEN_Msk
		| RCC_AHB4ENR_GPIOJEN_Msk;
	toggle_bits_10(
		&RCC->AHB4RSTR,
		RCC_AHB4RSTR_GPIOARST_Msk
			| RCC_AHB4RSTR_GPIOBRST_Msk
			| RCC_AHB4RSTR_GPIODRST_Msk
			| RCC_AHB4RSTR_GPIOHRST_Msk
			| RCC_AHB4RSTR_GPIOIRST_Msk
			| RCC_AHB4RSTR_GPIOJRST_Msk
	);

	// USART
	RCC->APB1LENR |= RCC_APB1LENR_USART3EN_Msk;
	toggle_bits_10(&RCC->APB1LRSTR, RCC_APB1LRSTR_USART3RST_Msk);
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


freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green_arduino("blink_green_arduino", bsp::pin_led_green_arduino, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green_vbus("blink_green_vbus", bsp::pin_led_green_vbus_usb_fs, PRIO_BLINK);


__attribute__ ((noreturn)) void main()
{
#if 0
	// clear event
	__SEV();
	for (volatile int i=0; i<1000000; ++i) {}
	__WFE();
	for (volatile int i=0; i<1000000; ++i) {}

	if (1) {
		__WFE(); // wait for "START" event from CM4

		__SEV(); // wakeup CM4
		__WFE(); // clear our wakeup event
	}
#endif
	periph_init();

	g_pin_green.init_pin();
	g_pin_green.pulse_continuous(configTICK_RATE_HZ, configTICK_RATE_HZ*2);

	g_pin_green_arduino.init_pin();
	g_pin_green_arduino.pulse_continuous(configTICK_RATE_HZ/20, configTICK_RATE_HZ/11);

	g_pin_green_vbus.init_pin();
	g_pin_green_vbus.pulse_continuous(configTICK_RATE_HZ/2, configTICK_RATE_HZ);

	usart_init(USART_STLINK);
	logger.set_usart(USART_STLINK);
	logger.log_sync("\r\nCM7: Logger initialized (sync)\r\n");

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

