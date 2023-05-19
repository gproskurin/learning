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


void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


void periph_init()
{
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOIEN_Msk | RCC_AHB4ENR_GPIOJEN_Msk;
	toggle_bits_10(
		&RCC->AHB4RSTR,
		RCC_AHB4RSTR_GPIOIRST_Msk | RCC_AHB4RSTR_GPIOJRST_Msk
	);

	// USART
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_USART1RST_Msk);
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
			"blink_green",
			pin_led_green,
			configTICK_RATE_HZ/10,
			configTICK_RATE_HZ/10*2
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

        if (1) {
                __WFE(); // wait for "START" event from CM4

                __SEV(); // wakeup CM4
                __WFE(); // clear our wakeup event
        }
#endif
	periph_init();

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

