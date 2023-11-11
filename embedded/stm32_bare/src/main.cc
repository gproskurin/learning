#include "lib_stm32.h"
#include "freertos_utils.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>


#define PRIO_BLINK 1
#define PRIO_T1 2
#define PRIO_LOGGER 3

constexpr stm32_lib::gpio::pin_t pin_led_red{GPIOA_BASE,3};
constexpr stm32_lib::gpio::pin_t pin_led_greep{GPIOA_BASE,4};

#define USART_CON_BAUDRATE 115200
#define USART_LOG USART1
#define USART_LOG_TX_AF 1
constexpr stm32_lib::gpio::pin_t pin_usart_tx{GPIOA_BASE, 9};


usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_FIFOEN | USART_CR1_TE;

#ifdef TARGET_STM32F103
	stm32_lib::gpio::set_mode_af_lowspeed_pu(usart_stlink_pin_tx);
	const uint32_t div = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;
	usart->BRR = ((div / 16) << USART_BRR_DIV_Mantissa_Pos) | ((div % 16) << USART_BRR_DIV_Fraction_Pos);
#else
	stm32_lib::gpio::set_mode_af_lowspeed_pu(pin_usart_tx, USART_LOG_TX_AF);
	usart->BRR = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;
#endif

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
#if defined TARGET_STM32G030
	// GPIOs
	RCC->IOPENR = RCC_IOPENR_GPIOAEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_GPIOARST);

	// USART1
	RCC->APBENR2 = RCC_APBENR2_USART1EN_Msk | RCC_APBENR2_SYSCFGEN_Msk;
	toggle_bits_10(&RCC->APBRSTR2, RCC_APBRSTR2_USART1RST_Msk | RCC_APBRSTR2_SYSCFGRST_Msk);

	// enable PA9 in place of PA11
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP;
#endif
}


StaticTask_t xTaskBufferIdle;
using idle_task_stack_t = freertos_utils::task_stack_t<64>;
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


#ifdef TARGET_STM32G030
freertos_utils::pin_toggle_task_t g_pin_led_red("blink_red", pin_led_red, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_led_green("blink_green", pin_led_greep, PRIO_BLINK);
#endif


void task_function_t1(void*)
{
	logger.log_async("T1 task started\r\n");
	g_pin_led_green.pulse_continuous(configTICK_RATE_HZ/30, configTICK_RATE_HZ/10);
	for(;;) {
		logger.log_async("T1 keep-alive\r\n");
		vTaskDelay(configTICK_RATE_HZ*10);
	}
}


freertos_utils::task_data_t<128> task_data_t1;
void create_task_t1()
{
	task_data_t1.task_handle = xTaskCreateStatic(
		&task_function_t1,
		"t1",
		task_data_t1.stack.size(),
		nullptr,
		PRIO_T1,
		task_data_t1.stack.data(),
		&task_data_t1.task_buffer
	);
}



__attribute__ ((noreturn)) void main()
{
	periph_init();

#if 1
	usart_init(USART_LOG);
	logger.set_usart(USART_LOG);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");
#endif

	g_pin_led_red.init_pin();
	g_pin_led_green.init_pin();
	g_pin_led_red.pulse_continuous(configTICK_RATE_HZ/20, configTICK_RATE_HZ/20*19);
	//g_pin_led_green.pulse_continuous(configTICK_RATE_HZ/30, configTICK_RATE_HZ/10);

	logger.log_sync("Creating task T1...\r\n");
	create_task_t1();
	logger.log_sync("Created task T1\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {
	}
}

