#include "lib_stm32.h"
#include "freertos_utils.h"
#include "logging.h"

#include "lora.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>


#define PRIO_BLINK 1
#define PRIO_LORA 3
#define PRIO_LOGGER 2

const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 5);
const stm32_lib::gpio::gpio_pin_t pin_led_blue(GPIOB, 6);
const stm32_lib::gpio::gpio_pin_t pin_led_red(GPIOB, 7);
const stm32_lib::gpio::gpio_pin_t pin_led_green2(GPIOA, 5);
const stm32_lib::gpio::gpio_pin_t pin_userbutton(GPIOB, 2);

// USART2 (st-link vcom)
#define USART_LOG USART2
#define USART_LOG_AF 4
const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOA, 2);

#define CLOCK_SPEED configCPU_CLOCK_HZ
#define USART_CON_BAUDRATE 115200

namespace sx1276 {
	struct hwconf_t;
	extern const hwconf_t hwc;
};

usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_TE;

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


void bus_init()
{
#if defined TARGET_STM32L072
	// flash
	FLASH->ACR |= FLASH_ACR_LATENCY;

	// clock: switch from MSI to HSI
	// turn on HSI & wait
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY)) {}
	// switch to HSI
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (0b01 << RCC_CFGR_SW_Pos);
	// switch off MSI & wait
	RCC->CR &= ~RCC_CR_MSION;
	while (RCC->CR & RCC_CR_MSIRDY) {}

	// GPIOs
	RCC->IOPENR = RCC_IOPENR_IOPAEN_Msk | RCC_IOPENR_IOPBEN_Msk | RCC_IOPENR_IOPCEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_IOPARST | RCC_IOPRSTR_IOPBRST | RCC_IOPRSTR_IOPCRST);

	// USART2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_USART2RST_Msk);

	// SPI1 & SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk | RCC_APB2ENR_SYSCFGEN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST_Msk | RCC_APB2RSTR_SYSCFGRST_Msk);

#if 0
	// PB2 interrupt TODO: do not hardcode PB2, use bit masks stuff
	{
		auto const cr = &SYSCFG->EXTICR[pin_userbutton.reg / 4];
		*cr = (*cr & ~(SYSCFG_EXTICR1_EXTI2_Msk)) | SYSCFG_EXTICR1_EXTI2_PB;
	}

	// EXTI
	pin_userbutton.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::no_pupd);
	EXTI->IMR |= (1 << pin_userbutton.reg);
	EXTI->RTSR &= ~(1 << pin_userbutton.reg); // disable rising edge interrupt (button release)
	EXTI->FTSR |= (1 << pin_userbutton.reg); // enable falling edge interrupt (button press)
	NVIC_SetPriority(EXTI2_3_IRQn, 13);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
#endif
#endif
}


volatile bool blue_state = false;
extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI23()
{
	if (EXTI->PR & (1 << pin_userbutton.reg)) {
		//stm32_lib::gpio::set_state(pin_led_blue, (blue_state = !blue_state));
		EXTI->PR = (1 << pin_userbutton.reg);
	}
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


lora::task_data_t lora_task_data;


freertos_utils::pin_toggle_task_t g_pin_green2("blink_green2", pin_led_green2, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", pin_led_red, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green("blink_green", pin_led_green, PRIO_BLINK);


__attribute__ ((noreturn)) void main()
{
	bus_init();

	usart_init(USART_LOG);
	logger.set_usart(USART_LOG);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");

	g_pin_green.init_pin();
	g_pin_green2.init_pin();
	g_pin_green2.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/25);
	g_pin_blue.init_pin();
	g_pin_red.init_pin();

	logger.log_sync("Creating LORA task...\r\n");
	lora::create_task("lora", PRIO_LORA, lora_task_data, &sx1276::hwc);
	logger.log_sync("Created LORA task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

