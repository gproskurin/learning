#include "lib_stm32.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>


#define PRIO_BLINK 1
#define PRIO_LORA 2
#define PRIO_LOGGER 2

#if defined TARGET_STM32L072
	const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 5);
	const stm32_lib::gpio::gpio_pin_t pin_led_blue(GPIOB, 6);
	const stm32_lib::gpio::gpio_pin_t pin_led_red(GPIOB, 7);
	const stm32_lib::gpio::gpio_pin_t pin_led_green2(GPIOA, 5);

	// USART1, tx(PA9)
	#define USART_LOG USART1
	const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOA, 9);
	//#define USART_LOG_PIN_RX 10
	#define USART_LOG_AF 4

	// SX1276 SPI
	#define SPI_SX1276 SPI2
	#define SPI_SX1276_AF 0
	const stm32_lib::gpio::gpio_pin_t pin_sxspi_sck(GPIOB, 13);
	const stm32_lib::gpio::gpio_pin_t pin_sxspi_mosi(GPIOB, 15);
	const stm32_lib::gpio::gpio_pin_t pin_sxspi_miso(GPIOB, 14);
	const stm32_lib::gpio::gpio_pin_t pin_sxspi_nss(GPIOB, 12);
#endif


#define CLOCK_SPEED configCPU_CLOCK_HZ
#define USART_CON_BAUDRATE 115200


template <size_t StackSize>
using task_stack_t = std::array<StackType_t, StackSize>;

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


void delay(int val)
{
	for (volatile int i = 0; i<val; ++i) {
	}
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
	// enable GPIOs
	RCC->IOPENR |= RCC_IOPENR_IOPAEN_Msk | RCC_IOPENR_IOPBEN_Msk;

	// USART & SPI1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk | RCC_APB2ENR_SPI1EN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_USART1RST_Msk | RCC_APB2RSTR_SPI1RST_Msk);

	// TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_TIM2RST_Msk);
#endif
}


void do_blink(const stm32_lib::gpio::gpio_pin_t& pin, int n)
{
	while (n-- > 0) {
		stm32_lib::gpio::set_state(pin, 1);
		delay(500000);
		stm32_lib::gpio::set_state(pin, 0);
		delay(100000);
	}
}


StaticTask_t xTaskBufferIdle;
using idle_task_stack_t = task_stack_t<128>;
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


// LED blinking tasks
struct blink_task_data_t {
	const char* const task_name;
	const stm32_lib::gpio::gpio_pin_t pin;
	const TickType_t ticks_on;
	const TickType_t ticks_off;
	blink_task_data_t(const char* tname, const stm32_lib::gpio::gpio_pin_t& p, TickType_t ton, TickType_t toff)
		: task_name(tname), pin(p), ticks_on(ton), ticks_off(toff)
	{}

	task_stack_t<128> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
};

struct blink_tasks_t {
#if defined TARGET_STM32L072
	std::array<blink_task_data_t, 1> tasks = {
		blink_task_data_t(
			"blink_green2",
			pin_led_green2,
			configTICK_RATE_HZ/2,
			configTICK_RATE_HZ/2
		)
	};
#endif
} blink_tasks;

void blink_task_function(void* arg)
{
	const blink_task_data_t* const args = reinterpret_cast<blink_task_data_t*>(arg);
	for(int i=0;;++i) {
		const bool do_log = (i % 64 == 0);
		if (do_log) {
			logger.log_async("LED -> on\r\n");
		}
		stm32_lib::gpio::set_state(args->pin, true);
		vTaskDelay(args->ticks_on);
		if (do_log) {
			logger.log_async("LED -> off\r\n");
		}
		stm32_lib::gpio::set_state(args->pin, false);
		vTaskDelay(args->ticks_off);
	}
}


// LORA task
struct lora_task_data_t {
	const char* const task_name;
	lora_task_data_t(const char* tname) : task_name(tname) {}

	task_stack_t<128> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
} lora_task_data("lora");

void lora_task_function(void* arg)
{
	const lora_task_data_t* const args = reinterpret_cast<lora_task_data_t*>(arg);
	stm32_lib::gpio::set_mode_output_lowspeed_pushpull(pin_led_blue);
	for(int i=0;;++i) {
		logger.log_async("LORA keepalive\r\n");
		vTaskDelay(configTICK_RATE_HZ * 2);
		stm32_lib::gpio::set_state(pin_led_blue, true);
		vTaskDelay(configTICK_RATE_HZ * 5);
		stm32_lib::gpio::set_state(pin_led_blue, false);
	}
}

void create_lora_task(lora_task_data_t& args)
{
	args.task_handle = xTaskCreateStatic(
		&lora_task_function,
		args.task_name,
		args.stack.size(),
		reinterpret_cast<void*>(&args),
		PRIO_LORA,
		args.stack.data(),
		&args.task_buffer
	);
}


void str_cpy_3(char* dst, const char* s1, const char* s2, const char* s3)
{
	auto p = stpcpy(dst, s1);
	p = stpcpy(p, s2);
	stpcpy(p, s3);
}


void create_blink_task(blink_task_data_t& args)
{
	char log_buf[64];

	str_cpy_3(log_buf, "Creating blink task \"", args.task_name, "\" ...\r\n");
	logger.log_sync(log_buf);

	args.task_handle = xTaskCreateStatic(
		&blink_task_function,
		args.task_name,
		args.stack.size(),
		reinterpret_cast<void*>(&args),
		PRIO_BLINK,
		args.stack.data(),
		&args.task_buffer
	);

	str_cpy_3(log_buf, "Created blink task \"", args.task_name, "\"\r\n");
	logger.log_sync(log_buf);
}


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

	logger.log_sync("Creating blink tasks...\r\n");
	for (auto& bt : blink_tasks.tasks) {
		stm32_lib::gpio::set_mode_output_lowspeed_pushpull(bt.pin);
		create_blink_task(bt);
	}
	logger.log_sync("Created blink tasks\r\n");

	logger.log_sync("Creating LORA task...\r\n");
	create_lora_task(lora_task_data);
	logger.log_sync("Created LORA task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {
		//blink(2);
		//delay(100000);
		//__WFI();
	}
}

