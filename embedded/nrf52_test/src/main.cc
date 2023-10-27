//#include "freertos_utils.h"
//#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>

#include <array>

#include <nrf52.h>

#define PRIO_NRF52 1


//#define USART_CON_BAUDRATE 115200

const std::array<uint8_t, 4> pin_leds{17, 19, 20, 18};

template <size_t StackSize>
using task_stack_t = std::array<StackType_t, StackSize>;

template <size_t StackSize>
struct task_data_t {
	task_stack_t<StackSize> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
};

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
	for (const auto p : pin_leds) {
		NRF_P0->PIN_CNF[p] = 0b11;
	}
}


StaticTask_t xTaskBufferIdle;
task_stack_t<64> idle_task_stack;
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


char* print_bits(uint8_t x, char* buf)
{
	for (size_t i=0; i<8; ++i) {
		*buf++ = (x & 0b10000000) ? '1' : '0';
		x <<= 1;
	}
	*buf++ = '\r';
	*buf++ = '\n';
	*buf = 0;
	return buf;
}


void nrf52_task_function(void* arg)
{
	//logger.log_async("NRF-52 task started\r\n");
	for (;;) {
		for (const auto p : pin_leds) {
			NRF_P0->OUTCLR = (1 << p);
			vTaskDelay(configTICK_RATE_HZ/10);
			NRF_P0->OUTSET = (1 << p);
			vTaskDelay(configTICK_RATE_HZ/5);
		}
		vTaskDelay(configTICK_RATE_HZ/2);
	}
}


task_data_t<1024> task_data;

void create_nrf52_task(const char* task_name, UBaseType_t prio)
{
	task_data.task_handle = xTaskCreateStatic(
		&nrf52_task_function,
		task_name,
		task_data.stack.size(),
		nullptr,
		prio,
		task_data.stack.data(),
		&task_data.task_buffer
	);
}


__attribute__ ((noreturn)) void main()
{
	periph_init();

	//logger.log_sync("Creating NRF-52 task...\r\n");
	create_nrf52_task("nrf52_task", PRIO_NRF52);
	//logger.log_sync("Created NRF-52 task\r\n");

	//logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	//logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

