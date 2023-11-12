#include "lib_stm32.h"
#include "freertos_utils.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>


#define PRIO_TL 1

struct task_data_tl3_t {
	freertos_utils::task_data_t<64> task_data;
	const stm32_lib::gpio::pin_t pin_red;
	const stm32_lib::gpio::pin_t pin_yellow;
	const stm32_lib::gpio::pin_t pin_green;
};

task_data_tl3_t g_td_tl3_1{
	.pin_red{GPIOA_BASE, 2},
	.pin_yellow{GPIOA_BASE, 3},
	.pin_green{GPIOA_BASE, 4}
};


static
void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


static
void periph_init()
{
#if defined TARGET_STM32G030
	// GPIOs
	RCC->IOPENR = RCC_IOPENR_GPIOAEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_GPIOARST);

	//// USART1
	//RCC->APBENR2 = RCC_APBENR2_USART1EN_Msk | RCC_APBENR2_SYSCFGEN_Msk;
	//toggle_bits_10(&RCC->APBRSTR2, RCC_APBRSTR2_USART1RST_Msk | RCC_APBRSTR2_SYSCFGRST_Msk);

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


static
void task_function_tl3(void* arg)
{
	auto const tl3 = reinterpret_cast<const task_data_tl3_t*>(arg);

	tl3->pin_red.set_mode_output_lowspeed_pushpull();
	tl3->pin_yellow.set_mode_output_lowspeed_pushpull();
	tl3->pin_green.set_mode_output_lowspeed_pushpull();

	tl3->pin_yellow.set_state(0);
	tl3->pin_green.set_state(0);
	for(;;) {
		// red
		tl3->pin_red.set_state(1);
		vTaskDelay(configTICK_RATE_HZ*4);

		// red + yellow
		tl3->pin_yellow.set_state(1);
		vTaskDelay(configTICK_RATE_HZ);

		// green
		tl3->pin_red.set_state(0);
		tl3->pin_yellow.set_state(0);
		tl3->pin_green.set_state(1);
		vTaskDelay(configTICK_RATE_HZ*4);

		// flashing green
		for(int i=0; i<3; ++i) {
			tl3->pin_green.set_state(0);
			vTaskDelay(configTICK_RATE_HZ/2);
			tl3->pin_green.set_state(1);
			vTaskDelay(configTICK_RATE_HZ/2);
		}

		// yellow
		tl3->pin_green.set_state(0);
		tl3->pin_yellow.set_state(1);
		vTaskDelay(configTICK_RATE_HZ);

		// goto red
		tl3->pin_yellow.set_state(0);
	}
}


static
void create_task_tl3(task_data_tl3_t* const td)
{
	td->task_data.task_handle = xTaskCreateStatic(
		&task_function_tl3,
		"tl3_1",
		td->task_data.stack.size(),
		const_cast<void*>(reinterpret_cast<const void*>(td)),
		PRIO_TL,
		td->task_data.stack.data(),
		&td->task_data.task_buffer
	);
}



__attribute__ ((noreturn)) void main()
{
	periph_init();
	create_task_tl3(&g_td_tl3_1);
	vTaskStartScheduler();
	for(;;) {}
}

