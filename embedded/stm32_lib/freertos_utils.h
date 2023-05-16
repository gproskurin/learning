#ifndef _my_freertos_utils_included_
#define _my_freertos_utils_included_

#include "lib_stm32.h"

#include <array>


namespace freertos_utils {


template <size_t StackSize>
using task_stack_t = std::array<StackType_t, StackSize>;

// LED blinking tasks
struct blink_task_data_t {
	const char* const task_name;
	const stm32_lib::gpio::gpio_pin_t pin;
	const TickType_t ticks_on;
	const TickType_t ticks_off;
	blink_task_data_t(const char* n, const stm32_lib::gpio::gpio_pin_t& p, TickType_t ton, TickType_t toff)
		: task_name(n), pin(p), ticks_on(ton), ticks_off(toff)
	{}

	task_stack_t<64> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
};

void blink_task_function(void* arg);
void create_blink_task(blink_task_data_t& args, UBaseType_t prio);

} // namespace

#endif

