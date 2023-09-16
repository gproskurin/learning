#ifndef _my_freertos_utils_included_
#define _my_freertos_utils_included_

#include "lib_stm32.h"

#include <array>


namespace freertos_utils {


template <size_t StackSize>
using task_stack_t = std::array<StackType_t, StackSize>;


class pin_toggle_task_t {
public:
	pin_toggle_task_t(const char* task_name, const stm32_lib::gpio::gpio_pin_t&, UBaseType_t prio);

	void init_pin() const;

	void on();
	void off();
	void pulse_once(TickType_t tm_on);
	void pulse_many(TickType_t tm_on, TickType_t tm_off, uint8_t times);
	void pulse_continuous(TickType_t tm_on, TickType_t tm_off);

private:
	const stm32_lib::gpio::gpio_pin_t pin_;

	task_stack_t<64> stack_;
	StaticTask_t task_buffer_;
	TaskHandle_t const task_handle_;

	static void task_function(void*);
};


} // namespace

#endif

