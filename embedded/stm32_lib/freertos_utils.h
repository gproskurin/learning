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


namespace pinpoll {


typedef void (*pin_cb_t)(const stm32_lib::gpio::gpio_pin_t& pin, bool new_status);

namespace impl {
        constexpr uint8_t poll_history_mask = 0b1111;

        struct pin_info_t {
                const stm32_lib::gpio::gpio_pin_t pin;
                const pin_cb_t cb;
                pin_info_t(const stm32_lib::gpio::pin_t& p, pin_cb_t c) : pin(p), cb(c) {}

                // internal data for poll function
                uint8_t poll_history = 0xff;
                bool pin_status = true;
        };
}

template <typename... Args>
impl::pin_info_t make_pin_info(Args&&... args)
{
        return impl::pin_info_t(std::forward<Args>(args)...);
}


template <size_t Size>
struct task_arg_t {
        std::array<impl::pin_info_t, Size> pin_info;

        template <typename ...Args>
        task_arg_t(Args&&... args) : pin_info{std::forward<Args>(args)...} {}

        task_stack_t<128> task_stack_;
	StaticTask_t task_buffer_;
	TaskHandle_t task_handle_ = nullptr;
};


template <typename ...Args>
task_arg_t<sizeof...(Args)> make_task_arg(Args&&... args)
{
        return task_arg_t<sizeof...(Args)>(std::forward<Args>(args)...);
}


namespace impl {
        void task_function(pin_info_t* pin_info_items, size_t size);
}


template <size_t Size>
void task_function(void *arg)
{
        using arg_t = task_arg_t<Size>;
        arg_t* const task_arg = reinterpret_cast<arg_t*>(arg);
        return impl::task_function(task_arg->pin_info.data(), Size);
}


template <size_t Size>
void create_task(const char* task_name, UBaseType_t prio, task_arg_t<Size>* arg)
{
	arg->task_handle_ = xTaskCreateStatic(
		&task_function<Size>,
		task_name,
		arg->task_stack_.size(),
		reinterpret_cast<task_arg_t<Size>*>(arg),
		prio,
		arg->task_stack_.data(),
		&arg->task_buffer_
	);
}

} // namespace pin_poll

} // namespace

#endif

