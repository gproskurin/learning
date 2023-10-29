#ifndef _my_freertos_utils_included_
#define _my_freertos_utils_included_

#include "FreeRTOS.h"
#include "task.h"

#include <algorithm>
#include <array>
#include <utility>


namespace freertos_utils {


template <size_t StackSize>
using task_stack_t = std::array<StackType_t, StackSize>;


template <size_t StackSize>
struct task_data_t {
	task_stack_t<StackSize> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
};


namespace pin_toggle_task_impl {
	enum pin_toggle_cmd_t : uint32_t {
		nop = 0b000,
		off = 0b001,
		on = 0b010,
		once = 0b011,
		many = 0b100,
		continuous = 0b101
	};

	/*
	Notification value format (uint32_t)
	tm_high(8bit),tm_on_low(8bit),tm_off_low(8bit),count(5bit),command(3bit)
	*/
	inline
	std::pair<uint16_t, uint16_t> tm_decode(uint32_t nf)
	{
		const uint16_t tm_on = nf >> 16;
		const uint16_t tm_off = (tm_on & 0xff00) | ((nf >> 8) & 0xff);
		return std::make_pair(tm_on, tm_off);
	}

	inline
	uint32_t tm_encode(TickType_t tm_on, TickType_t tm_off)
	{
		tm_on = std::min<TickType_t>(tm_on, 0xffff);
		// TODO do our best to calculate suitable tm_off
		return (tm_on << 16) | ((tm_off & 0xff) << 8);
	}
}


template <typename Pin>
class pin_toggle_task_t {
public:
	pin_toggle_task_t(const char* task_name, const Pin&, UBaseType_t prio);

	void init_pin() const
	{
		pin_.set_state(0);
		pin_.set_mode_output_lowspeed_pushpull();
	}

	void on()
	{
		xTaskNotify(task_handle_, pin_toggle_task_impl::pin_toggle_cmd_t::on, eSetValueWithOverwrite);
	}

	void off()
	{
		xTaskNotify(task_handle_, pin_toggle_task_impl::pin_toggle_cmd_t::off, eSetValueWithOverwrite);
	}

	void pulse_once(TickType_t tm_on)
	{
		const uint32_t v = pin_toggle_task_impl::tm_encode(tm_on, 0) | pin_toggle_task_impl::pin_toggle_cmd_t::once;
		xTaskNotify(task_handle_, v, eSetValueWithOverwrite);
	}

	void pulse_many(TickType_t tm_on, TickType_t tm_off, uint8_t count)
	{
		count = std::min(count, uint8_t(0b00011111));
		const uint32_t v = pin_toggle_task_impl::tm_encode(tm_on, tm_off) | (count << 3) | pin_toggle_task_impl::pin_toggle_cmd_t::many;
		xTaskNotify(task_handle_, v, eSetValueWithOverwrite);
	}

	void pulse_continuous(TickType_t tm_on, TickType_t tm_off)
	{
		const uint32_t v = pin_toggle_task_impl::tm_encode(tm_on, tm_off)
			| pin_toggle_task_impl::pin_toggle_cmd_t::continuous;
		xTaskNotify(task_handle_, v, eSetValueWithOverwrite);
	}

private:
	const Pin pin_;

	task_stack_t<64> stack_;
	StaticTask_t task_buffer_;
	TaskHandle_t const task_handle_;

	static void task_function(void*);
};

template <typename Pin>
pin_toggle_task_t<Pin> make_pin_toggle_task(const char* tn, const Pin& pin, UBaseType_t prio)
{
	return pin_toggle_task_t<Pin>(tn, pin, prio);
}


template <typename Pin>
pin_toggle_task_t<Pin>::pin_toggle_task_t(
	const char* task_name,
	const Pin& pin,
	UBaseType_t prio
)
	: pin_(pin)
	, task_handle_(xTaskCreateStatic(
		&task_function,
		task_name,
		stack_.size(),
		reinterpret_cast<void*>(this),
		prio,
		stack_.data(),
		&task_buffer_
	))
{
}



namespace pinpoll_impl {
	constexpr uint8_t poll_history_mask = 0b1111;

	template <typename Pin>
	using pin_cb_t = void (*)(const Pin& pin, bool new_status);

	template <typename Pin>
	struct pin_info_t {
		const Pin pin;
		pin_cb_t<Pin> const cb;
		pin_info_t(const Pin& p, pin_cb_t<Pin> c) : pin(p), cb(c) {}

		// internal data for poll function
		uint8_t poll_history = 0xff;
		bool pin_status = true;
	};

	template <typename Pin>
	void task_function(pin_info_t<Pin>* pin_info_items, size_t size);
}


namespace pinpoll {

template <typename Pin>
pinpoll_impl::pin_info_t<Pin> make_pin_info(const Pin& pin, pinpoll_impl::pin_cb_t<Pin> cb)
{
	return pinpoll_impl::pin_info_t<Pin>(pin, cb);
}


template <typename Pin, size_t Size>
struct task_arg_t {
	std::array<pinpoll_impl::pin_info_t<Pin>, Size> pin_info;

	template <typename ...Args>
	task_arg_t(Args&&... args) : pin_info{std::forward<Args>(args)...} {}

	task_data_t<128> task_data;
};


template <typename Pin, size_t Size>
void task_function(void *arg)
{
	using arg_t = task_arg_t<Pin, Size>;
	arg_t* const task_arg = reinterpret_cast<arg_t*>(arg);
	return pinpoll_impl::task_function(task_arg->pin_info.data(), Size);
}


template <typename Pin, size_t Size>
void create_task(const char* task_name, UBaseType_t prio, task_arg_t<Pin, Size>* arg)
{
	arg->task_data.task_handle = xTaskCreateStatic(
		&task_function<Pin, Size>,
		task_name,
		arg->task_data.stack.size(),
		reinterpret_cast<task_arg_t<Pin,Size>*>(arg),
		prio,
		arg->task_data.stack.data(),
		&arg->task_data.task_buffer
	);
}


} // namespace pin_poll


template <typename Pin>
void pin_toggle_task_t<Pin>::task_function(void* arg)
{
	pin_toggle_task_t* const This = reinterpret_cast<pin_toggle_task_t*>(arg);
	uint32_t nf = 0;
	for(;;) {
		while (nf == 0) {
			xTaskNotifyWait(0, 0xffffffff, &nf, configTICK_RATE_HZ /*1sec*/);
		}
		const auto cmd = static_cast<pin_toggle_task_impl::pin_toggle_cmd_t>(nf & 0b111);
		switch (cmd) {
			case pin_toggle_task_impl::pin_toggle_cmd_t::nop: nf = 0; break;
			case pin_toggle_task_impl::pin_toggle_cmd_t::off: This->pin_.set_state(0); nf = 0; break;
			case pin_toggle_task_impl::pin_toggle_cmd_t::on: This->pin_.set_state(1); nf = 0; break;
			case pin_toggle_task_impl::pin_toggle_cmd_t::once:
				This->pin_.set_state(1);
				vTaskDelay(pin_toggle_task_impl::tm_decode(nf).first);
				This->pin_.set_state(0);
				nf = 0;
				break;
			case pin_toggle_task_impl::pin_toggle_cmd_t::many: {
				uint8_t count = (nf & 0xff) >> 3;
				const auto tm = pin_toggle_task_impl::tm_decode(nf);
				nf = 0;
				while (count--) {
					This->pin_.set_state(1);
					vTaskDelay(tm.first);
					This->pin_.set_state(0);
					vTaskDelay(tm.second);
					// TODO poll for new command?
				}
				break;
			}
			case pin_toggle_task_impl::pin_toggle_cmd_t::continuous: {
				const auto tm = pin_toggle_task_impl::tm_decode(nf);
				nf = 0;
				while (nf == 0) {
					This->pin_.set_state(1);
					vTaskDelay(tm.first);
					This->pin_.set_state(0);
					vTaskDelay(tm.second);
					xTaskNotifyWait(0, 0xffffffff, &nf, 0); // poll
				}
				break;
			}
			default: nf = 0; break; // unknown cmd
		}
	}
}


template <typename Pin>
void pinpoll_impl::task_function(pin_info_t<Pin>* const pin_info, size_t size)
{
	//logger.log_async("PINPOLL task started\r\n");

	constexpr auto poll_delay = configTICK_RATE_HZ/20;
	static_assert(poll_delay*20 == configTICK_RATE_HZ); // ensure no truncation

	for (size_t i=0; i<size; ++i) {
		pin_info[i].pin.set_mode_button();
	}

	//logger.log_async("PINPOLL start polling\r\n");
	for(;;) {
		for (size_t i=0; i<size; ++i) {
			auto& pi = pin_info[i];
			const auto& pin = pi.pin;
			const bool status_now = pin.get_state();
			pi.poll_history <<= 1;
			if (status_now) {
				pi.poll_history |= 1;
			}
			if (pi.pin_status==true && (pi.poll_history & poll_history_mask)==0) {
				// released
				pi.cb(pin, (pi.pin_status=false));
			} else if (pi.pin_status==false && (pi.poll_history & poll_history_mask)==poll_history_mask) {
				// pressed
				pi.cb(pin, (pi.pin_status=true));
			}
		}
		vTaskDelay(poll_delay);
	}
}

} // namespace

#endif

