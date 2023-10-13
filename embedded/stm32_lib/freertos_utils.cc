#include "freertos_utils.h"

#include <stdint.h>
#include <algorithm>
#include <utility>


namespace freertos_utils {

/*
Notification value format (uint32_t)
tm_on_high(8bit),tm_on_low(8bit),tm_off_low(8bit),count(5bit),command(3bit)
*/

enum pin_toggle_cmd_t : uint32_t {
	nop = 0b000,
	off = 0b001,
	on = 0b010,
	once = 0b011,
	many = 0b100,
	continuous = 0b101
};


std::pair<uint16_t, uint16_t> tm_decode(uint32_t nf)
{
	const uint16_t tm_on = nf >> 16;
	const uint16_t tm_off = (tm_on & 0x00) | ((nf >> 8) & 0xff);
	return std::make_pair(tm_on, tm_off);
}

uint32_t tm_encode(TickType_t tm_on, TickType_t tm_off)
{
	tm_on = std::min<TickType_t>(tm_on, 0xffff);
	// TODO do our best to calculate suitable tm_off
	return (tm_on << 16) | ((tm_off & 0xff) << 8);
}


void pin_toggle_task_t::off()
{
	xTaskNotify(task_handle_, pin_toggle_cmd_t::off, eSetValueWithOverwrite);
}

void pin_toggle_task_t::on()
{
	xTaskNotify(task_handle_, pin_toggle_cmd_t::on, eSetValueWithOverwrite);
}

void pin_toggle_task_t::pulse_once(TickType_t tm_on)
{
	const uint32_t v = tm_encode(tm_on, 0) | pin_toggle_cmd_t::once;
	xTaskNotify(task_handle_, v, eSetValueWithOverwrite);
}

void pin_toggle_task_t::pulse_many(TickType_t tm_on, TickType_t tm_off, uint8_t count)
{
	count = std::min(count, uint8_t(0b00011111));
	const uint32_t v = tm_encode(tm_on, tm_off) | (count << 3) | pin_toggle_cmd_t::many;
	xTaskNotify(task_handle_, v, eSetValueWithOverwrite);
}

void pin_toggle_task_t::pulse_continuous(TickType_t tm_on, TickType_t tm_off)
{
	const uint32_t v = tm_encode(tm_on, tm_off) | pin_toggle_cmd_t::continuous;
	xTaskNotify(task_handle_, v, eSetValueWithOverwrite);
}


void pin_toggle_task_t::task_function(void* arg)
{
	pin_toggle_task_t* const This = reinterpret_cast<pin_toggle_task_t*>(arg);
	uint32_t nf = 0;
	for(;;) {
		while (nf == 0) {
			xTaskNotifyWait(0, 0xffffffff, &nf, configTICK_RATE_HZ /*1sec*/);
		}
		const auto cmd = static_cast<pin_toggle_cmd_t>(nf & 0b111);
		switch (cmd) {
			case pin_toggle_cmd_t::nop: nf = 0; break;
			case pin_toggle_cmd_t::off: stm32_lib::gpio::set_state(This->pin_, 0); nf = 0; break;
			case pin_toggle_cmd_t::on: stm32_lib::gpio::set_state(This->pin_, 1); nf = 0; break;
			case pin_toggle_cmd_t::once:
				stm32_lib::gpio::set_state(This->pin_, 1);
				vTaskDelay(tm_decode(nf).first);
				stm32_lib::gpio::set_state(This->pin_, 0);
				nf = 0;
				break;
			case pin_toggle_cmd_t::many: {
				uint8_t count = (nf & 0xff) >> 3;
				const auto tm = tm_decode(nf);
				nf = 0;
				while (count--) {
					stm32_lib::gpio::set_state(This->pin_, 1);
					vTaskDelay(tm.first);
					stm32_lib::gpio::set_state(This->pin_, 0);
					vTaskDelay(tm.second);
					// TODO poll for new command?
				}
				break;
			}
			case continuous: {
				const auto tm = tm_decode(nf);
				nf = 0;
				while (nf == 0) {
					stm32_lib::gpio::set_state(This->pin_, 1);
					vTaskDelay(tm.first);
					stm32_lib::gpio::set_state(This->pin_, 0);
					vTaskDelay(tm.second);
					xTaskNotifyWait(0, 0xffffffff, &nf, 0); // poll
				}
				break;
			}
			default: nf = 0; break; // unknown cmd
		}
	}
}


pin_toggle_task_t::pin_toggle_task_t(
	const char* task_name,
	const stm32_lib::gpio::gpio_pin_t& pin,
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


void pin_toggle_task_t::init_pin() const
{
	stm32_lib::gpio::set_mode_output_lowspeed_pushpull(pin_);
}


void pinpoll::impl::task_function(pin_info_t* const pin_info, size_t size)
{
	logger.log_async("PINPOLL task started\r\n");

	constexpr auto poll_delay = configTICK_RATE_HZ/20;
	static_assert(poll_delay*20 == configTICK_RATE_HZ); // ensure no truncation

	for (size_t i=0; i<size; ++i) {
		stm32_lib::gpio::set_mode_button(pin_info[i].pin);
	}

	logger.log_async("PINPOLL start polling\r\n");
	for(;;) {
		for (size_t i=0; i<size; ++i) {
			auto& pi = pin_info[i];
			const auto& pin = pi.pin;
			const auto idr = pin.gpio()->IDR;
			const bool status_now = idr & (1 << pin.reg);
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

