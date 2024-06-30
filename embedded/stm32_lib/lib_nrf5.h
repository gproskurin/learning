#ifndef _gpr_lib_nrf5_included_
#define _gpr_lib_nrf5_included_

#include "cmsis_device.h"

#if defined(TARGET_NRF52DK)
#include "nrf52_bitfields.h"
#endif

#include <stdbool.h>
#include <stdint.h>

namespace {
	constexpr uint32_t mask1(int n) { return 1 << (n); }
	constexpr uint32_t mask2(int n) { return 0b11 << ((n) * 2); }
	constexpr uint32_t mask4(int n) { return 0b1111 << ((n) * 4); }
}


namespace nrf5_lib {

namespace gpio {

enum class dir_t {
	input = 0,
	output = 1,
};

enum class input_buffer_t {
	connect = 0,
	disconnect = 1
};

enum class pull_t {
	no_pull = 0,
	pd = 1,
	pu = 3
};

enum class state_t {
	lo = 0,
	hi = 1
};


template <bool Invert>
struct pin_impl_t {
	uint32_t const gpio_base;
	uint8_t const reg;
	constexpr NRF_GPIO_Type* gpio() const { return reinterpret_cast<NRF_GPIO_Type*>(gpio_base); }

	constexpr pin_impl_t(uint32_t gb, uint8_t r) : gpio_base(gb), reg(r) {}

	void set_state(bool s) const
	{
		if (s ^ Invert) {
			gpio()->OUTSET = (1UL << reg);
		} else {
			gpio()->OUTCLR = (1UL << reg);
		}
	}

	template <typename... Targs>
	void set(dir_t d, Targs... args) const
	{
		auto const p = ((d == dir_t::input) ? &gpio()->DIRCLR : &gpio()-> DIRSET);
		*p = mask1(reg);
		set(args...);
	}

	template <typename... Targs>
	void set(input_buffer_t b, Targs... args) const
	{
		gpio()->PIN_CNF[reg] = (gpio()->PIN_CNF[reg] & ~0b10) | (static_cast<uint32_t>(b) << 1);
		set(args...);
	}

	template <typename... Targs>
	void set(pull_t p, Targs... args) const
	{
		gpio()->PIN_CNF[reg] = (gpio()->PIN_CNF[reg] & ~0b1100) | (static_cast<uint32_t>(p) << 2);
		set(args...);
	}

	template <typename... Targs>
	void set(state_t s, Targs... args) const
	{
		set_state(static_cast<bool>(s));
		set(args...);
	}

	void set_mode_output_lowspeed_pushpull() const
	{
		set(dir_t::output, pull_t::no_pull);
	}

	void set_mode_button() const
	{
		set(dir_t::input, input_buffer_t::connect, pull_t::pu);
	}

	bool get_state() const
	{
		return gpio()->IN & (1UL << reg);
	}

private:
	void set() const {} // terminate arguments recursion
};

using pin_t = pin_impl_t<false>;
using pin_inverted_t = pin_impl_t<true>;


} // gpio


namespace uart {


#ifdef TARGET_NRF52DK
void uart_init(NRF_UART_Type* uart, uint8_t pin_reg)
{
	uart->ENABLE = 0;
	uart->CONFIG = 0;
	uart->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud115200;
	uart->PSELTXD = pin_reg;
	uart->PSELRXD = 0xFFFFFFFF;
	uart->PSELCTS = 0xFFFFFFFF;
	uart->PSELRTS = 0xFFFFFFFF;
	uart->ENABLE = 4;
}


void uart_send(NRF_UART_Type* uart, const char* s)
{
	uart->TASKS_STARTTX = 1;

	// tx_byte
	uart->EVENTS_TXDRDY = 0;
	uart->TXD = *s;
	++s;

	while (*s) {
		while (! (uart->EVENTS_TXDRDY)) {}
		uart->EVENTS_TXDRDY = 0;
		uart->TXD = *s;
		++s;
	}

	while (! (uart->EVENTS_TXDRDY)) {}
	uart->TASKS_STOPTX = 1;
}


void uart_deinit(NRF_UART_Type* uart)
{
	uart->ENABLE = 0;
}
#endif


void uarte_init(NRF_UARTE_Type* uarte, uint8_t pin_reg)
{
	uarte->ENABLE = 0;
	uarte->CONFIG = 0;
	uarte->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud115200;
	uarte->PSEL.TXD = pin_reg;
	uarte->PSEL.RXD = 0xFFFFFFFF;
	uarte->PSEL.CTS = 0xFFFFFFFF;
	uarte->PSEL.RTS = 0xFFFFFFFF;
	uarte->ENABLE = 8;
}


} // namespace uart


} // namespace nrf5_lib


#endif

