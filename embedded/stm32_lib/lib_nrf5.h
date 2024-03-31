#ifndef _gpr_lib_nrf5_included_
#define _gpr_lib_nrf5_included_

#include "cmsis_device.h"

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


#ifdef TARGET_NRF52DK
#define MY_P0 (NRF_P0)
#elif defined TARGET_NRF5340DK_APP
#define MY_P0 (NRF_P0_S)
#endif

template <bool Invert>
struct pin_impl_t {
	uint8_t const reg;

	explicit constexpr pin_impl_t(uint8_t r) : reg(r) {}

	void set_state(bool s) const
	{
		if (s ^ Invert) {
			MY_P0->OUTSET = (1UL << reg);
		} else {
			MY_P0->OUTCLR = (1UL << reg);
		}
	}

	template <typename... Targs>
	void set(dir_t d, Targs... args) const
	{
		auto const p = ((d == dir_t::input) ? &MY_P0->DIRCLR : &MY_P0-> DIRSET);
		*p = mask1(reg);
		set(args...);
	}

	template <typename... Targs>
	void set(input_buffer_t b, Targs... args) const
	{
		MY_P0->PIN_CNF[reg] = (MY_P0->PIN_CNF[reg] & ~0b10) | (static_cast<uint32_t>(b) << 1);
		set(args...);
	}

	template <typename... Targs>
	void set(pull_t p, Targs... args) const
	{
		MY_P0->PIN_CNF[reg] = (MY_P0->PIN_CNF[reg] & ~0b1100) | (static_cast<uint32_t>(p) << 2);
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
		return MY_P0->IN & (1UL << reg);
	}

private:
	void set() const {} // terminate arguments recursion
};

using pin_t = pin_impl_t<false>;
using pin_inverted_t = pin_impl_t<true>;


} // gpio


} // namespace nrf5_lib


#endif

