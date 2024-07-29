#ifndef _gpr_lib_rp2040_included_
#define _gpr_lib_rp2040_included_

#include "hardware/structs/iobank0.h"
#include "hardware/structs/sio.h"

#include <stdbool.h>
#include <stdint.h>


namespace rp2040_lib {


namespace gpio {


enum class mode_t {
	input,
	output
};

struct af_t {
	const uint8_t af_num;
	explicit constexpr af_t(uint8_t af) : af_num(af) {}
};


template <bool Invert>
struct pin_impl_t {
	uint8_t const reg;

	explicit constexpr pin_impl_t(uint8_t r) : reg(r) {}

	void set_state(bool s) const
	{
		((s ^ Invert) ? sio_hw->gpio_set : sio_hw->gpio_clr) = 1 << reg;
	}

	template <typename... Targs>
	void set(mode_t m, Targs... args) const
	{
		switch (m) {
			case mode_t::input:
				break;
			case mode_t::output:
				sio_hw->gpio_oe_set = 1 << reg;
				break;
		}
		set(args...);
	}

	template <typename... Targs>
	void set(af_t af, Targs... args) const
	{
		iobank0_hw->io[reg].ctrl = af.af_num;
		set(args...);
	}

#if 0
	bool get_state() const
	{
		// TODO
		return true;
	}
	void set_mode_output_lowspeed_pushpull() const
	{
		// TODO
	}
#endif

private:
	void set() const {} // terminate arguments recursion
};

using pin_t = pin_impl_t<false>;
using pin_inverted_t = pin_impl_t<true>;


} // gpio


} // namespace rp2040_lib


#endif

