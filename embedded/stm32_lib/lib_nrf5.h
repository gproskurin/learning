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

enum class pull_t {
	no_pull = 0,
	pd = 1,
	pu = 3
};

struct pin_t {
	uint8_t const reg;

	void set_state(bool s) const
	{
		if (s) {
			NRF_P0->OUTSET = (1UL << reg);
		} else {
			NRF_P0->OUTCLR = (1UL << reg);
		}
	}

	template <typename... Targs>
	void set(dir_t d, Targs... args) const
	{
		auto const p = ((d == input) ? &NRF_P0->DIRCLR : &NRF_P0-> DIRSET);
		*p = mask1(reg);
		set(args...);
	}

	template <typename... Targs>
	void set(pull_t p, Targs... args) const
	{
		NRF_P0[reg] = (NRF_P0[reg] & ~0b1100) | (p << 2);
		set(args...);
	}

private:
	void set() const {} // terminate arguments recursion
};


}

} // namespace nrf5_lib


#endif

