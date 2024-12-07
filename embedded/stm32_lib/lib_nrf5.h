#ifndef _gpr_lib_nrf5_included_
#define _gpr_lib_nrf5_included_

#include "cmsis_device.h"

#if defined(TARGET_NRF52DK)
#include "nrf52_bitfields.h"
#include "nrf52832_xxaa_memory.h"
#elif defined(TARGET_NRF5340DK_APP)
#include "nrf5340_application_bitfields.h"
#include "nrf5340_xxaa_application_memory.h"
#endif


#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <array>


namespace nrf5_lib {


#if defined(TARGET_NRF52DK)
inline
bool can_dma(const void* ptr)
{
	auto const addr = reinterpret_cast<uint32_t>(ptr);
	return (addr >= NRF_MEMORY_RAM_BASE) && (addr < (NRF_MEMORY_RAM_BASE + NRF_MEMORY_RAM_SIZE));
}
#endif
#if defined(TARGET_NRF5340DK_APP)
inline
bool can_dma(const void* ptr)
{
	auto const addr = reinterpret_cast<uint32_t>(ptr);
	return (addr >= NRF_MEMORY_RAM0_BASE) && (addr < (NRF_MEMORY_RAM0_BASE + NRF_MEMORY_RAM0_SIZE));
}
#endif


inline
size_t chunk0(void* dst0, const void* src0, size_t max_size)
{
	auto const dst = reinterpret_cast<char*>(dst0);
	auto const src = reinterpret_cast<const char*>(src0);
	size_t size = 0;
	while (size < max_size) {
		auto const c = src[size];
		if (c == 0) {
			break;
		}
		dst[size] = c;
		++size;
	}
	return size;
}


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
		*p = 1 << reg;
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


template <typename Pin>
void uarte_init(NRF_UARTE_Type* uarte, const Pin& pin)
{
	uarte->ENABLE = 0;
	uarte->CONFIG = 0;
	uarte->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud115200;
#ifdef TARGET_NRF52DK
	uarte->PSEL.TXD = pin.reg;
#else
	uarte->PSEL.TXD = (((pin.gpio_base == NRF_P0_S_BASE) ? 0 : 1) << 5) | pin.reg; // FIXME
#endif
	uarte->PSEL.RXD = 0xFFFFFFFF;
	uarte->PSEL.CTS = 0xFFFFFFFF;
	uarte->PSEL.RTS = 0xFFFFFFFF;
	uarte->ENABLE = 8;
}

namespace impl {
	inline
	void uarte_send_start(NRF_UARTE_Type* uarte, const void* ptr, size_t size)
	{
		uarte->EVENTS_ENDTX = 0;
		uarte->TXD.MAXCNT = size;
		uarte->TXD.PTR = reinterpret_cast<uint32_t>(ptr);
		uarte->TASKS_STARTTX = 1;
	}

	inline
	void uarte_send_wait(NRF_UARTE_Type* uarte)
	{
		while (! (uarte->EVENTS_ENDTX)) {}
	}
}

inline
void uarte_send(NRF_UARTE_Type* uarte, const char* s)
{
	std::array<char, 8> buf;
	for (;;) {
		auto const chunk_size = nrf5_lib::chunk0(buf.data(), s, buf.size());
		if (chunk_size == 0) {
			break;
		}

		impl::uarte_send_start(uarte, buf.data(), chunk_size);
		s += chunk_size;
		impl::uarte_send_wait(uarte);
	}
}


} // namespace uart


} // namespace nrf5_lib


#endif

