#ifndef _gpr_lib_stm32_included_
#define _gpr_lib_stm32_included_

#include "cmsis_device.h"

#include <stdbool.h>
#include <stdint.h>

#include "logging.h"
extern usart_logger_t logger;

namespace {
	constexpr uint32_t mask1(int n) { return 1 << (n); }
	constexpr uint32_t mask2(int n) { return 0b11 << ((n) * 2); }
	constexpr uint32_t mask4(int n) { return 0b1111 << ((n) * 4); }
}


namespace stm32_lib {

namespace gpio {

#ifdef TARGET_STM32F103
enum class mode_t {
	input = 0b00,
	output_10mhz = 0b01,
	output_2mhz = 0b10,
	output_50mhz = 0b11
};

enum class cnf_t {
	input_analog = 0b00,
	input_float = 0b01,
	input_pupd = 0b10,
	input_reserved1 = 0b11,
	output_pushpull = 0b00,
	output_opendrain = 0b01,
	output_af_pushpull = 0b10,
	output_af_opendrain = 0b11
};
#else
enum class mode_t {
	input = 0b00,
	output = 0b01,
	af = 0b10,
	analog = 0b11
};

enum class otype_t {
	push_pull = 0,
	open_drain = 1
};

enum class speed_t {
	bits_00 = 0b00,
	bits_01 = 0b01,
	bits_10 = 0b10,
	bits_11 = 0b11
};

enum class pupd_t {
	no_pupd = 0b00,
	pu = 0b01,
	pd = 0b10
};

struct af_t {
	const uint8_t af_num;
	explicit constexpr af_t(uint8_t af) : af_num(af) {}
};
#endif

template <bool Invert>
struct pin_impl_t {
	uint32_t const gpio_base; // store addr to avoid "unsafe" type cast and keep it constexpr
	uint8_t const reg;
	constexpr pin_impl_t(uint32_t base_addr, uint8_t r) : gpio_base(base_addr), reg(r) {}

	constexpr GPIO_TypeDef* gpio() const { return reinterpret_cast<GPIO_TypeDef*>(gpio_base); }

	void set_state(bool s) const
	{
		uint32_t const mask = ((s ^ Invert) ? (1U << reg) : (1U << reg) << 16);
		gpio()->BSRR = mask;
	}

#ifdef TARGET_STM32F103
	void set(mode_t m, cnf_t c) const
	{
		const uint32_t reg_lo = ((reg < 8) ? reg : reg - 8);
		auto const cr = ((reg < 8) ? &gpio()->CRL : &gpio()->CRH);
		*cr =
			(*cr & ~mask4(reg_lo))
			| (
				((static_cast<uint32_t>(c) << 2) | static_cast<uint32_t>(m))
				<<
				(reg_lo * 4)
			);
	}
#else
	template <typename... Targs>
	void set(mode_t m, Targs... args) const
	{
		set_mode(m);
		set(args...);
	}

	template <typename... Targs>
	void set(otype_t ot, Targs... args) const
	{
		gpio()->OTYPER = (gpio()->OTYPER & ~mask1(reg)) | (static_cast<uint32_t>(ot) << reg);
		set(args...);
	}

	template <typename... Targs>
	void set(speed_t s, Targs... args) const
	{
		gpio()->OSPEEDR = (gpio()->OSPEEDR & ~mask2(reg)) | (static_cast<uint32_t>(s) << (reg * 2));
		set(args...);
	}

	template <typename... Targs>
	void set(pupd_t p, Targs... args) const
	{
		gpio()->PUPDR = (gpio()->PUPDR & ~mask2(reg)) | (static_cast<uint32_t>(p) << (reg * 2));
		set(args...);
	}

	template <typename... Targs>
	void set(af_t af, Targs... args) const
	{
		const auto reg_lo = (reg < 8) ? reg : (reg-8);
		auto const afr = &gpio()->AFR[ (reg<8) ? 0 : 1 ];
		*afr = (*afr & ~mask4(reg_lo)) | (af.af_num << (reg_lo * 4));
		set_mode(mode_t::af);
		set(args...);
	}

	void set_mode_output_lowspeed_pushpull() const
	{
		#ifdef TARGET_STM32F103
		set(mode_t::output_2mhz, cnf_t::output_pushpull);
		#else
		set(mode_t::output, otype_t::push_pull, speed_t::bits_00);
		#endif
	}

	void set_mode_button() const
	{
		set(mode_t::input, pupd_t::pu);
	}

	bool get_state() const
	{
		return gpio()->IDR & (1UL << reg);
	}

private:
	void set() const {} // terminate arguments recursion

	void set_mode(mode_t m) const
	{
		gpio()->MODER = (gpio()->MODER & ~mask2(reg)) | (static_cast<uint32_t>(m) << (reg * 2));
	}
#endif
};
using pin_t = pin_impl_t<false>;
using pin_inverted_t = pin_impl_t<true>;
using gpio_pin_t = pin_t; // compat


// compat
template <typename Pin>
void set_state(const Pin& pin, bool s)
{
	return pin.set_state(s);
}


inline
void set_mode_output_hispeed_pushpull(const gpio_pin_t& pin)
{
#ifdef TARGET_STM32F103
	#error
	//pin.set(mode_t::output_2mhz, cnf_t::output_pushpull);
#else
	pin.set(mode_t::output, otype_t::push_pull, speed_t::bits_11);
#endif
}


inline
#ifdef TARGET_STM32F103
void set_mode_af_lowspeed_pu(const gpio_pin_t& pin)
{
	// TODO pullup
	pin.set(mode_t::output_2mhz, cnf_t::output_af_pushpull);
}
#else
void set_mode_af_lowspeed_pu(const gpio_pin_t& pin, uint32_t af_num)
{
	pin.set(af_t(af_num), pupd_t::pu);
}
#endif


inline
#ifdef TARGET_STM32F103
void set_mode_af_lowspeed_pushpull(const gpio_pin_t& pin)
{
	pin.set(mode_t::output_2mhz, cnf_t::output_af_pushpull);
}
#else
void set_mode_af_lowspeed_pushpull(const gpio_pin_t& pin, uint32_t af_num)
{
	pin.set(af_t(af_num), otype_t::push_pull, speed_t::bits_00);
}
#endif

inline
void set_mode_output_analog(const gpio_pin_t& pin)
{
#ifdef TARGET_STM32F103
#error "TODO"
#else
	pin.set(mode_t::analog);
#endif
}


} // namespace gpio


namespace spi {

inline
#ifdef TARGET_STM32F103
void init_pins(
		const gpio::gpio_pin_t& mosi,
		const gpio::gpio_pin_t& miso,
		const gpio::gpio_pin_t& sck
	)
{
	using namespace gpio;
	mosi.set(mode_t::output_2mhz, cnf_t::output_af_pushpull);
	miso.set(mode_t::input, cnf_t::input_float);
	sck.set(mode_t::output_2mhz, cnf_t::output_af_pushpull);
}
#else
void init_pins(
		const gpio::gpio_pin_t& mosi, uint32_t af_mosi,
		const gpio::gpio_pin_t& miso, uint32_t af_miso,
		const gpio::gpio_pin_t& sck, uint32_t af_sck
	)
{
	using namespace gpio;
	mosi.set(af_t(af_mosi), otype_t::push_pull, pupd_t::pd, speed_t::bits_00);
	miso.set(af_t(af_miso), otype_t::push_pull, pupd_t::pd, speed_t::bits_00);
	sck.set(af_t(af_sck), otype_t::push_pull, pupd_t::pd, speed_t::bits_00);
}


inline
void init_pin_nss(const gpio::gpio_pin_t& pin)
{
	using namespace gpio;
	pin.set(mode_t::output, otype_t::push_pull, pupd_t::pu, speed_t::bits_00);
	set_state(pin, 1);
}

#endif


template <typename T>
void write(SPI_TypeDef* const spi, size_t const size, const T* const tx_buf, T* const rx_buf)
{
	static_assert(sizeof(T) == 1 || sizeof(T) == 2); // FIXME
#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
	auto const txdr = reinterpret_cast<volatile T*>(&spi->TXDR);
	auto const rxdr = reinterpret_cast<volatile T*>(&spi->RXDR);
	bool tx_started = false;
#else
	auto const txdr = reinterpret_cast<volatile T*>(&spi->DR);
	auto const rxdr = txdr;
#endif

	size_t tx_done = 0, rx_done = 0;
	constexpr size_t max_count = 1000000;
	size_t count_without_progress = 0; // count iterations without progress

	while (tx_done < size || rx_done < size) {
		bool progress = false; // did something in current iteration?

#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
		while (tx_done < size && (spi->SR & SPI_SR_TXP)) // fill tx buffer
#else
		if (tx_done < size && (spi->SR & SPI_SR_TXE)) // write one item to tx buffer
#endif
		{
			*txdr = (tx_buf ? tx_buf[tx_done] : 0);
			++tx_done;
			progress = true;
		}
#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
		if (!tx_started && tx_done>0) {
			// something was written to tx buffer, start transfer
			spi->CR1 |= SPI_CR1_CSTART_Msk;
			tx_started = true;
		}
#endif

#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
		while (rx_done < size && (spi->SR & SPI_SR_RXP))
#else
		if (rx_done < size && (spi->SR & SPI_SR_RXNE))
#endif
		{
			const T d = *rxdr;
			if (rx_buf) {
				rx_buf[rx_done] = d;
			}
			++rx_done;
			progress = true;
		}

		if (progress) {
			count_without_progress = 0; // done something, reset counter
		} else {
			++count_without_progress;
			if (count_without_progress >= max_count) {
				// TODO handle timeout
			}
		}
	}

#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
#else
	count_without_progress = 0;
	while(spi->SR & SPI_SR_BSY) {
		++count_without_progress;
		if (count_without_progress >= max_count) {
			// TODO handle timeout
		}
	}
#endif
}

template <typename T>
T write(SPI_TypeDef* const spi, T data)
{
	T rx_buf = 0;
	write(spi, 1, &data, &rx_buf);
	return rx_buf;
}


} // namespace spi


#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
namespace hsem {


template <size_t N>
struct hsem_t {
	static_assert(N >= HSEM_SEMID_MIN);
	static_assert(N <= HSEM_SEMID_MAX);
	bool fast_take() // return false (to match with HAL_OK) on successful lock
	{
		if (HSEM->RLR[N] == (HSEM_CR_COREID_CURRENT | HSEM_RLR_LOCK)) {
			return false; // taken successfully
		}
		return true;
	}

	void release()
	{
		HSEM->R[N] = HSEM_CR_COREID_CURRENT;
	}
};


} // namespace hsem
#endif

} // namespace stm32_lib


#endif

