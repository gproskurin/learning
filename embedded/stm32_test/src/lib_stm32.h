#include "cmsis_device.h"

#include <stdbool.h>
#include <stdint.h>


namespace {
	constexpr uint32_t mask1(int n) { return 1 << (n); }
	constexpr uint32_t mask2(int n) { return 0b11 << ((n) * 2); }
	constexpr uint32_t mask4(int n) { return 0b1111 << ((n) * 4); }
}


namespace stm32_lib {

namespace rcc {

#ifdef TARGET_STM32L152
void init_clock()
{
	// tune MSI speed
	RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE_Msk) | RCC_ICSCR_MSIRANGE_6;

#if 0
	// switch on HSI
	RCC->CR |= RCC_CR_HSION;
	while (! (RCC->CR & RCC_CR_HSIRDY) ) {}

	// select HSI as system clock
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SWS_Msk) | RCC_CFGR_SWS_HSI;

	// switch off MSI
#endif
}
#endif

} // namespace rcc


namespace gpio {

#ifndef TARGET_STM32F103
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
	const uint32_t af_num;
	explicit af_t(uint32_t af) : af_num(af) {}
};
#endif

struct gpio_pin_t {
	GPIO_TypeDef* gpio;
	uint32_t reg;
	gpio_pin_t(GPIO_TypeDef* g, uint32_t r) : gpio(g), reg(r) {}

#ifndef TARGET_STM32F103
	template <typename... Targs>
	void set(mode_t m, Targs... args) const
	{
		set_mode(m);
		set(args...);
	}

	template <typename... Targs>
	void set(otype_t ot, Targs... args) const
	{
		gpio->OTYPER = (gpio->OTYPER & ~mask1(reg)) | (static_cast<uint32_t>(ot) << reg);
		set(args...);
	}

	template <typename... Targs>
	void set(speed_t s, Targs... args) const
	{
		gpio->OSPEEDR = (gpio->OSPEEDR & ~mask2(reg)) | (static_cast<uint32_t>(s) << (reg * 2));
		set(args...);
	}

	template <typename... Targs>
	void set(pupd_t p, Targs... args) const
	{
		gpio->PUPDR = (gpio->PUPDR & ~mask2(reg)) | (static_cast<uint32_t>(p) << (reg * 2));
		set(args...);
	}

	template <typename... Targs>
	void set(af_t af, Targs... args) const
	{
		const auto reg_lo = (reg < 8) ? reg : (reg-8);
		auto const afr = &gpio->AFR[ (reg<8) ? 0 : 1 ];
		*afr = (*afr & ~mask4(reg_lo)) | (af.af_num << (reg_lo * 4));
		set_mode(mode_t::af);
		set(args...);
	}

	void set() const {} // terminate arguments recursion

private:
	void set_mode(mode_t m) const
	{
		gpio->MODER = (gpio->MODER & ~mask2(reg)) | (static_cast<uint32_t>(m) << (reg * 2));
	}
#endif
};

inline
void set_state(const gpio_pin_t& pin, bool high)
{
#ifdef TARGET_STM32F103
	high = !high; // FIXME better?
#endif
	uint32_t const mask = (high ? (1U << pin.reg) : (1U << pin.reg) << 16);
	pin.gpio->BSRR = mask;
}


#ifdef TARGET_STM32F103
namespace {

inline
void set_mode_cnf(const gpio_pin_t& pin, uint32_t mode, uint32_t cnf)
{
	const uint32_t reg_lo = ((pin.reg < 8) ? pin.reg : pin.reg - 8);
	const uint32_t bits = (cnf << 2) | mode;
	const uint32_t mask_value = bits << (reg_lo * 4);
	auto const cr = ((pin.reg < 8) ? &pin.gpio->CRL : &pin.gpio->CRH);
	*cr = (*cr & ~mask4(reg_lo)) | mask_value;
}

} // namespace
#endif


inline
void set_mode_output_lowspeed_pushpull(const gpio_pin_t& pin)
{
#ifdef TARGET_STM32F103
	constexpr uint32_t mode = 0b10; // output mode, max speed 2 MHz
	constexpr uint32_t cnf = 0b00; // output push-pull
	set_mode_cnf(pin, mode, cnf);
#else
	pin.set(mode_t::output, otype_t::push_pull, speed_t::bits_00);
#endif
}


inline
#ifdef TARGET_STM32F103
void set_mode_af_lowspeed_pu(const gpio_pin_t& pin)
{
	// TODO pullup
	constexpr uint32_t mode = 0b10; // output mode, max speed 2 MHz
	constexpr uint32_t cnf = 0b10; // af push-pull
	set_mode_cnf(pin, mode, cnf);
}
#else
void set_mode_af_lowspeed_pu(const gpio_pin_t& pin, uint32_t af_num)
{
	pin.set(af_t(af_num), pupd_t::pu);
}
#endif


#ifdef TARGET_STM32F103
inline
void set_mode_af_lowspeed_pushpull(const gpio_pin_t& pin)
{
	// TODO
}
#else
inline
void set_mode_af_lowspeed_pushpull(const gpio_pin_t& pin, uint32_t af_num)
{
	pin.set(af_t(af_num), otype_t::push_pull, speed_t::bits_00);
}
#endif

} // namespace gpio


namespace spi {

inline
#ifdef TARGET_STM32F103
void init_pins(
		const gpio::gpio_pin_t& mosi,
		const gpio::gpio_pin_t& miso,
		const gpio::gpio_pin_t& sck,
		const gpio::gpio_pin_t& ss
	)
{
	// TODO
}
#else
void init_pins(
		const gpio::gpio_pin_t& mosi, int af_mosi,
		const gpio::gpio_pin_t& miso, int af_miso,
		const gpio::gpio_pin_t& sck, int af_sck,
		const gpio::gpio_pin_t& ss, int af_ss
	)
{
	using namespace gpio;
	mosi.set(af_t(af_mosi), otype_t::push_pull, speed_t::bits_00);
	miso.set(af_t(af_miso), pupd_t::no_pupd);
	sck.set(af_t(af_sck), pupd_t::no_pupd);
	ss.set(af_t(af_ss), pupd_t::pu, speed_t::bits_00);
}
#endif


class spi_t {
	SPI_TypeDef* const spi_;
	const gpio::gpio_pin_t pin_nss_;
public:
	spi_t(SPI_TypeDef* spi, const gpio::gpio_pin_t& nss);
	uint16_t write16(uint16_t data);
};

inline
spi_t::spi_t(SPI_TypeDef* spi, const gpio::gpio_pin_t& nss)
	: spi_(spi)
	, pin_nss_(nss)
{
}

inline
uint16_t spi_t::write16(uint16_t data)
{
	gpio::set_state(pin_nss_, 0);
	// TODO wait a bit?
#ifdef TARGET_STM32H7A3
	volatile uint16_t* const tx = reinterpret_cast<volatile uint16_t*>(&spi_->TXDR);
	*tx = data;
	spi_->CR1 |= SPI_CR1_CSTART_Msk;
	volatile uint16_t* const rx = reinterpret_cast<volatile uint16_t*>(&spi_->RXDR);
	const uint16_t r = *rx;
#else
	while(! (spi_->SR & SPI_SR_TXE)) {}
	spi_->DR = data;
	while(! (spi_->SR & SPI_SR_RXNE)) {}
	const uint16_t r = spi_->DR;
#endif
	gpio::set_state(pin_nss_, 1);
	return r;
}


} // namespace spi

} // namespace stm32_lib


