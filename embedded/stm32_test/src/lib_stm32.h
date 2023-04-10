#ifndef _gpr_lib_stm32_included_
#define _gpr_lib_stm32_included_

#include "cmsis_device.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "logging.h"
extern usart_logger_t logger;

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
	const uint32_t af_num;
	explicit af_t(uint32_t af) : af_num(af) {}
};
#endif

struct gpio_pin_t {
	GPIO_TypeDef* const gpio;
	uint32_t const reg;
	gpio_pin_t(GPIO_TypeDef* g, uint32_t r) : gpio(g), reg(r) {}

#ifdef TARGET_STM32F103
	void set(mode_t m, cnf_t c) const
	{
		const uint32_t reg_lo = ((reg < 8) ? reg : reg - 8);
		auto const cr = ((reg < 8) ? &gpio->CRL : &gpio->CRH);
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

private:
	void set() const {} // terminate arguments recursion

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


inline
void set_mode_output_lowspeed_pushpull(const gpio_pin_t& pin)
{
#ifdef TARGET_STM32F103
	pin.set(mode_t::output_2mhz, cnf_t::output_pushpull);
#else
	pin.set(mode_t::output, otype_t::push_pull, speed_t::bits_00);
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
	using namespace gpio;
	mosi.set(mode_t::output_2mhz, cnf_t::output_af_pushpull);
	miso.set(mode_t::input, cnf_t::input_float);
	sck.set(mode_t::output_2mhz, cnf_t::output_af_pushpull);
	ss.set(mode_t::output_2mhz, cnf_t::output_pushpull);
}
#else
void init_pins(
		const gpio::gpio_pin_t& mosi, uint32_t af_mosi,
		const gpio::gpio_pin_t& miso, uint32_t af_miso,
		const gpio::gpio_pin_t& sck, uint32_t af_sck,
		const gpio::gpio_pin_t& ss, uint32_t af_ss
	)
{
	using namespace gpio;
	mosi.set(af_t(af_mosi), otype_t::push_pull, pupd_t::no_pupd, speed_t::bits_11);
	//miso.set(af_t(af_miso), pupd_t::no_pupd, speed_t::bits_00);
	sck.set(af_t(af_sck), pupd_t::no_pupd, speed_t::bits_11);
	//ss.set(af_t(af_ss), pupd_t::pu, speed_t::bits_00);
	ss.set(mode_t::output, otype_t::push_pull, pupd_t::pu, speed_t::bits_11);
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
	//vTaskDelay(1);
	for (volatile int i=0; i<10; ++i) {}
	// TODO wait a bit?
#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7
	volatile uint16_t* const tx = reinterpret_cast<volatile uint16_t*>(&spi_->TXDR);
	*tx = data;
	spi_->CR1 |= SPI_CR1_CSTART_Msk;
	volatile uint16_t* const rx = reinterpret_cast<volatile uint16_t*>(&spi_->RXDR);
	const uint16_t r = *rx;
#else
	while(! (spi_->SR & SPI_SR_TXE)) {}
	spi_->DR = data;
	// FIXME need all this?
	//while(! (spi_->SR & SPI_SR_TXE)) {}
	//while(spi_->SR & SPI_SR_BSY) {}
	while(! (spi_->SR & SPI_SR_RXNE)) {}
	const uint16_t r = spi_->DR;
#endif
	gpio::set_state(pin_nss_, 1);
	return r;
}


} // namespace spi

} // namespace stm32_lib


#endif

