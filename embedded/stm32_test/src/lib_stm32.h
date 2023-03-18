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

struct gpio_pin_t {
	GPIO_TypeDef* gpio;
	uint32_t reg;
	gpio_pin_t(GPIO_TypeDef* g, uint32_t r) : gpio(g), reg(r) {}
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


#ifndef TARGET_STM32F103
inline
void set_af(const gpio_pin_t& pin, uint32_t af_num)
{
	const auto reg_lo = (pin.reg < 8) ? pin.reg : (pin.reg-8);
	auto const afr = &pin.gpio->AFR[ (pin.reg<8) ? 0 : 1 ];
	*afr = (*afr & ~mask4(reg_lo)) | (af_num << (reg_lo * 4));
}
#endif


inline
void set_mode_output_lowspeed_pushpull(const gpio_pin_t& pin)
{
#ifdef TARGET_STM32F103
	constexpr uint32_t mode = 0b10; // output mode, max speed 2 MHz
	constexpr uint32_t cnf = 0b00; // output push-pull
	set_mode_cnf(pin, mode, cnf);
#else
	pin.gpio->MODER = (pin.gpio->MODER & ~mask2(pin.reg)) | (0b01 << (pin.reg * 2)); // general-purpose output

	pin.gpio->OTYPER &= ~mask1(pin.reg); // 0 = push-pull
	pin.gpio->OSPEEDR &= ~mask2(pin.reg) ; // 0b00 = low speed
	pin.gpio->PUPDR &= ~mask2(pin.reg); // 0b00 = no pull up/down
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
void set_mode_af_lowspeed_pu(const gpio_pin_t& pin, int af_num)
{
	pin.gpio->MODER = (pin.gpio->MODER & ~mask2(pin.reg)) | (0b10 << (pin.reg * 2)); // alternate function
	pin.gpio->PUPDR = (pin.gpio->PUPDR & ~mask2(pin.reg)) | (0b01 << (pin.reg * 2)); // pull-up
	set_af(pin, af_num);
}
#endif

inline
#ifdef TARGET_STM32F103
void set_mode_af_hispeed_pushpull(const gpio_pin_t& pin)
{
	constexpr uint32_t mode = 0b11; // output mode, max speed 50 MHz
	constexpr uint32_t cnf = 0b10; // af push-pull
	set_mode_cnf(pin, mode, cnf);
}
#else
void set_mode_af_hispeed_pushpull(const gpio_pin_t& pin, int af_num)
{
	pin.gpio->MODER = (pin.gpio->MODER & ~mask2(pin.reg)) | (0b10 << (pin.reg * 2)); // alternate function
	pin.gpio->OTYPER = (pin.gpio->OTYPER & ~mask1(pin.reg)) | (0b0 << pin.reg); // output push-pull
	pin.gpio->PUPDR = (pin.gpio->PUPDR & ~mask2(pin.reg)) | (0b00 << (pin.reg * 2)); // no pupd
	pin.gpio->OSPEEDR = (pin.gpio->OSPEEDR & ~mask2(pin.reg)) | (0b11 << (pin.reg * 2)); // very high speed
	set_af(pin, af_num);
}
#endif


#ifdef TARGET_STM32F103
inline
void set_mode_af_hispeed_pushpull_pullup(const gpio_pin_t& pin)
{
	// TODO
	set_mode_af_hispeed_pushpull(pin);
}

inline
void set_mode_af_hispeed_pushpull_float(const gpio_pin_t& pin)
{
	// TODO
	set_mode_af_hispeed_pushpull(pin);
}
#else
namespace {
void set_mode_af_hispeed_pushpull(const gpio_pin_t& pin, int af_num, uint32_t pupd)
{
	pin.gpio->MODER = (pin.gpio->MODER & ~mask2(pin.reg)) | (0b10 << (pin.reg * 2)); // alternate function
	pin.gpio->OTYPER = (pin.gpio->OTYPER & ~mask1(pin.reg)) | (0b0 << pin.reg); // output push-pull
	pin.gpio->PUPDR = (pin.gpio->PUPDR & ~mask2(pin.reg)) | (pupd << (pin.reg * 2));
	pin.gpio->OSPEEDR = (pin.gpio->OSPEEDR & ~mask2(pin.reg)) | (0b11 << (pin.reg * 2)); // very high speed
	set_af(pin, af_num);
}
}

inline
void set_mode_af_hispeed_pushpull_pullup(const gpio_pin_t& pin, int af_num)
{
	set_mode_af_hispeed_pushpull(pin, af_num, 0b01);
}

inline
void set_mode_af_hispeed_pushpull_float(const gpio_pin_t& pin, int af_num)
{
	set_mode_af_hispeed_pushpull(pin, af_num, 0b00);
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
	gpio::set_mode_af_hispeed_pushpull_float(mosi);
	gpio::set_mode_af_hispeed_pushpull_pullup(miso);
	gpio::set_mode_af_hispeed_pushpull_float(sck);
	gpio::set_mode_af_hispeed_pushpull_pullup(ss);
}
#else
void init_pins(
		const gpio::gpio_pin_t& mosi, int af_mosi,
		const gpio::gpio_pin_t& miso, int af_miso,
		const gpio::gpio_pin_t& sck, int af_sck,
		const gpio::gpio_pin_t& ss, int af_ss
	)
{
	gpio::set_mode_af_hispeed_pushpull_float(mosi, af_mosi);
	gpio::set_mode_af_hispeed_pushpull_pullup(miso, af_miso);
	gpio::set_mode_af_hispeed_pushpull_float(sck, af_sck);
	gpio::set_mode_af_hispeed_pushpull_pullup(ss, af_ss);
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


