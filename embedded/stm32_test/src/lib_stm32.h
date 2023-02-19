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

inline
void set_state(GPIO_TypeDef* gpio, int reg, bool high)
{
#ifdef TARGET_STM32F103
	high = !high; // FIXME better?
#endif
	uint32_t const mask = (high ? (1U << reg) : (1U << reg) << 16);
	gpio->BSRR = mask;
}


#ifdef TARGET_STM32F103
namespace {

inline
void set_mode_cnf(GPIO_TypeDef* gpio, int reg, uint32_t mode, uint32_t cnf)
{
	const uint32_t reg_lo = ((reg < 8) ? reg : reg - 8);
	const uint32_t bits = (cnf << 2) | mode;
	const uint32_t mask_value = bits << (reg_lo * 4);
	auto const cr = ((reg < 8) ? &gpio->CRL : &gpio->CRH);
	*cr = (*cr & ~mask4(reg_lo)) | mask_value;
}

} // namespace
#endif


#ifndef TARGET_STM32F103
inline
void set_af(GPIO_TypeDef* gpio, int reg, int af_num)
{
	const auto reg_lo = (reg < 8) ? reg : (reg-8);
	auto const afr = &gpio->AFR[ (reg<8) ? 0 : 1 ];
	*afr = (*afr & ~mask4(reg_lo)) | (af_num << (reg_lo * 4));
}
#endif


inline
void set_mode_output_lowspeed_pushpull(GPIO_TypeDef* gpio, int reg)
{
#ifdef TARGET_STM32F103
	constexpr uint32_t mode = 0b10; // output mode, max speed 2 MHz
	constexpr uint32_t cnf = 0b00; // output push-pull
	set_mode_cnf(gpio, reg, mode, cnf);
#else
	gpio->MODER = (gpio->MODER & ~mask2(reg)) | (0b01 << (reg * 2)); // general-purpose output

	gpio->OTYPER &= ~mask1(reg); // 0 = push-pull
	gpio->OSPEEDR &= ~mask2(reg) ; // 0b00 = low speed
	gpio->PUPDR &= ~mask2(reg); // 0b00 = no pull up/down
#endif
}


inline
#ifdef TARGET_STM32F103
void set_mode_af_lowspeed_pu(GPIO_TypeDef* gpio, int reg)
{
	constexpr uint32_t mode = 0b10; // output mode, max speed 2 MHz
	constexpr uint32_t cnf = 0b10; // af push-pull
	set_mode_cnf(gpio, reg, mode, cnf);
}
#else
void set_mode_af_lowspeed_pu(GPIO_TypeDef* gpio, int reg, int af_num)
{
	gpio->MODER = (gpio->MODER & ~mask2(reg)) | (0b10 << (reg * 2)); // alternate function
	gpio->PUPDR = (gpio->PUPDR & ~mask2(reg)) | (0b01 << (reg * 2)); // pull-up
	set_af(gpio, reg, af_num);
}
#endif

inline
#ifdef TARGET_STM32F103
TODO
void set_mode_af_hispeed_pushpull(GPIO_TypeDef* gpio, int reg)
{
	constexpr uint32_t mode = 0b10; // output mode, max speed 2 MHz
	constexpr uint32_t cnf = 0b10; // af push-pull
	set_mode_cnf(gpio, reg, mode, cnf);
}
#else
void set_mode_af_hispeed_pushpull(GPIO_TypeDef* gpio, int reg, int af_num)
{
	gpio->MODER = (gpio->MODER & ~mask2(reg)) | (0b10 << (reg * 2)); // alternate function
	gpio->OTYPER = (gpio->OTYPER & ~mask1(reg)) | (0b0 << reg); // output push-pull
	gpio->PUPDR = (gpio->PUPDR & ~mask2(reg)) | (0b00 << (reg * 2)); // no pupd
	gpio->OSPEEDR = (gpio->OSPEEDR & ~mask2(reg)) | (0b11 << (reg * 2)); // very high speed
	set_af(gpio, reg, af_num);
}
#endif


#ifndef TARGET_STM32F103
namespace {
void set_mode_af_hispeed_pushpull(GPIO_TypeDef* gpio, int reg, int af_num, uint32_t pupd)
{
	gpio->MODER = (gpio->MODER & ~mask2(reg)) | (0b10 << (reg * 2)); // alternate function
	gpio->OTYPER = (gpio->OTYPER & ~mask1(reg)) | (0b0 << reg); // output push-pull
	gpio->PUPDR = (gpio->PUPDR & ~mask2(reg)) | (pupd << (reg * 2));
	gpio->OSPEEDR = (gpio->OSPEEDR & ~mask2(reg)) | (0b11 << (reg * 2)); // very high speed
	set_af(gpio, reg, af_num);
}
}

void set_mode_af_hispeed_pushpull_pullup(GPIO_TypeDef* gpio, int reg, int af_num)
{
	set_mode_af_hispeed_pushpull(gpio, reg, af_num, 0b01);
}

void set_mode_af_hispeed_pushpull_float(GPIO_TypeDef* gpio, int reg, int af_num)
{
	set_mode_af_hispeed_pushpull(gpio, reg, af_num, 0b00);
}
#endif

} // namespace gpio


namespace spi {

void init_pins(
		GPIO_TypeDef* gpio_mosi, int pin_mosi, int af_mosi,
		GPIO_TypeDef* gpio_miso, int pin_miso, int af_miso,
		GPIO_TypeDef* gpio_sck, int pin_sck, int af_sck,
		GPIO_TypeDef* gpio_ss, int pin_ss, int af_ss
	)
{
	gpio::set_mode_af_hispeed_pushpull_float(gpio_mosi, pin_mosi, af_mosi);
	gpio::set_mode_af_hispeed_pushpull_pullup(gpio_miso, pin_miso, af_miso);
	gpio::set_mode_af_hispeed_pushpull_float(gpio_sck, pin_sck, af_sck);
	gpio::set_mode_af_hispeed_pushpull_pullup(gpio_ss, pin_ss, af_ss);
}


class spi_t {
	SPI_TypeDef* const spi_;
	GPIO_TypeDef* const nss_gpio_;
	int const nss_pin_;
public:
	spi_t(SPI_TypeDef* spi, GPIO_TypeDef* nss_gpio, int nss_pin);
	uint16_t write16(uint16_t data);
};

inline
spi_t::spi_t(SPI_TypeDef* spi, GPIO_TypeDef* nss_gpio, int nss_pin)
	: spi_(spi)
	, nss_gpio_(nss_gpio)
	, nss_pin_(nss_pin)
{
}

inline
uint16_t spi_t::write16(uint16_t data)
{
	gpio::set_state(nss_gpio_, nss_pin_, 0);
	// TODO wait a bit?
#ifdef TARGET_STM32H7A3
	volatile uint16_t* const tx = reinterpret_cast<volatile uint16_t*>(&spi->TXDR);
	*tx = data;
	spi->CR1 |= SPI_CR1_CSTART_Msk;
	volatile uint16_t* const rx = reinterpret_cast<volatile uint16_t*>(&spi->RXDR);
	const uint16_t r = *rx;
#else
	while(! (spi_->SR & SPI_SR_TXE)) {}
	spi_->DR = data;
	while(! (spi_->SR & SPI_SR_RXNE)) {}
	const uint16_t r = spi_->DR;
#endif
	gpio::set_state(nss_gpio_, nss_pin_, 1);
	return r;
}


} // namespace spi

} // namespace stm32_lib


