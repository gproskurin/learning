#include "cmsis_device.h"

#include <stdbool.h>
#include <stdint.h>


namespace stm32_lib {
namespace gpio {

namespace {
	constexpr uint32_t mask1(int n) { return 1 << (n); }
	constexpr uint32_t mask2(int n) { return 0b11 << ((n) * 2); }
	constexpr uint32_t mask4(int n) { return 0b1111 << ((n) * 4); }
}


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


inline
void set_mode_output_lowspeed_pushpull(GPIO_TypeDef* gpio, int reg)
{
#ifdef TARGET_STM32F103
	constexpr uint32_t mode = 0b10; // output mode, max speed 2 MHz
	constexpr uint32_t cnf = 0b00; // output push-pull
	set_mode_cnf(gpio, reg, mode, cnf);
#else
	const uint32_t mask_value = 0b01 << (reg * 2); // general-purpose output
	gpio->MODER = (gpio->MODER & ~mask2(reg)) | mask_value;

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
#else
void set_mode_af_lowspeed_pu(GPIO_TypeDef* gpio, int reg, int af_num)
{
	// MODER
	const uint32_t mask_value = 0b10 << (reg * 2); // alternate function
	gpio->MODER = (gpio->MODER & ~mask2(reg)) | mask_value;

	// PUPD
	const uint32_t pupd_value = 0b01 << (reg * 2); // pull-up
	gpio->PUPDR = (gpio->PUPDR & ~mask2(reg)) | pupd_value;

	// AF
	const auto reg_lo = (reg < 8) ? reg : (reg-8);
	const uint32_t af_mask_value = af_num << (reg_lo * 4);
	auto const afr = &gpio->AFR[ (reg<8) ? 0 : 1 ];
	*afr = (*afr & ~mask4(reg_lo)) | af_mask_value;
#endif
}


}
}

