#if defined TARGET_STM32L152
#include <cmsis_device_l1/Include/stm32l152xe.h>
#elif defined TARGET_STM32F103
#include <cmsis_device_f1/Include/stm32f103xb.h>
#else
#error "Unsupported target"
#endif

#include <optional>
#include <stdint.h>


namespace stm32_lib {
namespace gpio {

namespace {
	inline uint32_t mask1(int n) { return 1 << (n); }
	inline uint32_t mask2(int n) { return 0b11 << ((n) * 2); }
	inline uint32_t mask4(int n) { return 0b1111 << ((n) * 4); }
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


inline
void set_mode_output_lowspeed_pushpull(GPIO_TypeDef* gpio, int reg)
{
#ifdef TARGET_STM32F103
	constexpr uint32_t mode = 0b10; // output mode, max speed 2 MHz
	constexpr uint32_t cnf = 0b00; // output push-pull
	const uint32_t reg_lo = ((reg < 8) ? reg : reg - 8);
	constexpr uint32_t bits = (cnf << 2) | mode;
	const uint32_t mask_value = bits << (reg_lo * 4);
	auto const cr = ((reg < 8) ? &gpio->CRL : &gpio->CRH);
	*cr = (*cr & ~mask4(reg_lo)) | mask_value;
#elif defined TARGET_STM32L152
	const uint32_t mask_value = 0b01 << (reg * 2); // general-purpose output
	gpio->MODER = (gpio->MODER & ~mask2(reg)) | mask_value;

	gpio->OTYPER &= ~mask1(reg); // 0 = push-pull
	gpio->OSPEEDR &= ~mask2(reg) ; // 0b00 = low speed
	gpio->PUPDR &= ~mask2(reg); // 0b00 = no pull up/down
#endif
}


inline
void set_mode_af_lowspeed_pu(GPIO_TypeDef* gpio, int reg, int af_num)
{
#ifdef TARGET_STM32F103
#elif defined TARGET_STM32L152
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

