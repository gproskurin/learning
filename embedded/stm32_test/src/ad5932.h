#ifndef _gpr_ad5932_h_included_
#define _gpr_ad5932_h_included_

#include "lib_stm32.h"


class ad5932_t {
	const stm32_lib::gpio::gpio_pin_t pin_mclk_;
	const stm32_lib::gpio::gpio_pin_t pin_ctrl_;
	stm32_lib::spi::spi_t spi_;
public:
	ad5932_t(
			const stm32_lib::gpio::gpio_pin_t& pin_mclk,
			const stm32_lib::gpio::gpio_pin_t& pin_ctrl,
			SPI_TypeDef* spi,
			const stm32_lib::gpio::gpio_pin_t& pin_nss
		)
		: pin_mclk_(pin_mclk)
		, pin_ctrl_(pin_ctrl)
		, spi_(spi, pin_nss)
	{}
	void start();
private:
	void ctrl_pulse();
};


#endif

