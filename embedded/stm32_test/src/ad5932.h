#ifndef _gpr_ad5932_h_included_
#define _gpr_ad5932_h_included_

#include "lib_stm32.h"


class ad5932_t {
	stm32_lib::gpio::gpio_pin_t pin_mclk_;
	stm32_lib::spi::spi_t spi_;
public:
	ad5932_t(
			const stm32_lib::gpio::gpio_pin_t& pin_mclk,
			SPI_TypeDef* spi,
			const stm32_lib::gpio::gpio_pin_t& pin_nss
		)
		: pin_mclk_(pin_mclk)
		, spi_(spi, pin_nss)
	{}
	void start();
private:
	void ctrl_pulse();
};


#endif

