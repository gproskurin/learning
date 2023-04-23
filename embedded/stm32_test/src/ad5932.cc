#include "ad5932_defs.h"
#include "ad5932.h"

#include "FreeRTOS.h"
#include "task.h"

#include "FreeRTOSConfig.h"

#include "logging.h"
extern usart_logger_t logger;

void ad5932_t::start()
{
	// configure
	spi_.write16(
		ad5932_defs_t::REG_Creg
		| ad5932_defs_t::REG_Creg_B24
		| ad5932_defs_t::REG_Creg_DAC_ENABLE
		| ad5932_defs_t::REG_Creg_SINE_TRI
		| ad5932_defs_t::REG_Creg_MSBOUTEN
		| ad5932_defs_t::REG_Creg_INT_EXT_INCR
		| ad5932_defs_t::REG_Creg_SYNCSEL
		| ad5932_defs_t::REG_Creg_SYNCOUTEN
	);

	// set frequency
	const ad5932_freq_pair_t freq = ad5932_calc_delta_freq(220, 200000/*FIXME*/);
	spi_.write16(ad5932_defs_t::REG_f_start_lo | freq.lo_12);
	spi_.write16(ad5932_defs_t::REG_f_start_hi | freq.hi_12);

	spi_.write16(ad5932_defs_t::REG_Nincr | 4095);

	spi_.write16(ad5932_defs_t::REG_delta_f_lo | 10);
	spi_.write16(ad5932_defs_t::REG_delta_f_hi | 0);

	spi_.write16(
		ad5932_defs_t::REG_t_int_mclk_periods
		| ad5932_defs_t::REG_t_int_multiplier_1
		| 2047
	);

	ctrl_pulse();
}


void ad5932_t::ctrl_pulse()
{
	stm32_lib::gpio::set_state(pin_ctrl_, 1);
	//vTaskDelay(1); // FIXME: portable, >2 mclk periods
	for (volatile int i=0; i<100; ++i) {}
	stm32_lib::gpio::set_state(pin_ctrl_, 0);
}

