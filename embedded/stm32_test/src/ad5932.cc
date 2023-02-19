#include "ad5932_defs.h"
#include "ad5932.h"


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
	int freq_lo, freq_hi;
	ad5932_calc_delta_freq(500, 0/*FIXME*/, &freq_lo, &freq_hi);
	spi_.write16(ad5932_defs_t::REG_f_start_lo | freq_lo);
	spi_.write16(ad5932_defs_t::REG_f_start_hi | freq_hi);

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
}

