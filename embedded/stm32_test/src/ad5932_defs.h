#ifndef _gpr_ad5932_defs_h_included_
#define _gpr_ad5932_defs_h_included_

#include <stdint.h>


enum ad5932_defs_t {
	REG_Creg_ADDR = 0b0000 << 12,
	REG_Creg_B24 = 1 << 11,
	REG_Creg_DAC_ENABLE = 1 << 10,
	REG_Creg_SINE_TRI = 1 << 9,
	REG_Creg_MSBOUTEN = 1 << 8,
	REG_Creg_INT_EXT_INCR = 1 << 5,
	REG_Creg_SYNCSEL = 1 << 3,
	REG_Creg_SYNCOUTEN = 1 << 2,

	REG_Creg_Reserved_1 = 0b11010011, // reserved bits to be set to 1
	REG_Creg = REG_Creg_ADDR | REG_Creg_Reserved_1,

	REG_Nincr = 0b0001 << 12,

	REG_delta_f_lo = 0b0010 << 12,
	REG_delta_f_hi = 0b0011 << 12,
	// TODO positive/negative

	REG_t_int_num_cycles     = 0b01000 << 11,
	REG_t_int_mclk_periods   = 0b01100 << 11,
	REG_t_int_multiplier_1   = 0b00000 << 11,
	REG_t_int_multiplier_5   = 0b00001 << 11,
	REG_t_int_multiplier_100 = 0b00010 << 11,
	REG_t_int_multiplier_500 = 0b00011 << 11,

	REG_f_start_lo = 0b1100 << 12,
	REG_f_start_hi = 0b1101 << 12
};


inline
void ad5932_calc_delta_freq(const uint64_t desired_freq, uint64_t mclk_freq, uint16_t* const hi, uint16_t* const lo)
{
	constexpr uint64_t pow24 = (1 << 24); // 2^24
	constexpr uint64_t pow12 = (1 << 12); // 2^12
	const uint64_t res = pow24 * desired_freq / mclk_freq;
	const uint64_t res_lo = res & (pow12 - 1);
	const uint64_t res_hi = (res >> 12) & (pow12 - 1);
	*hi = res_hi;
	*lo = res_lo;
}


#endif

