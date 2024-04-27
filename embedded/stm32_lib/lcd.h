#ifndef _gpr_lcd_included_
#define _gpr_lcd_included_

#include "cmsis_device.h"

#include <stddef.h>
#include <algorithm>
#include <array>


namespace stm32_lib {


namespace display {


typedef void (*display_write_cmd_t)(uint8_t cmd);
typedef void (*display_write_data_t)(const uint8_t *data, size_t size);


template <uint16_t Xsize, uint16_t Ysize, display_write_cmd_t write_cmd_func, display_write_data_t write_data_func>
class display_t {
	static_assert(Ysize % 8 == 0);
	std::array<uint8_t, Xsize*Ysize/8> fb_data{0};
public:
	static constexpr uint16_t x_size() { return Xsize; }
	static constexpr uint16_t y_size() { return Ysize; }
public:
	void draw_point(uint16_t x, uint16_t y, bool color)
	{
		if (x < Xsize && y < Ysize) {
			const auto page = y / 8;
			const auto seg_bit = y % 8;
			const size_t idx = page * Xsize + x;
			if (color) {
				fb_data[idx] |= (1 << seg_bit);
			} else {
				fb_data[idx] &= ~(1 << seg_bit);
			}
		}
	}

	void clear()
	{
		std::fill(fb_data.begin(), fb_data.end(), 0);
	}

	void flush_buffer() const
	{
		for (uint8_t page=0; page < Ysize/8; ++page) {
			write_cmd_func(0xB0 + page);

			// set X offset
			static_assert(Xsize <= 128);
			static_assert(Xsize % 2 == 0);
			constexpr uint8_t dx = (128 - Xsize) / 2;
			write_cmd_func(0x00 + (dx & 0b1111 /*low nimble*/));
			write_cmd_func(0x10 + (dx >> 4 /*high nimble*/));

			write_data_func(&fb_data[Xsize * page], Xsize);
		}
	}
};

template <display_write_cmd_t write_cmd_func, display_write_data_t write_data_func>
struct display_type_72_40 {
	static constexpr std::array<uint8_t, 10> init_cmds{
		0xAE, // display off
		0xDA, 0b00010010, // COM pins configuration: sequential, disable COM left/right remap
		0x20, 0b10, // page addressing mode
		0x8D, 0x14, // enable charge pump
		0xAD, 0b00110000, // set internal Iref
		0xAF // display on
	};

	using display_t = stm32_lib::display::display_t<72, 40, write_cmd_func, write_data_func>;
};


#if defined(TARGET_STM32H745_CM7)

constexpr uint32_t hs = 2;
constexpr uint32_t vs = 10;
// h/v front/back porch
constexpr uint32_t hbp = 43;
constexpr uint32_t vbp = 12;
constexpr uint32_t hfp = 8;
constexpr uint32_t vfp = 4;


template <uint32_t Hsize, uint32_t Vsize> class lcd_t
{
	template <uint32_t LayerBaseAddr>
	struct layer_t {
		static
		LTDC_Layer_TypeDef* layer_ptr() { return reinterpret_cast<LTDC_Layer_TypeDef*>(LayerBaseAddr); }

		static void enable() { layer_ptr()->CR = 1; }
		static void disable() { layer_ptr()->CR = 0; }

		uint32_t start_x, start_y, width, height;

		void reconfig_x()
		{
			auto const end_x = std::min(start_x + width, Hsize);
			layer_ptr()->WHPCR =
				((/*Hstop*/ hs + hbp + end_x - 1) << 16)
				| (/*Hstart*/ hs + hbp + start_x)
			;
			layer_ptr()->CFBLR = (((width)*3) << 16) | ((end_x-start_x)*3 + 7);
		}

		void reconfig_y()
		{
			auto const end_y = std::min(start_y + height, Vsize);
			layer_ptr()->WVPCR =
				((/*Vstop*/ vs + vbp + end_y - 1) << 16)
				| (/*Vstart*/ vs + vbp + start_y)
			;
			layer_ptr()->CFBLNR = ((end_y > start_y) ? (end_y - start_y) : 0);
		}
	};


public:
	layer_t<LTDC_Layer1_BASE> layer1;
	layer_t<LTDC_Layer2_BASE> layer2;

public:
	void init()
	{
		LTDC->SSCR = ((hs - 1) << 16) | (vs - 1); // hsync, vsync
		LTDC->BPCR = ((hs + hbp - 1) << 16) | (vs + vbp - 1); // back porch
		LTDC->AWCR = ((hs + hbp + Hsize - 1) << 16) | (vs + vbp + Vsize - 1); // active width/height
		LTDC->TWCR = ((hs + hbp + Hsize + hfp - 1) << 16) | (vs + vbp + Vsize + vfp - 1); // total width/height
		layer_t<LTDC_Layer1_BASE>::disable();
		layer_t<LTDC_Layer2_BASE>::disable();
		LTDC_Layer1->PFCR = 1; // RGB888, FIXME
		LTDC_Layer2->PFCR = 1; // RGB888, FIXME
	}

	static void update_now() { LTDC->SRCR = LTDC_SRCR_IMR; }
	static void update_v() { LTDC->SRCR = LTDC_SRCR_VBR; }
};

#endif


} // namespace display


} // namespace stm32_lib


#endif

