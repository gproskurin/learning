#ifndef _gpr_lcd_included_
#define _gpr_lcd_included_

#include "cmsis_device.h"

#include <algorithm>


namespace stm32_lib {


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


} // namespace stm32_lib


#endif

