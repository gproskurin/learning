#include "lib_stm32.h"
#include "lcd.h"
#include "bsp.h"
#include "logging.h"
#include "freertos_utils.h"
#include "ugui.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>


#define PRIO_BLINK 1
#define PRIO_LCD 2
#define PRIO_UGUI 2
#define PRIO_LOGGER 3

#define I2C_TS I2C4


#define USART_CON_BAUDRATE 115200


usart_logger_t logger;

stm32_lib::hsem::hsem_t<0> hsem0;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_FIFOEN | USART_CR1_TE;

	stm32_lib::gpio::set_mode_af_lowspeed_pu(bsp::usart_stlink_pin_tx, USART_STLINK_PIN_TX_AF);
	usart->BRR = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;

	usart->CR1 = cr1;
	usart->CR1 = cr1 | USART_CR1_UE;
}


void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


void periph_init()
{
	RCC->AHB4ENR |=
		RCC_AHB4ENR_GPIOAEN_Msk
		| RCC_AHB4ENR_GPIOBEN_Msk
		| RCC_AHB4ENR_GPIODEN_Msk
		| RCC_AHB4ENR_GPIOGEN_Msk
		| RCC_AHB4ENR_GPIOHEN_Msk
		| RCC_AHB4ENR_GPIOIEN_Msk
		| RCC_AHB4ENR_GPIOJEN_Msk
		| RCC_AHB4ENR_GPIOKEN_Msk
		| RCC_AHB4ENR_HSEMEN_Msk;
	toggle_bits_10(
		&RCC->AHB4RSTR,
		RCC_AHB4RSTR_GPIOARST_Msk
			| RCC_AHB4RSTR_GPIOBRST_Msk
			| RCC_AHB4RSTR_GPIODRST_Msk
			| RCC_AHB4RSTR_GPIOGRST_Msk
			| RCC_AHB4RSTR_GPIOHRST_Msk
			| RCC_AHB4RSTR_GPIOIRST_Msk
			| RCC_AHB4RSTR_GPIOJRST_Msk
			| RCC_AHB4RSTR_GPIOKRST_Msk
			// | RCC_AHB4RSTR_HSEMRST_Msk // TODO
	);

	// USART
	RCC->APB1LENR |= RCC_APB1LENR_USART3EN_Msk;
	toggle_bits_10(&RCC->APB1LRSTR, RCC_APB1LRSTR_USART3RST_Msk);

}


StaticTask_t xTaskBufferIdle;
freertos_utils::task_stack_t<128> idle_task_stack;
extern "C"
void vApplicationGetIdleTaskMemory(StaticTask_t **tcbIdle, StackType_t **stackIdle, uint32_t *stackSizeIdle)
{
	*tcbIdle = &xTaskBufferIdle;
	*stackIdle = idle_task_stack.data();
	*stackSizeIdle = idle_task_stack.size();
}

extern "C"
void vApplicationIdleHook(void)
{
	__WFI();
}


freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green_arduino("blink_green_arduino", bsp::pin_led_green_arduino, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green_vbus("blink_green_vbus", bsp::pin_led_green_vbus_usb_fs, PRIO_BLINK);


constexpr uint32_t display_w = 480;
constexpr uint32_t display_h = 272;
constexpr uint32_t l1_w = 320;
constexpr uint32_t l1_h = 240;
extern std::array<unsigned char, l1_w * l1_h * 3> fb; // RGB888

namespace lcd {

void init_pin_af(const stm32_lib::gpio::pin_t& pin, uint8_t af)
{
	pin.set(
		stm32_lib::gpio::mode_t::af,
		stm32_lib::gpio::otype_t::push_pull,
		stm32_lib::gpio::pupd_t::no_pupd,
		stm32_lib::gpio::speed_t::bits_11,
		stm32_lib::gpio::af_t(af)
	);
}


void ts_init()
{
	RCC->APB4ENR |= RCC_APB4ENR_I2C4EN_Msk;
	toggle_bits_10(&RCC->APB4RSTR, RCC_APB4RSTR_I2C4RST_Msk);

	stm32_lib::i2c::init_pins(
		bsp::lcd::pin_scl, bsp::lcd::af_i2c,
		bsp::lcd::pin_sda, bsp::lcd::af_i2c
	);

	I2C_TS->CR1 = 0;
	//I2C->TIMINGR
	I2C_TS->CR2 = (1 << I2C_CR2_NBYTES_Pos);
	I2C_TS->CR1 = I2C_CR1_PE;
}


stm32_lib::lcd_t<480,272> lcd;
void init()
{
	// lcd pixel clock, PLL3(=64)/div
	RCC->PLL3DIVR = (RCC->PLL3DIVR & ~RCC_PLL3DIVR_R3_Msk) | ((/*div24*/24 - 1) << RCC_PLL3DIVR_R3_Pos);
	RCC->CR |= RCC_CR_PLL3ON;
	while (!(RCC->CR & RCC_CR_PLL3RDY)) {}

	RCC->APB3ENR |= RCC_APB3ENR_LTDCEN_Msk;
	toggle_bits_10(&RCC->APB3RSTR, RCC_APB3RSTR_LTDCRST_Msk);

	bsp::lcd::pin_disp.set(stm32_lib::gpio::speed_t::bits_00);
	bsp::lcd::pin_disp.set_state(1);
	bsp::lcd::pin_disp.set(stm32_lib::gpio::mode_t::output);

	init_pin_af(bsp::lcd::pin_de, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_vsync, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_hsync, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_clk, bsp::lcd::af);

	init_pin_af(bsp::lcd::pin_r0, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_r1, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_r2, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_r3, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_r4, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_r5, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_r6, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_r7, bsp::lcd::af);

	init_pin_af(bsp::lcd::pin_g0, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_g1, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_g2, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_g3, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_g4, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_g5, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_g6, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_g7, bsp::lcd::af);

	init_pin_af(bsp::lcd::pin_b0, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_b1, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_b2, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_b3, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_b4, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_b5, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_b6, bsp::lcd::af);
	init_pin_af(bsp::lcd::pin_b7, bsp::lcd::af);

	lcd.init();

	lcd.layer1.start_x = 20;
	lcd.layer1.width = 320;
	lcd.layer1.reconfig_x();

	lcd.layer1.start_y = 10;
	lcd.layer1.height = 240;
	lcd.layer1.reconfig_y();

#if 0
	lcd.layer2.start_x = 120;
	lcd.layer2.width = 320;
	lcd.layer2.reconfig_x();

	lcd.layer2.start_y = 100;
	lcd.layer2.height = 240;
	lcd.layer2.reconfig_y();
	LTDC_Layer2->DCCR = 0x00000000;
	LTDC_Layer2->CACR = 0xA0;
	LTDC_Layer2->CFBAR = reinterpret_cast<uint32_t>(fb.data());
#endif

	static_assert(sizeof(uint32_t) == sizeof(fb.data()));
	LTDC_Layer1->CFBAR = reinterpret_cast<uint32_t>(fb.data());

	LTDC_Layer1->DCCR = 0x00000000;
	LTDC_Layer1->CACR = 0xC0;

	LTDC->BCCR = 0x00FF0000;

	//LTDC->LIPCR = 10000;
	LTDC->IER = /*LTDC_IER_RRIE | LTDC_IER_TERRIE |*/ LTDC_IER_FUIE /*| LTDC_IER_LIE*/;

	lcd.update_v();

	for (size_t i=0; i<fb.size(); i+=3) {
		std::swap(fb[i], fb[i+2]);
	}

	lcd.layer1.enable();
	//lcd.layer2.enable();
	LTDC->GCR |= LTDC_GCR_LTDCEN;
	logger.log_async("LCD enabled\r\n");
}

} // namespace lcd


freertos_utils::task_data_t<1024> task_data_lcd;


void task_function_lcd(void*)
{
	logger.log_async("LCD task started\r\n");
	lcd::init();
	lcd::ts_init();
	int32_t dx1 = 1;
	int32_t dy1 = 1;
	//int32_t dx2 = 2;
	//int32_t dy2 = -2;
	for(;;) {
		uint32_t icr = 0;
		if (LTDC->ISR & LTDC_ISR_RRIF) {
			logger.log_async("ISR: RRIF\r\n");
			icr |= LTDC_ICR_CRRIF;
		}
		if (LTDC->ISR & LTDC_ISR_TERRIF) {
			logger.log_async("ISR: TERRIF\r\n");
			icr |= LTDC_ICR_CTERRIF;
		}
		if (LTDC->ISR & LTDC_ISR_FUIF) {
			logger.log_async("ISR: FUIF\r\n");
			icr |= LTDC_ICR_CFUIF;
		}
		if (LTDC->ISR & LTDC_ISR_LIF) {
			logger.log_async("ISR: LIF\r\n");
			icr |= LTDC_ICR_CLIF;
		}
		if (icr) {
			LTDC->ICR = icr;
		}
		vTaskDelay(configTICK_RATE_HZ/50);

		if (lcd::lcd.layer1.start_x < 2 || lcd::lcd.layer1.start_x > 450)
			dx1 = -dx1;

		if (lcd::lcd.layer1.start_y < 1 || lcd::lcd.layer1.start_y > 260)
			dy1 = -dy1;

		lcd::lcd.layer1.start_x += dx1;
		lcd::lcd.layer1.reconfig_x();
		lcd::lcd.layer1.start_y += dy1;
		lcd::lcd.layer1.reconfig_y();

#if 0
		if (lcd::lcd.layer2.start_x < 2 || lcd::lcd.layer2.start_x > 453)
			dx2 = -dx2;

		if (lcd::lcd.layer2.start_y < 2 || lcd::lcd.layer2.start_y > 267)
			dy2 = -dy2;

		lcd::lcd.layer2.start_x += dx2;
		lcd::lcd.layer2.reconfig_x();
		lcd::lcd.layer2.start_y += dy2;
		lcd::lcd.layer2.reconfig_y();
#endif

		LTDC->SRCR = 0b10;
	}
}

void create_task_lcd()
{
	task_data_lcd.task_handle = xTaskCreateStatic(
		&task_function_lcd,
		"LCD",
		task_data_lcd.stack.size(),
		nullptr,
		PRIO_LCD,
		task_data_lcd.stack.data(),
		&task_data_lcd.task_buffer
	);
}


freertos_utils::task_data_t<1024> task_data_ugui;

constexpr uint16_t ugui_x = 480/2;
constexpr uint16_t ugui_y = 272/2;
std::array<uint8_t, ugui_x*ugui_y*3> fb_ugui = {0};
UG_GUI ugui;

void fb2_pset(int16_t x, int16_t y, uint32_t color)
{
	// framebuffer is in BGR format
	const size_t idx = 3 * ((ugui_x * y) + x);
	fb_ugui[idx] = color & 0xff; // B
	fb_ugui[idx+1] = (color >> 8) & 0xff; // G
	fb_ugui[idx+2] = (color >> 16) & 0xff; // R
}

std::array<UG_OBJECT, 3> objbuf1;
UG_WINDOW w1;
constexpr uint16_t wx = 10;
constexpr uint16_t wy = 20;
constexpr uint16_t bx = 50;
constexpr uint16_t by = 30;

UG_BUTTON b1;
constexpr uint8_t b1_id = 1;

UG_BUTTON b2;
constexpr uint8_t b2_id = 2;

void w_cb(UG_MESSAGE*)
{
	logger.log_async("UGUI: window message\r\n");
}


uint32_t r = 314159;
uint32_t rnd()
{
	r = r*69069 + 1;
	return r;
}

void task_function_ugui(void*)
{
	logger.log_async("UGUI task started\r\n");
	UG_Init(&ugui, fb2_pset, ugui_x, ugui_y);
	UG_SelectGUI(&ugui);
	UG_FontSelect(&FONT_8X12);
	logger.log_async("UGUI ugui initialized\r\n");

	vTaskDelay(configTICK_RATE_HZ/10);

	lcd::lcd.layer2.start_x = 5;
	lcd::lcd.layer2.width = ugui_x;
	lcd::lcd.layer2.reconfig_x();
	lcd::lcd.layer2.start_y = 5;
	lcd::lcd.layer2.height = ugui_y;
	lcd::lcd.layer2.reconfig_y();
	LTDC_Layer2->CFBAR = reinterpret_cast<uint32_t>(fb_ugui.data());
	lcd::lcd.layer2.enable();
	lcd::lcd.update_v();

	UG_WindowCreate(&w1, objbuf1.data(), objbuf1.size(), w_cb);
	UG_WindowSetTitleText(&w1, "Bzyk bzyk");
	UG_ButtonCreate(&w1, &b1, b1_id, wx, wy, wx+bx, wy+by);
	UG_ButtonSetText(&w1, b1_id, "b1");
	UG_ButtonCreate(&w1, &b2, b2_id, wx+bx+10, wy, wx+bx+10+bx, wy+by);
	UG_ButtonSetText(&w1, b2_id, "b2");
	UG_WindowShow(&w1);
	UG_Update();
	lcd::lcd.update_v();

	if(0) {
		for (int i=0; i<ugui_y/2-1; i += 2) {
			UG_DrawFrame(i, i, (ugui_x-1)-i, (ugui_y-1)-i, rnd() & 0xffffff);
		}
		UG_FillCircle(
			ugui_x/2 + (rnd() % (ugui_x/4)),
			ugui_y/2 + (rnd() % (ugui_y/4)),
			rnd() % (ugui_y/8),
			rnd() & 0xffffff
		);
		UG_Update();
		lcd::lcd.update_v();
		vTaskDelay(configTICK_RATE_HZ/10);
	}

	for(;;) {
		vTaskDelay(configTICK_RATE_HZ);
	}
}

void create_task_ugui()
{
	task_data_lcd.task_handle = xTaskCreateStatic(
		&task_function_ugui,
		"UGUI",
		task_data_ugui.stack.size(),
		nullptr,
		PRIO_UGUI,
		task_data_ugui.stack.data(),
		&task_data_ugui.task_buffer
	);
}

#if 1
template <typename Pin>
void blink(const Pin& pin, int n)
{
	return;
	while (n-- > 0) {
		pin.set_state(0);
		for (volatile int i=0; i<2000000; ++i) {}
		pin.set_state(1);
		for (volatile int i=0; i<1000000; ++i) {}
	}
}
#endif


__attribute__ ((noreturn)) void main()
{
#if 1
	//periph_init();
	const auto& pin = bsp::pin_led_green;
	pin.set_state(1);
	pin.set_mode_output_lowspeed_pushpull();

	blink(pin, 1);
#endif
#if 0
	while (RCC->CR & RCC_CR_D2CKRDY) { blink(pin,3); for (volatile int i=0; i<5000000;++i) {}; } // wait for cpu2 to stop
	//blink(pin, 1);
	periph_init();
	// wakeup cpu2 by generating hsem interrupt
	hsem0.fast_take();
	hsem0.release();
	while (!(RCC->CR & RCC_CR_D2CKRDY)) {} // wait for cpu2 to start
	hsem0.fast_take();
	hsem0.release();
#endif

	periph_init();

	g_pin_green.init_pin();
	g_pin_green.pulse_continuous(configTICK_RATE_HZ, configTICK_RATE_HZ*2);

	g_pin_green_arduino.init_pin();
	g_pin_green_arduino.pulse_continuous(configTICK_RATE_HZ/20, configTICK_RATE_HZ/11);

	g_pin_green_vbus.init_pin();
	g_pin_green_vbus.pulse_continuous(configTICK_RATE_HZ/2, configTICK_RATE_HZ);

	usart_init(USART_STLINK);
	logger.set_usart(USART_STLINK);
	logger.log_sync("\r\nCM7: Logger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");

	logger.log_sync("Creating LCD task...\r\n");
	create_task_lcd();
	logger.log_sync("Created LCD task\r\n");

	logger.log_sync("Creating UGUI task...\r\n");
	create_task_ugui();
	logger.log_sync("Created UGUI task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	for (;;) {}
}

