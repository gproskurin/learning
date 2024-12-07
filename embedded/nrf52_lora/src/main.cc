#include "cmsis_device.h"
#include <nrf52.h>
#include "lib_nrf5.h"
#include <nrf52_bitfields.h>
#include "lcd.h"
#include "bsp.h"
#include "freertos_utils.h"
#include "logging.h"

#include "lora.h"

#include "FreeRTOS.h"
#include "task.h"
#include "sx1276.h"
#include "logger_fwd.h"

#include <stdint.h>
#include <string.h>
#include <array>

#include "ugui.h"

#define PRIO_BLINK 1
#define PRIO_DISPLAY 2
#define PRIO_LORA 4
#define PRIO_LOGGER 3


#define LOG_UART UARTE_VCOM

logging::logger_t<log_dev_t> logger("logger", PRIO_LOGGER);

void log_sync(const char* s)
{
	nrf5_lib::uart::uarte_send(LOG_UART, s);
}


lora::task_data_t task_data_lora;
extern "C" const sx1276::hwconf_t hwc;


StaticTask_t xTaskBufferIdle;
freertos_utils::task_stack_t<64> idle_task_stack;
extern "C"
void vApplicationGetIdleTaskMemory(StaticTask_t **tcbIdle, StackType_t **stackIdle, uint32_t *stackSizeIdle)
{
	*tcbIdle = &xTaskBufferIdle;
	*stackIdle = idle_task_stack.data();
	*stackSizeIdle = idle_task_stack.size();
}


freertos_utils::pin_toggle_task_t g_pin_led1("blink_led1", bsp::pin_led_1, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_led2("blink_led2", bsp::pin_led_2, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_led3("blink_led3", bsp::pin_led_3, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_led4("blink_led4", bsp::pin_led_4, PRIO_BLINK);


namespace display {
	constexpr nrf5_lib::gpio::pin_t pin_vcc{NRF_P0_BASE, 2};
	constexpr nrf5_lib::gpio::pin_t pin_scl{NRF_P0_BASE, 26};
	constexpr nrf5_lib::gpio::pin_t pin_sda{NRF_P0_BASE, 27};
	constexpr uint8_t i2c_addr = 0x3C;

	void i2c_write_cmd(uint8_t cmd);
	void i2c_write_data(const uint8_t* const data, size_t size);

	using s_display = stm32_lib::display::display_type_72_40<i2c_write_cmd, i2c_write_data>;
	s_display::display_t display;
	UG_GUI ugui;

#define DISPLAY_DEV NRF_TWIM0
	void init_display()
	{
		pin_vcc.set(nrf5_lib::gpio::state_t::lo, nrf5_lib::gpio::dir_t::output);
		pin_scl.set(nrf5_lib::gpio::pull_t::pu);
		pin_sda.set(nrf5_lib::gpio::pull_t::pu);
		vTaskDelay(configTICK_RATE_HZ/10);
		pin_vcc.set(nrf5_lib::gpio::state_t::hi);
		vTaskDelay(configTICK_RATE_HZ/5);

		// init I2C
		DISPLAY_DEV->ENABLE = 0;
		DISPLAY_DEV->INTENCLR = 0xFFFFFFFF;
		DISPLAY_DEV->PSEL.SCL = pin_scl.reg;
		DISPLAY_DEV->PSEL.SDA = pin_sda.reg;
		//DISPLAY_DEV->FREQUENCY = 0x01980000; // 100 kbps
		//DISPLAY_DEV->FREQUENCY = 0x04000000; // 250 kbps
		DISPLAY_DEV->FREQUENCY = 0x06400000; // 400 kbps
		DISPLAY_DEV->ADDRESS = i2c_addr;
		DISPLAY_DEV->ENABLE = 6;
		vTaskDelay(configTICK_RATE_HZ/10);
	}

	void i2c_write(const uint8_t* data, size_t size)
	{
		if (size < 1) {
			logger.log_async("ERROR: empty i2c write to display\r\n");
			return;
		}

		DISPLAY_DEV->TXD.PTR = reinterpret_cast<uint32_t>(data);
		DISPLAY_DEV->TXD.MAXCNT = size;
		DISPLAY_DEV->SHORTS = (1 << 9); // LASTTX_STOP
		DISPLAY_DEV->EVENTS_STOPPED = 0;
		DISPLAY_DEV->TASKS_STARTTX = 1;
		while (!DISPLAY_DEV->EVENTS_STOPPED) {}
	}

	void i2c_write_cmd(uint8_t cmd)
	{
		const std::array<uint8_t, 2> buf{0, cmd};
		i2c_write(buf.data(), buf.size());
	}

	void i2c_write_data(const uint8_t* const data, size_t size)
	{
		if (size != display.x_size()) {
			logger.log_async("WRONG SIZE\r\n");
		}
		static std::array<uint8_t, display.x_size()+1> buf; // FIXME
		buf[0] = 0b01000000; // data
		for (size_t i=0; i<size; ++i) {
			buf[1+i] = data[i];
		}
		i2c_write(buf.data(), buf.size());
	}

	void ugui_pset(int16_t x, int16_t y, uint32_t color)
	{
		display.draw_point(x, y, color);
	}
}


freertos_utils::task_data_t<1024> task_data_display;


void task_function_display(void*)
{
	logger.log_async("DISPLAY task started\r\n");
	display::init_display();

	for (const auto cmd : display::s_display::init_cmds) {
		display::i2c_write_cmd(cmd);
	}
	display::display.flush_buffer();
	logger.log_async("DISPLAY init done\r\n");

	UG_Init(&display::ugui, &display::ugui_pset, display::display.x_size(), display::display.y_size());
	UG_SelectGUI(&display::ugui);
	UG_FontSelect(&FONT_8X14);

	UG_FillScreen(0);
	UG_PutString(1, 2, "Display 012345678901234567890123456789012345678901234567890123456789");
	UG_Update();
	display::display.flush_buffer();

	vTaskDelay(configTICK_RATE_HZ*10);

	logger.log_async("DISPLAY loop\r\n");
	for (;;) {
		display::display.clear();
		display::display.flush_buffer();
		for (uint16_t y=0; y<display::display.y_size(); ++y) {
			display::display.draw_point(0, y, 1);
			display::display.draw_point(display::display.x_size()-1, y, 1);
		}
		for (uint16_t x=0; x<display::display.x_size(); ++x) {
			display::display.draw_point(x, 0, 1);
			display::display.draw_point(x, display::display.y_size()-1, 1);
		}
		for (uint16_t y=0; y<display::display.y_size(); ++y) {
			display::display.draw_point(y, y, 1);
			display::display.flush_buffer();
			vTaskDelay(5);
		}
		vTaskDelay(configTICK_RATE_HZ*3);
	}
}


void create_task_display()
{
	task_data_display.task_handle = xTaskCreateStatic(
		&task_function_display,
		"DISPLAY",
		task_data_display.stack.size(),
		nullptr,
		PRIO_DISPLAY,
		task_data_display.stack.data(),
		&task_data_display.task_buffer
	);
}


extern "C" __attribute__ ((interrupt)) void IntHandler_Uart0()
{
	if (NRF_UARTE0->EVENTS_ENDTX) {
		NRF_UARTE0->EVENTS_ENDTX = 0;
		logger.notify_from_isr(0b11); // FIXME
	}
}


__attribute__ ((noreturn)) void main()
{
	nrf5_lib::uart::uarte_init(LOG_UART, bsp::pin_vcom_txd);
	LOG_UART->INTEN = 0;
	log_sync("\r\nLogger initialized (sync)\r\n");

	g_pin_led1.init_pin();
	g_pin_led2.init_pin();
	g_pin_led3.init_pin();
	g_pin_led4.init_pin();
	g_pin_led1.pulse_continuous(configTICK_RATE_HZ/32, configTICK_RATE_HZ*31/32);
	g_pin_led2.pulse_continuous(configTICK_RATE_HZ/32, configTICK_RATE_HZ/10);
	//g_pin_led3.pulse_continuous(configTICK_RATE_HZ/13, configTICK_RATE_HZ/7);
	//g_pin_led4.pulse_continuous(configTICK_RATE_HZ/5, configTICK_RATE_HZ/11);

	log_sync("Creating LORA task...\r\n");
	lora::create_task("lora", PRIO_LORA, task_data_lora, &hwc);
	log_sync("Created LORA task\r\n");

	log_sync("Creating DISPLAY task...\r\n");
	create_task_display();
	log_sync("Created DISPLAY task\r\n");

	log_sync("Starting FreeRTOS scheduler\r\n");
	LOG_UART->INTEN = UARTE_INTEN_ENDTX_Msk;
	logger.init();
	NVIC_EnableIRQ(UARTE0_UART0_IRQn);
	vTaskStartScheduler();

	log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

