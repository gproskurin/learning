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
//#include "semphr.h"
#include "sx1276.h"

#include <stdint.h>
#include <string.h>
#include <array>

#include "ugui.h"

#define PRIO_BLINK 1
#define PRIO_DISPLAY 2
#define PRIO_LORA 6
#define PRIO_LOGGER 8 // FIXME


lora::task_data_t task_data_lora;
extern "C" const sx1276::hwconf_t hwc;

#if 0
constexpr sx1276::hwconf_t hwc_emb = {
	.spi = SPI1,
	.spi_af = 0,
	.pin_spi_nss = bsp::sx1276::pin_spi_nss,
	.pin_spi_sck = bsp::sx1276::pin_spi_sck,
	.pin_spi_miso = bsp::sx1276::pin_spi_miso,
	.pin_spi_mosi = bsp::sx1276::pin_spi_mosi,
	.pin_dio0 = bsp::sx1276::pin_dio0,
	//.pin_radio_reset{GPIOC_BASE, 0},
	//.pin_sx1276_reset{GPIOA_BASE, 11},
	//.pin_radio_tcxo_vcc{GPIOA_BASE, 12},
};
#endif


usart_logger_t logger(UART_VCOM, "logger", PRIO_LOGGER);


void usart_init(NRF_UART_Type* const usart)
{
	bsp::pin_vcom_txd.set(nrf5_lib::gpio::dir_t::output, nrf5_lib::gpio::pull_t::pu);

	usart->CONFIG = 0;
	usart->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud115200;
	static_assert(UART_BAUDRATE_BAUDRATE_Baud115200 == 0x01D7E000);
	usart->PSELTXD = bsp::pin_vcom_txd.reg;
	usart->PSELRXD = 0xFFFFFFFF;
	usart->PSELCTS = 0xFFFFFFFF;
	usart->PSELRTS = 0xFFFFFFFF;
	usart->ENABLE = 4; // enable
}


StaticTask_t xTaskBufferIdle;
freertos_utils::task_stack_t<64> idle_task_stack;
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


freertos_utils::pin_toggle_task_t g_pin_led1("blink_led1", bsp::pin_led_1, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_led2("blink_led2", bsp::pin_led_2, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_led3("blink_led3", bsp::pin_led_3, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_led4("blink_led4", bsp::pin_led_4, PRIO_BLINK);


namespace display {
	constexpr nrf5_lib::gpio::pin_t pin_spi_nss{31};
	constexpr nrf5_lib::gpio::pin_t pin_spi_sck{3};
	constexpr nrf5_lib::gpio::pin_t pin_spi_mosi{22};
	constexpr nrf5_lib::gpio::pin_t pin_rst{28};
	constexpr nrf5_lib::gpio::pin_t pin_dc{29};

	void spi_write_data(const uint8_t* const data, size_t size);
	void spi_write_cmd(uint8_t cmd);

	using display_t = stm32_lib::display::display_t<128, 64, spi_write_cmd, spi_write_data>;

	display_t display;
	UG_GUI ugui;

#define SPI_DEV (NRF_SPIM0) // uses DMA
	void spi_write(const uint8_t* const data, size_t size)
	{
		SPI_DEV->EVENTS_END = 0;
		SPI_DEV->TXD.PTR = reinterpret_cast<uint32_t>(data);
		SPI_DEV->TXD.MAXCNT = size;
		uint8_t garbage;
		SPI_DEV->RXD.PTR = reinterpret_cast<uint32_t>(&garbage); // be safe and set this to something valid
		SPI_DEV->RXD.MAXCNT = 0;
		SPI_DEV->TASKS_START = 1;
		while(! (SPI_DEV->EVENTS_END)) {}
		SPI_DEV->TASKS_STOP = 1;
	}

	void init_display()
	{
		pin_rst.set(nrf5_lib::gpio::state_t::hi, nrf5_lib::gpio::dir_t::output);
		pin_dc.set(nrf5_lib::gpio::state_t::lo, nrf5_lib::gpio::dir_t::output);

		pin_spi_nss.set(
			nrf5_lib::gpio::state_t::hi,
			nrf5_lib::gpio::dir_t::output
		);
		pin_spi_sck.set(
			nrf5_lib::gpio::state_t::lo, // CPOL
			//nrf5_lib::gpio::pull_t::pd,
			nrf5_lib::gpio::dir_t::output
		);
		pin_spi_mosi.set(
			nrf5_lib::gpio::state_t::lo,
			//nrf5_lib::gpio::pull_t::pd,
			nrf5_lib::gpio::dir_t::output
		);

		// reset
		pin_rst.set_state(0);
		vTaskDelay(1);
		pin_rst.set_state(1);

		// init SPI
		SPI_DEV->ENABLE = 0;
		SPI_DEV->INTENCLR = 0xFFFFFFFF;
		SPI_DEV->CONFIG = 0b000; // LSB first
		SPI_DEV->PSEL.SCK = pin_spi_sck.reg;
		SPI_DEV->PSEL.MOSI = pin_spi_mosi.reg;
		SPI_DEV->PSEL.MISO = 0xFFFFFFFF;
		//SPI_DEV->FREQUENCY = 0x10000000; // 1Mbps
		SPI_DEV->FREQUENCY = 0x02000000; // 125kbps
		SPI_DEV->ENABLE = 7;
	}

	void nss_0()
	{
		pin_spi_nss.set_state(0);
		for (volatile int i=0; i<100; ++i) {}
	}
	void nss_1()
	{
		for (volatile int i=0; i<100; ++i) {}
		pin_spi_nss.set_state(1);
	}

	void spi_write_cmd(uint8_t cmd)
	{
		pin_dc.set_state(0);
		nss_0();
		spi_write(&cmd, 1);
		nss_1();
	}

	void spi_write_data(const uint8_t* const data, size_t size)
	{
		pin_dc.set_state(1);
		nss_0();
		spi_write(data, size);
		nss_1();
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
	logger.log_async("DISPLAY init done\r\n");

	constexpr std::array<uint8_t, 8> init_cmds{
		0xAE, // display off
		0xDA, 0b00010010, // COM pins configuration: sequential, disable COM left/right remap
		0x20, 0b10, // page addressing mode
		0x8D, 0x14, // enable charge pump
		0xAF // display on
	};

	for (const auto cmd : init_cmds) {
		display::spi_write_cmd(cmd);
	}
	display::display.flush_buffer();

	UG_Init(&display::ugui, &display::ugui_pset, display::display.x_size(), display::display.y_size());
	UG_SelectGUI(&display::ugui);
	UG_FontSelect(&FONT_8X14);

	UG_FillScreen(0);
	UG_PutString(1, 2, "Display 012345678901234567890123456789012345678901234567890123456789");
	UG_Update();
	display::display.flush_buffer();

	vTaskDelay(configTICK_RATE_HZ*10);

	for (;;) {
		display::display.clear();
		display::display.flush_buffer();
		for(uint16_t i=0; i < display::display.y_size(); ++i) {
			display::display.draw_point(i, i, 1);
			display::display.flush_buffer();
			vTaskDelay(5);
		}
		vTaskDelay(configTICK_RATE_HZ);
	}

	logger.log_async("DISPLAY loop\r\n");
	for (;;) {
		vTaskDelay(configTICK_RATE_HZ);
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


__attribute__ ((noreturn)) void main()
{
	usart_init(UART_VCOM);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	g_pin_led1.init_pin();
	g_pin_led2.init_pin();
	g_pin_led3.init_pin();
	g_pin_led4.init_pin();
	g_pin_led1.pulse_continuous(configTICK_RATE_HZ/32, configTICK_RATE_HZ*31/32);
	g_pin_led2.pulse_continuous(configTICK_RATE_HZ/32, configTICK_RATE_HZ/10);
	//g_pin_led3.pulse_continuous(configTICK_RATE_HZ/13, configTICK_RATE_HZ/7);
	//g_pin_led4.pulse_continuous(configTICK_RATE_HZ/5, configTICK_RATE_HZ/11);

	logger.log_sync("Creating LORA task...\r\n");
	lora::create_task("lora", PRIO_LORA, task_data_lora, &hwc);
	logger.log_sync("Created LORA task\r\n");

	logger.log_sync("Creating DISPLAY task...\r\n");
	create_task_display();
	logger.log_sync("Created DISPLAY task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

