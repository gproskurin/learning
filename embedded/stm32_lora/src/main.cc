#include "lib_stm32.h"
#include "lcd.h"
#include "sx1276.h"
#include "bsp.h"
#include "freertos_utils.h"
#include "logging.h"
#include "logger_fwd.h"

#include "lora.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>

#include "ugui.h"

#define PRIO_BLINK 1
#define PRIO_DISPLAY_1 2
#define PRIO_LORA_EMB 6
#define PRIO_LOGGER 8 // FIXME


lora::task_data_t task_data_lora_emb;

extern "C"
const sx1276::hwconf_t hwc_emb = {
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


logging::logger_t<log_dev_t> logger("logger", PRIO_LOGGER);


void log_sync(const char* s)
{
	stm32_lib::usart::send(USART_STLINK, s);
}


void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


void bus_init()
{
#if defined TARGET_STM32L072
	// flash
	FLASH->ACR |= FLASH_ACR_LATENCY;

	// clock: switch from MSI to HSI
	// turn on HSI & wait
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY)) {}
	// switch to HSI
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (0b01 << RCC_CFGR_SW_Pos);
	// switch off MSI & wait
	RCC->CR &= ~RCC_CR_MSION;
	while (RCC->CR & RCC_CR_MSIRDY) {}

	// GPIOs
	RCC->IOPENR = RCC_IOPENR_IOPAEN_Msk | RCC_IOPENR_IOPBEN_Msk | RCC_IOPENR_IOPCEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_IOPARST | RCC_IOPRSTR_IOPBRST | RCC_IOPRSTR_IOPCRST);

	// USART2 & I2C1
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN_Msk | RCC_APB1ENR_I2C1EN_Msk /*| RCC_APB1ENR_SPI2EN_Msk*/;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_USART2RST_Msk | RCC_APB1RSTR_I2C1RST_Msk /*| RCC_APB1RSTR_SPI2RST_Msk*/);

	// SPI1 & SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk | RCC_APB2ENR_SYSCFGEN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST_Msk | RCC_APB2RSTR_SYSCFGRST_Msk);

	// DMA
	RCC->AHBENR |= RCC_AHBENR_DMAEN;
	toggle_bits_10(&RCC->AHBRSTR, RCC_AHBRSTR_DMARST_Msk);

	DMA1_CSELR->CSELR =
		(0b0001 << DMA_CSELR_C2S_Pos) // SPI1_RX
		| (0b0001 << DMA_CSELR_C3S_Pos) // SPI1_TX
		| (0b0100 << DMA_CSELR_C4S_Pos); // USART2_TX
	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

#if 0
	// PB2 interrupt TODO: do not hardcode PB2, use bit masks stuff
	{
		auto const cr = &SYSCFG->EXTICR[pin_userbutton.reg / 4];
		*cr = (*cr & ~(SYSCFG_EXTICR1_EXTI2_Msk)) | SYSCFG_EXTICR1_EXTI2_PB;
	}

	// EXTI
	pin_userbutton.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::no_pupd);
	EXTI->IMR |= (1 << pin_userbutton.reg);
	EXTI->RTSR &= ~(1 << pin_userbutton.reg); // disable rising edge interrupt (button release)
	EXTI->FTSR |= (1 << pin_userbutton.reg); // enable falling edge interrupt (button press)
	NVIC_SetPriority(EXTI2_3_IRQn, 13);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
#endif
#endif
}


void perif_init_irq_dio0()
{
	// PB4 sx1276/dio0
	auto const cr = &SYSCFG->EXTICR[bsp::sx1276::pin_dio0.reg / 4];
	*cr = (*cr & ~(SYSCFG_EXTICR2_EXTI4_Msk)) | SYSCFG_EXTICR2_EXTI4_PB;

	// EXTI
	EXTI->IMR |= (1 << bsp::sx1276::pin_dio0.reg);
	EXTI->RTSR |= (1 << bsp::sx1276::pin_dio0.reg); // irq on raising edge
	EXTI->FTSR &= ~(1 << bsp::sx1276::pin_dio0.reg);
	NVIC_SetPriority(EXTI4_15_IRQn, 0);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
}


extern "C" __attribute__ ((interrupt)) void IntHandler_DMA1_Channel4_5_6_7()
{
	auto const isr = DMA1->ISR;

	{
		uint32_t ev_log = 0;
		if (isr & DMA_ISR_TCIF4) {
			DMA1->IFCR = DMA_IFCR_CTCIF4;
			ev_log |= stm32_lib::dma::dma_result_t::tc;
		}
		if (isr & DMA_ISR_TEIF4) {
			DMA1->IFCR = DMA_IFCR_CTEIF4;
			ev_log |= stm32_lib::dma::dma_result_t::te;
		}
		if (ev_log) {
			logger.notify_from_isr(ev_log);
		}
	}
}


#if 0
//volatile bool blue_state = false;
extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI23()
{
	if (EXTI->PR & (1 << bsp::pin_userbutton.reg)) {
		//stm32_lib::gpio::set_state(pin_led_blue, (blue_state = !blue_state));
		EXTI->PR = (1 << bsp::pin_userbutton.reg);
	}
}
#endif

extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI4_15()
{
	if (EXTI->PR & (1 << bsp::sx1276::pin_dio0.reg)) {
		EXTI->PR = (1 << bsp::sx1276::pin_dio0.reg);

		BaseType_t yield = pdFALSE;
		xTaskNotifyFromISR(task_data_lora_emb.task_handle, 0, eSetBits, &yield);
		portYIELD_FROM_ISR(yield);
	}
}


extern "C" __attribute__ ((interrupt)) void IntHandler_DMA1_Channel2_3()
{
	uint32_t events = 0;
	uint32_t ifcr_clear = 0;
	auto const isr = DMA1->ISR;

	// channel2 - RX
	if (isr & DMA_ISR_TEIF2) {
		events |= stm32_lib::dma::dma_result_t::te;
		ifcr_clear |= DMA_IFCR_CTEIF2 | DMA_IFCR_CGIF2;
	}
	if (isr & DMA_ISR_TCIF2) {
		events |= stm32_lib::dma::dma_result_t::tc;
		ifcr_clear |= DMA_IFCR_CTCIF2 | DMA_IFCR_CGIF2;
	}

	if (events) {
		DMA1->IFCR = ifcr_clear;
		xTaskNotifyFromISR(task_data_lora_emb.task_handle, events, eSetBits, nullptr);
	}
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


freertos_utils::pin_toggle_task_t g_pin_green2("blink_green2", bsp::pin_led_green2, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);


namespace display1 {
#define i2c_display I2C1
	constexpr stm32_lib::gpio::pin_t pin_vcc(GPIOA_BASE, 9);
	constexpr stm32_lib::gpio::pin_t pin_scl(GPIOB_BASE, 8);
	constexpr stm32_lib::gpio::pin_t pin_sda(GPIOB_BASE, 9);
	constexpr uint8_t i2c_af = 4;
	constexpr uint8_t i2c_addr = 0x3C;

	void i2c_write_data(const uint8_t* const data, size_t size);
	void i2c_write_cmd0(uint8_t cmd);
	using display_t = stm32_lib::display::display_t<128, 32, i2c_write_cmd0, i2c_write_data>;

	display_t display;
	UG_GUI ugui;

	void init_display()
	{
		// power on
		stm32_lib::gpio::set_state(pin_vcc, 0);
		pin_vcc.set(
			stm32_lib::gpio::mode_t::output,
			stm32_lib::gpio::pupd_t::no_pupd,
			stm32_lib::gpio::speed_t::bits_00
		);
		stm32_lib::gpio::set_state(pin_vcc, 1);
		vTaskDelay(configTICK_RATE_HZ/10);

		stm32_lib::i2c::init_pins(pin_scl, i2c_af, pin_sda, i2c_af);
		i2c_display->CR1 = 0;
		i2c_display->CR2 = 0;
		i2c_display->TIMINGR = (i2c_display->TIMINGR & ~(I2C_TIMINGR_PRESC_Msk))
			| (0b0010 << I2C_TIMINGR_PRESC_Pos);
		i2c_display->CR1 = I2C_CR1_PE;
	}


	void i2c_write_data(const uint8_t* const data, size_t size)
	{
		if (size != display.x_size()) {
			logger.log_async("WRONG SIZE\r\n");
		}
		//static_assert(size == 128);
		std::array<uint8_t, display.x_size()+1> buf; // FIXME
		buf[0] = 0b01000000; // data
		for (size_t i=0; i<size; ++i) {
			buf[1+i] = data[i];
		}
		stm32_lib::i2c::write(i2c_display, i2c_addr, buf.data(), size+1);
	}

	void i2c_write_cmd0(uint8_t cmd)
	{
		const std::array<uint8_t, 2> buf{0, cmd};
		stm32_lib::i2c::write(i2c_display, i2c_addr, buf.data(), buf.size());
	}

	void ugui_pset(int16_t x, int16_t y, uint32_t color)
	{
		display.draw_point(x, y, color);
	}
}


#if 0
namespace display2 {
	constexpr stm32_lib::gpio::pin_t pin_dc{GPIOA_BASE, 10};
	constexpr stm32_lib::gpio::pin_t pin_rst{GPIOA_BASE, 0};
	constexpr stm32_lib::gpio::pin_t pin_vdd{GPIOA_BASE, 8};
	constexpr stm32_lib::gpio::pin_t pin_spi_nss{GPIOA_BASE, 9};
	constexpr stm32_lib::gpio::pin_t pin_spi_sck{GPIOB_BASE, 13};
	constexpr stm32_lib::gpio::pin_t pin_spi_mosi{GPIOB_BASE, 15};

	void spi_write_data(const uint8_t* const data, size_t size);
	void spi_write_cmd(uint8_t cmd);

	using display_t = stm32_lib::display::display_t<128, 64, spi_write_cmd, spi_write_data>;

	display_t display;
	UG_GUI ugui;

	void init_display()
	{
		stm32_lib::spi::init_pin_nss(pin_spi_nss);

		// D/C#
		pin_dc.set(
			stm32_lib::gpio::pupd_t::no_pupd,
			stm32_lib::gpio::speed_t::bits_00,
			stm32_lib::gpio::mode_t::output
		);
		stm32_lib::gpio::set_state(pin_dc, 0);

		pin_rst.set(
			stm32_lib::gpio::pupd_t::no_pupd,
			stm32_lib::gpio::speed_t::bits_00,
			stm32_lib::gpio::mode_t::output
		);
		stm32_lib::gpio::set_state(pin_rst, 1);

		// power up
		pin_vdd.set(
			stm32_lib::gpio::pupd_t::no_pupd,
			stm32_lib::gpio::speed_t::bits_00,
			stm32_lib::gpio::mode_t::output
		);
		stm32_lib::gpio::set_state(pin_vdd, true);

		// reset
		stm32_lib::gpio::set_state(pin_rst, 0);
		vTaskDelay(1);
		stm32_lib::gpio::set_state(pin_rst, 1);

		logger.log_async("DISPLAY-2 init done\r\n");
	}

	void nss_0()
	{
		stm32_lib::gpio::set_state(pin_spi_nss, 0);
		for (volatile int i=0; i<100; ++i) {}
	}
	void nss_1()
	{
		for (volatile int i=0; i<100; ++i) {}
		stm32_lib::gpio::set_state(pin_spi_nss, 1);
	}

	void spi_write_cmd(uint8_t cmd)
	{
		stm32_lib::gpio::set_state(pin_dc, 0);
		nss_0();
		stm32_lib::spi::write<uint8_t>(SPI2, 1, &cmd, nullptr);
		nss_1();
	}

	void spi_write_data(const uint8_t* const data, size_t size)
	{
		stm32_lib::gpio::set_state(pin_dc, 1);
		nss_0();
		stm32_lib::spi::write<uint8_t>(SPI2, size, data, nullptr);
		nss_1();
	}

	void ugui_pset(int16_t x, int16_t y, uint32_t color)
	{
		display.draw_point(x, y, color);
	}
}
#endif


freertos_utils::task_data_t<1024> task_data_display_1;
//freertos_utils::task_data_t<1024> task_data_display_2;


void task_function_display_1(void*)
{
	logger.log_async("DISPLAY-1 task started\r\n");
	display1::init_display();

	constexpr std::array<uint8_t, 6> init_cmds{
		0xAE, // display off
		0xDA, 0b00000010, // COM pins configuration: sequential, disable COM left/right remap
		0x8D, 0x14, // enable charge pump
		0xAF // display on
	};
	for (const auto cmd : init_cmds) {
		display1::i2c_write_cmd0(cmd);
	}
	display1::display.flush_buffer();

	UG_Init(&display1::ugui, &display1::ugui_pset, display1::display.x_size(), display1::display.y_size());
	UG_SelectGUI(&display1::ugui);
	UG_FontSelect(&FONT_8X14);

	UG_FillScreen(0);
	UG_PutString(1, 2, "Display-1 01234567890123456789");
	UG_Update();
	display1::display.flush_buffer();

	vTaskDelay(configTICK_RATE_HZ*3);

	display1::display.clear();
	display1::display.flush_buffer();

	/*
	for(uint16_t y=0; y < display1::display.y_size(); ++y) {
		for (uint16_t x=0; x < display1::display.x_size(); ++x) {
			display1::display.draw_point(x, y, 1);
			display1::display.flush_buffer();
			vTaskDelay(1);
		}
		logger.log_async("DISPLAY-1 row done\r\n");
	}
	*/

	logger.log_async("DISPLAY-1 loop\r\n");
	for (;;) {
		vTaskDelay(configTICK_RATE_HZ*5);
	}
}


#if 0
void task_function_display_2(void*)
{
	vTaskDelay(configTICK_RATE_HZ*5);
	logger.log_async("DISPLAY-2 task started\r\n");
	display2::init_display();

	constexpr std::array<uint8_t, 8> init_cmds{
		0xAE, // display off
		0xDA, 0b00010010, // COM pins configuration: sequential, disable COM left/right remap
		0x20, 0b10, // page addressing mode
		0x8D, 0x14, // enable charge pump
		0xAF // display on
	};

	spi2_lock();
	for (const auto cmd : init_cmds) {
		display2::spi_write_cmd(cmd);
	}
	display2::display.flush_buffer();
	spi2_unlock();

	UG_Init(&display2::ugui, &display2::ugui_pset, display2::display.x_size(), display2::display.y_size());
	UG_SelectGUI(&display2::ugui);
	UG_FontSelect(&FONT_8X14);

	UG_FillScreen(0);
	UG_PutString(1, 2, "Display-2 012345678901234567890123456789012345678901234567890123456789");
	UG_Update();
	display2::display.flush_buffer();

	vTaskDelay(configTICK_RATE_HZ*3);

	display2::display.clear();
	display2::display.flush_buffer();
	for(uint16_t y=0; y < display2::display.y_size(); ++y) {
		for (uint16_t x=0; x < display2::display.x_size(); ++x) {
			display2::display.draw_point(x, y, 1);
			spi2_lock();
			display2::display.flush_buffer();
			spi2_unlock();
			vTaskDelay(1);
		}
		logger.log_async("DISPLAY-2 row done\r\n");
	}

	logger.log_async("DISPLAY-2 loop\r\n");
	for (;;) {
		vTaskDelay(configTICK_RATE_HZ);
	}
}
#endif


void create_task_display_1()
{
	task_data_display_1.task_handle = xTaskCreateStatic(
		&task_function_display_1,
		"DISPLAY-1",
		task_data_display_1.stack.size(),
		nullptr,
		PRIO_DISPLAY_1,
		task_data_display_1.stack.data(),
		&task_data_display_1.task_buffer
	);
}


__attribute__ ((noreturn)) void main()
{
	bus_init();

	stm32_lib::usart::init_logger_uart<configCPU_CLOCK_HZ>(
		USART_STLINK,
		bsp::usart_stlink_pin_tx,
		USART_STLINK_PIN_TX_AF
	);

	log_sync("\r\nLogger initialized (sync)\r\n");

	g_pin_green.init_pin();
	g_pin_green2.init_pin();
	g_pin_green2.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/25);
	g_pin_blue.init_pin();
	g_pin_red.init_pin();

	log_sync("Creating LORA_EMB task...\r\n");
	lora::create_task_emb("lora_emb", PRIO_LORA_EMB, task_data_lora_emb, &hwc_emb);
	log_sync("Created LORA_EMB task\r\n");

	//logger.log_sync("Creating DISPLAY-1 task...\r\n");
	//create_task_display_1();
	//logger.log_sync("Created DISPLAY-1 task\r\n");

	log_sync("Starting FreeRTOS scheduler\r\n");
	logger.init();
	vTaskStartScheduler();

	log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

