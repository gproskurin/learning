#include "freertos_utils.h"

#include "lora.h"


extern freertos_utils::pin_toggle_task_t g_pin_green2;
extern freertos_utils::pin_toggle_task_t g_pin_blue;
extern freertos_utils::pin_toggle_task_t g_pin_red;
extern freertos_utils::pin_toggle_task_t g_pin_green;


namespace lora {


namespace {

char prn_halfbyte(uint8_t x)
{
	return ((x <= 9) ? '0' : ('A'-10)) + x;
}

char* printf_byte(uint8_t x, char* buf)
{
	buf[0] = prn_halfbyte(x >> 4);
	buf[1] = prn_halfbyte(x & 0b1111);
	return buf+2;
}

void log_async_2(uint8_t x1, uint8_t x2, char* const buf0)
{
	auto buf = buf0;
	buf = printf_byte(x1, buf);
	*buf++ = ' ';
	buf = printf_byte(x2, buf);
	*buf++ = '\r';
	*buf++ = '\n';
	*buf++ = 0;
	logger.log_async(buf0);
}

} // namespace


// LORA task
struct task_data_t {
	freertos_utils::task_stack_t<256> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
} task_data;


void task_function(void*)
{
	sx1276::spi_sx_init();

	//sx1276::init_radio_pin(sx1276::pin_radio_tcxo_vcc);
	sx1276::init_radio_pin(sx1276::pin_radio_ant_sw_rx);
	sx1276::init_radio_pin(sx1276::pin_radio_ant_sw_tx_boost);
	sx1276::init_radio_pin(sx1276::pin_radio_ant_sw_tx_rfo);

	//stm32_lib::gpio::set_state(pin_radio_tcxo_vcc, 1);
	vTaskDelay(configTICK_RATE_HZ/10);

	// init dio
	//for (const auto& p : sx1276::pins_dio) {
	//	p.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::pd);
	//}

	// reset radio
	{
		using namespace stm32_lib::gpio;
		using mode_t = stm32_lib::gpio::mode_t;

		sx1276::pin_radio_reset.set(
			stm32_lib::gpio::mode_t::output,
			stm32_lib::gpio::otype_t::push_pull,
			stm32_lib::gpio::pupd_t::no_pupd,
			stm32_lib::gpio::speed_t::bits_11
		);
		sx1276::pin_radio_reset.set(mode_t::output, otype_t::push_pull, pupd_t::pu);

		stm32_lib::gpio::set_state(sx1276::pin_radio_reset, 0);
		vTaskDelay(configTICK_RATE_HZ/10);

		sx1276::pin_radio_reset.set(
			stm32_lib::gpio::mode_t::input,
			stm32_lib::gpio::pupd_t::no_pupd
		);
		vTaskDelay(configTICK_RATE_HZ/10);
		//pin_radio_reset.set(stm32_lib::gpio::pupd_t::pu);
	}

	sx1276::spi_sx1276_t spi(SPI_SX1276, sx1276::pin_sxspi_nss);

	static std::array<std::array<char, 8>, 127> bufs;
	for (uint8_t reg=0x01; reg<=0x14; ++reg) {
		const auto x = spi.get_reg(reg);
		log_async_2(reg, x, bufs[reg].data());
	}
	{
		logger.log_async("--- version\r\n");
		static char buf[8];
		constexpr uint8_t reg = 0x42;
		log_async_2(reg, spi.get_reg(reg), buf);
	}

	constexpr uint8_t reg = 0x02;

	static char buf1[8];
	log_async_2(reg, spi.get_reg(reg), buf1);

	static char buf2[8];
	log_async_2(reg, spi.set_reg(reg, 0x3e), buf2);

	static char buf3[8];
	log_async_2(reg, spi.get_reg(reg), buf3);

	static char buf4[8];
	log_async_2(reg, spi.set_reg(reg, 0x1a), buf4);

	static char buf5[8];
	log_async_2(reg, spi.get_reg(reg), buf5);

	for(;;) {
		g_pin_green.pulse_once(configTICK_RATE_HZ/8);
		vTaskDelay(configTICK_RATE_HZ);
	}
}


void create_task(const char* task_name, UBaseType_t prio)
{
	task_data.task_handle = xTaskCreateStatic(
		&task_function,
		task_name,
		task_data.stack.size(),
		nullptr, // reinterpret_cast<void*>(&lora_task_data),
		prio,
		task_data.stack.data(),
		&task_data.task_buffer
	);
}


} // namespace

