#include "lora.h"


extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_green2;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_blue;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_red;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_green;


namespace lora {


namespace {


template <uint32_t Freq>
constexpr uint32_t calc_freq_reg()
{
	constexpr uint64_t fr = uint64_t(Freq) * uint64_t(1 << 19) / uint64_t(32000000);
	static_assert(fr <= 0xFFFFFF);
	return uint32_t(fr);
}


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

void log_async_1(uint8_t x, char* const buf0)
{
	auto buf = buf0;
	buf = printf_byte(x, buf);
	*buf++ = '\r';
	*buf++ = '\n';
	*buf++ = 0;
	logger.log_async(buf0);
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


void reset_radio_pin(const stm32_lib::gpio::pin_t& pin)
{
	// pull the pin down
	stm32_lib::gpio::set_state(pin, 0);
	pin.set(
		stm32_lib::gpio::mode_t::output,
		stm32_lib::gpio::otype_t::push_pull,
		stm32_lib::gpio::pupd_t::no_pupd,
		stm32_lib::gpio::speed_t::bits_11
	);
	vTaskDelay(1);

	// make floating
	pin.set(stm32_lib::gpio::mode_t::input);
	vTaskDelay(configTICK_RATE_HZ/10);
}


} // namespace


void task_function_emb(void* arg)
{
	logger.log_async("LORA_EMB task started\r\n");
	const sx1276::hwconf_t* const hwp = reinterpret_cast<const sx1276::hwconf_t*>(arg);

	sx1276::spi_sx_init(*hwp, true);

	//sx1276::init_radio_pin(hwp->pin_radio_tcxo_vcc);
	//sx1276::init_radio_pin(hwp->pin_radio_ant_sw_rx);
	//sx1276::init_radio_pin(hwp->pin_radio_ant_sw_tx_boost);
	//sx1276::init_radio_pin(hwp->pin_radio_ant_sw_tx_rfo);

	//stm32_lib::gpio::set_state(hwp->pin_radio_tcxo_vcc, 1);
	vTaskDelay(configTICK_RATE_HZ/10);

	// init dio
	//for (const auto& p : sx1276::pins_dio) {
	//	p.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::pd);
	//}

	reset_radio_pin(/*hwp->pin_radio_reset*/ stm32_lib::gpio::pin_t(GPIOC_BASE,0));

	sx1276::spi_sx1276_t spi(hwp->spi, hwp->pin_spi_nss);

	logger.log_async("LORA_EMB\r\n");
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

	// RegOpMode
	spi.set_reg(0x01, 0b10000000); // Lora, sleep mode

	// carrier frequency
	// RegFrMsb, RegFrMid, RegFrLsb
	constexpr uint32_t freq = calc_freq_reg<866000000>();
	static_assert(freq == 0xD88000);
	spi.set_reg(0x06, uint8_t((freq >> 16) & 0xFF)); // msb
	spi.set_reg(0x07, uint8_t((freq >> 8) & 0xFF)); // mid
	spi.set_reg(0x08, uint8_t(freq & 0xFF)); // lsb

	// RegModemConfig1
	spi.set_reg(0x1D, (0b0111 << 4) | (0b001 << 1) | 0); // 125kHz, 4/8 error coding, expilcit header
	// RegModemConfig2
	spi.set_reg(0x1E, (7/*SF*/ << 4) | (1 << 2/*crc on*/));

	// RegPreamble
	spi.set_reg(0x20, 0); // msb
	spi.set_reg(0x21, 8); // lsb

	constexpr uint8_t reg_irq_flags = 0x12;
	spi.set_reg(reg_irq_flags, 0xFF); // clear all

	spi.set_reg(0x01, (spi.get_reg(0x01) & ~(0b111)) | 0b101); // RXCONT

	for(;;) {
		constexpr uint8_t flag_rxdone = 1 << 6;
		const auto irq_flags = spi.get_reg(reg_irq_flags);
		if (irq_flags & flag_rxdone) {
			spi.set_reg(reg_irq_flags, flag_rxdone); // clear flag
			g_pin_blue.pulse_once(configTICK_RATE_HZ/100);
			logger.log_async("LORA_EMB: recv\r\n");
			std::array<uint8_t, 256> rx_buf;
			const auto rx_size = spi.recv(rx_buf.data());
			if (true || rx_size != 7) {
				char buf[5];
				log_async_1(rx_size, buf);
				logger.log_async(reinterpret_cast<const char*>(rx_buf.data()));
			}
		}
		vTaskDelay(1);
	}
}


void task_function_ext(void* arg)
{
	vTaskDelay(configTICK_RATE_HZ*3);
	logger.log_async("LORA_EXT task started\r\n");
	const sx1276::hwconf_t* const hwp = reinterpret_cast<const sx1276::hwconf_t*>(arg);

	sx1276::spi_sx_init(*hwp, false /*slow*/);

	// init dio
	//for (const auto& p : sx1276::pins_dio) {
	//	p.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::pd);
	//}

	reset_radio_pin(/*hwp->pin_radio_reset*/ stm32_lib::gpio::pin_t(GPIOB_BASE,2));

	sx1276::spi_sx1276_t spi(hwp->spi, hwp->pin_spi_nss);

	logger.log_async("LORA_EXT\r\n");
	static std::array<std::array<char, 8>, 127> bufs;
	for (uint8_t reg=0x01; reg<=0x14; ++reg) {
		const auto x = spi.get_reg(reg);
		log_async_2(reg, x, bufs[reg].data());
	}
	{
		logger.log_async("--- version ext\r\n");
		static char buf[8];
		constexpr uint8_t reg = 0x42;
		log_async_2(reg, spi.get_reg(reg), buf);
	}

	// RegOpMode
	spi.set_reg(0x01, 0b10000000); // Lora, sleep mode

	// carrier frequency
	// RegFrMsb, RegFrMid, RegFrLsb
	constexpr uint32_t freq = calc_freq_reg<866000000>();
	static_assert(freq == 0xD88000);
	spi.set_reg(0x06, (freq >> 16) & 0xFF); // msb
	spi.set_reg(0x07, (freq >> 8) & 0xFF); // mid
	spi.set_reg(0x08, freq & 0xFF); // lsb

	// RegModemConfig1
	spi.set_reg(0x1D, (0b0111 << 4) | (0b001 << 1) | 0); // 125kHz, 4/8 error coding, expilcit header
	// RegModemConfig2
	spi.set_reg(0x1E, (7/*SF*/ << 4) | (1 << 2/*crc on*/));

	// RegPreamble
	spi.set_reg(0x20, 0); // msb
	spi.set_reg(0x21, 8); // lsb

	constexpr uint8_t reg_irq_flags = 0x12;
	spi.set_reg(reg_irq_flags, 0xFF); // clear all

	for(;;) {
		static const std::array<uint8_t, 7> tx_buf{'B', 'z', 'y', 'k', '\r', '\n', 0};
		vTaskDelay(configTICK_RATE_HZ*10);
		spi.send(tx_buf.data(), tx_buf.size());
		g_pin_green.pulse_once(configTICK_RATE_HZ/100);
		logger.log_async("TX - done\r\n");
	}
}


void create_task_emb(const char* task_name, UBaseType_t prio, task_data_t& task_data, const sx1276::hwconf_t* hwp)
{
	task_data.task_handle = xTaskCreateStatic(
		&task_function_emb,
		task_name,
		task_data.stack.size(),
		const_cast<void*>(reinterpret_cast<const void*>(hwp)),
		prio,
		task_data.stack.data(),
		&task_data.task_buffer
	);
}


void create_task_ext(const char* task_name, UBaseType_t prio, task_data_t& task_data, const sx1276::hwconf_t* hwp)
{
	task_data.task_handle = xTaskCreateStatic(
		&task_function_ext,
		task_name,
		task_data.stack.size(),
		const_cast<void*>(reinterpret_cast<const void*>(hwp)),
		prio,
		task_data.stack.data(),
		&task_data.task_buffer
	);
}


} // namespace

