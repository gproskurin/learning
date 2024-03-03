#include "FreeRTOS.h"
#include "task.h"

#include "lora.h"
#include "bsp.h"


extern void perif_init_irq_dio0();
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


constexpr sx1276::lora_config_t lora_config = {
	.freq = 866000000,
	.sf = 7,
	.crc = true,
	.bw = sx1276::cf_t::bw_khz_125,
	.ecr = sx1276::cf_t::ecr_4_5,
	.preamble_length = 8,
	.implicit_header = false
};


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


#if 0
template <typename Pins>
void printf_dio_status(const char* msg, const Pins& pins)
{
	static char buf[6+3 +11];
	char* b = buf;
	for (size_t i=0; i<pins.size(); ++i) {
		*b++ = '0' + (pins[i].get_state() ? 1 : 0);
	}
	*b++ = '\r';
	*b++ = '\n';
	*b++ = 0;
	logger.log_async(msg);
	logger.log_async(buf);
	vTaskDelay(1);
}
#endif


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
	vTaskDelay(1); // > 100 us

	// make floating
	pin.set(stm32_lib::gpio::mode_t::input);
	vTaskDelay(configTICK_RATE_HZ/10); // > 5 ms
}


void lora_configure(const sx1276::hwconf_t& hwc)
{
	sx1276::spi_sx1276_t spi(hwc.spi, hwc.pin_spi_nss);

	logger.log_async("--- version\r\n");
	static char buf[8];
	log_async_2(sx1276::regs_t::Reg_Version, spi.get_reg(sx1276::regs_t::Reg_Version), buf);

	// RegOpMode
	spi.set_reg(0x01, 0b10000000); // Lora, sleep mode

	// carrier frequency
	// RegFrMsb, RegFrMid, RegFrLsb
	{
		constexpr uint32_t freq_reg = calc_freq_reg<lora_config.freq>();
		static_assert(freq_reg == 0xD88000);
		spi.set_reg(0x06, uint8_t((freq_reg >> 16) & 0xFF)); // msb
		spi.set_reg(0x07, uint8_t((freq_reg >> 8) & 0xFF)); // mid
		spi.set_reg(0x08, uint8_t(freq_reg & 0xFF)); // lsb
	}

	// RegModemConfig1
	spi.set_reg(
		0x1D,
		(lora_config.bw << sx1276::cf_t::bw_Pos)
			| (lora_config.ecr << sx1276::cf_t::ecr_Pos)
			| ((lora_config.implicit_header ? 1 : 0) << sx1276::cf_t::implicit_header_Pos)
	);
	// RegModemConfig2
	spi.set_reg(
		0x1E,
		(lora_config.sf << sx1276::cf_t::sf_Pos)
			| ((lora_config.crc ? 1 : 0) << sx1276::cf_t::crc_Pos)
	);

	// RegPreamble
	spi.set_reg(0x20, (lora_config.preamble_length >> 8) & 0xFF); // msb
	spi.set_reg(0x21, lora_config.preamble_length & 0xFF); // lsb

	spi.set_reg(sx1276::regs_t::Reg_IrqFlags, 0xFF); // clear all
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

	// init dio
	hwp->pin_dio0.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::no_pupd);

	reset_radio_pin(bsp::sx1276::pin_reset);

	lora_configure(*hwp);

	sx1276::spi_sx1276_t spi(hwp->spi, hwp->pin_spi_nss);

	spi.set_reg(sx1276::regs_t::Reg_IrqFlagsMask, 0);
	spi.set_reg(0x40, 0b00); // RegDioMapping1
	//spi.set_reg(0x41, 0b00); // RegDioMapping2
	perif_init_irq_dio0();
	spi.set_reg(0x01, (spi.get_reg(0x01) & ~(0b111)) | 0b101); // RXCONT

	//printf_dio_status("DIO after init\r\n", hwp->pins_dio);
	for(;;) {
		xTaskNotifyWait(0, 0xffffffff, nullptr, configTICK_RATE_HZ /*1sec*/);
		if (spi.get_reg(sx1276::regs_t::Reg_IrqFlags) & sx1276::reg_val_t::RegIrqFlags_RxDone)
		{
			// clear flag
			spi.set_reg(sx1276::regs_t::Reg_IrqFlags, sx1276::reg_val_t::RegIrqFlags_RxDone);

			// show off
			g_pin_blue.pulse_once(configTICK_RATE_HZ/100);
			logger.log_async("LORA_EMB: recv\r\n");

			static std::array<uint8_t, 256> rx_buf;
			const auto rx_size = spi.recv(rx_buf.data());
			if (true || rx_size != 7) {
				char buf[5];
				log_async_1(rx_size, buf);
				logger.log_async(reinterpret_cast<const char*>(rx_buf.data()));
			}
		}
	}
}


void task_function_ext(void* arg)
{
	vTaskDelay(configTICK_RATE_HZ/2);
	logger.log_async("LORA_EXT task started\r\n");
	const sx1276::hwconf_t* const hwp = reinterpret_cast<const sx1276::hwconf_t*>(arg);

	sx1276::spi_sx_init(*hwp, false /*slow*/);

	reset_radio_pin(stm32_lib::gpio::pin_t(GPIOB_BASE, 2));

	sx1276::spi_sx1276_t spi(hwp->spi, hwp->pin_spi_nss);

	lora_configure(*hwp);

	for(;;) {
		static const std::array<uint8_t, 7> tx_buf{'B', 'z', 'y', 'k', '\r', '\n', 0};
		vTaskDelay(configTICK_RATE_HZ*5);
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

