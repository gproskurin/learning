#include "FreeRTOS.h"
#include "task.h"

#include "logger_fwd.h"

#include "sx1276.h"
#include "bsp.h"

#include "lib_stm32.h"
#include "lora.h"



extern void perif_init_irq_dio0();
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_green2;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_blue;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_red;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_green;

extern "C" const sx1276::hwconf_t hwc_emb;
using emb_spi_dma_t = stm32_lib::dma::spi_dma_t<SPI1_BASE, DMA1_Channel3_BASE, DMA1_Channel2_BASE>;

namespace spi {
	void wait_spi_complete()
	{
		auto const _events = freertos_utils::notify_wait_any(
			stm32_lib::dma::dma_result_t::te | stm32_lib::dma::dma_result_t::tc
		);
	}

	void cb_spi_nss(bool s)
	{
		hwc_emb.pin_spi_nss.set_state(s);
	}

	void cb_spi_write(const uint8_t* tx_data, size_t size, uint8_t* rx_data)
	{
		emb_spi_dma_t::start(size, tx_data, rx_data);
		wait_spi_complete();
		emb_spi_dma_t::stop();
	}
} //spi


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
	*b = 0;
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
	*buf = 0;
	logger.log_async(buf0);
}

/*
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
*/


void task_function_emb(void* arg)
{
	logger.log_async("LORA_EMB task started\r\n");

	const sx1276::hwconf_t* const hwp = reinterpret_cast<const sx1276::hwconf_t*>(arg);

	sx1276::spi_sx_init(*hwp, true);
	emb_spi_dma_t::init();

	//sx1276::init_radio_pin(hwp->pin_radio_tcxo_vcc);
	//sx1276::init_radio_pin(hwp->pin_radio_ant_sw_rx);
	//sx1276::init_radio_pin(hwp->pin_radio_ant_sw_tx_boost);
	//sx1276::init_radio_pin(hwp->pin_radio_ant_sw_tx_rfo);

	//stm32_lib::gpio::set_state(hwp->pin_radio_tcxo_vcc, 1);

	// init dio
	hwp->pin_dio0.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::no_pupd);

	sx1276::reset_radio_pin(bsp::sx1276::pin_reset);

	//sx1276::lora_configure<sx1276::LoraConfig_1>(*hwp);
	//sx1276::spi_sx1276_t spi(hwp->spi, hwp->pin_spi_nss);
	using spi_t = sx1276::spi_sx1276_t<spi::cb_spi_write, spi::cb_spi_nss>;
	spi_t spi;
	sx1276::lora_configure<sx1276::LoraConfig_1>(spi);

	spi.set_reg(sx1276::regs_t::IrqFlagsMask, 0);
	spi.set_reg(0x40, 0b00); // RegDioMapping1
	//spi.set_reg(0x41, 0b00); // RegDioMapping2
	logger.log_async("LORA_EMB loop\r\n");

	perif_init_irq_dio0();
	spi.set_reg(sx1276::regs_t::FifoRxBaseAddr, 0);
	spi.set_reg(0x01, sx1276::opmode(spi.get_reg(0x01), sx1276::reg_val_t::OpMode_Mode_RXCONTINUOUS));

	for(;;) {
		xTaskNotifyWait(0, 0xffffffff, nullptr, portMAX_DELAY);
		const auto flags = spi.get_reg(sx1276::regs_t::IrqFlags);
		if (flags & sx1276::reg_val_t::IrqFlags_RxDone)
		{
			const bool crc_error = flags & sx1276::reg_val_t::IrqFlags_PayloadCrcError;
			if (crc_error) {
				g_pin_red.pulse_once(configTICK_RATE_HZ*2);
				logger.log_async("LORA_EMB: recv crc error\r\n");
			}
			// clear flags
			spi.set_reg(
				sx1276::regs_t::IrqFlags,
				sx1276::reg_val_t::IrqFlags_RxDone | sx1276::reg_val_t::IrqFlags_PayloadCrcError
			);

			// show off
			g_pin_blue.pulse_once(configTICK_RATE_HZ/10);
			logger.log_async("LORA_EMB: recv\r\n");

			static std::array<uint8_t, 257> rx_buf;
			rx_buf[256] = 0;
			const auto rx_size = spi.fifo_read(rx_buf.data());
			if (true /*|| rx_size != 7*/) {
				{
					static std::array<char, 8> num_buf;
					log_async_1(rx_size, num_buf.data());
				}

				logger.log_async(reinterpret_cast<const char*>(rx_buf.data()));
			}
			logger.log_async("RCV_DONE\r\n");
		}
	}
}


#if 0
void task_function_ext(void* arg)
{
	vTaskDelay(configTICK_RATE_HZ/2);
	logger.log_async("LORA_EXT task started\r\n");
	const sx1276::hwconf_t* const hwp = reinterpret_cast<const sx1276::hwconf_t*>(arg);

	spi2_lock();
	sx1276::spi_sx_init(*hwp, false /*slow*/);

	sx1276::reset_radio_pin(stm32_lib::gpio::pin_t(GPIOB_BASE, 2));

	sx1276::lora_configure<sx1276::LoraConfig_1>(*hwp);
	sx1276::spi_sx1276_t spi(hwp->spi, hwp->pin_spi_nss);

	{
		logger.log_async("--- version\r\n");
		static char buf[8];
		log_async_2(sx1276::regs_t::Version, spi.get_reg(sx1276::regs_t::Version), buf);
	}

	spi.set_reg(sx1276::regs_t::IrqFlagsMask, 0);

	const uint8_t reg_01 = spi.get_reg(sx1276::regs_t::OpMode);
	spi.set_reg(sx1276::regs_t::OpMode, sx1276::opmode(reg_01, sx1276::reg_val_t::OpMode_Mode_STDBY));
	spi2_unlock();
	for(;;) {
		static const std::array<uint8_t, 11> tx_buf{
			'B', 'E', 'E', 'F',
			'B', 'z', 'y', 'k', '\r', '\n', 0
		};
		vTaskDelay(configTICK_RATE_HZ*5);

		g_pin_green.on();
		spi2_lock();
		spi.fifo_write(tx_buf.data(), tx_buf.size());
		spi.set_reg(sx1276::regs_t::OpMode, sx1276::opmode(reg_01, sx1276::reg_val_t::OpMode_Mode_TX));
		while (!(spi.get_reg(sx1276::regs_t::IrqFlags) & sx1276::reg_val_t::IrqFlags_TxDone)) {
			spi2_unlock();
			vTaskDelay(1);
			spi2_lock();
		}
		spi.set_reg(sx1276::regs_t::IrqFlags, sx1276::reg_val_t::IrqFlags_TxDone); // clear flag
		spi2_unlock();
		g_pin_green.off();

		logger.log_async("TX - done\r\n");
	}
}
#endif


} // namespace


void lora::create_task_emb(
	const char* task_name,
	UBaseType_t prio,
	lora::task_data_t& task_data,
	const sx1276::hwconf_t* hwp
)
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

