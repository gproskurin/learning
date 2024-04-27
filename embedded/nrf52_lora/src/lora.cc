#include "FreeRTOS.h"
#include "task.h"

#include "sx1276.h"
#include "bsp.h"

#include "logging.h"
#include "lora.h"
#include <nrf52.h>

extern usart_logger_t logger;

extern void perif_init_irq_dio0();
extern freertos_utils::pin_toggle_task_t<nrf5_lib::gpio::pin_t> g_pin_led1;
extern freertos_utils::pin_toggle_task_t<nrf5_lib::gpio::pin_t> g_pin_led2;
extern freertos_utils::pin_toggle_task_t<nrf5_lib::gpio::pin_t> g_pin_led3;
extern freertos_utils::pin_toggle_task_t<nrf5_lib::gpio::pin_t> g_pin_led4;


extern "C"
const sx1276::hwconf_t hwc{
	.spi{NRF_SPIM1},

	.pin_spi_nss{3},
	.pin_spi_sck{4},
	.pin_spi_miso{28},
	.pin_spi_mosi{29},

	.pin_dio0{31}
};
constexpr nrf5_lib::gpio::pin_t sx1276_reset{30};

namespace {


namespace spi {
	void cb_spi_nss(bool s)
	{
		hwc.pin_spi_nss.set_state(s);
	}

	void cb_spi_write(const uint8_t* tx_data, size_t size, uint8_t* rx_data)
	{
		hwc.spi->TXD.PTR = reinterpret_cast<uint32_t>(tx_data);
		hwc.spi->TXD.MAXCNT = (tx_data ? size : 0);
		hwc.spi->RXD.PTR = reinterpret_cast<uint32_t>(rx_data);
		hwc.spi->RXD.MAXCNT = (rx_data ? size : 0);

		hwc.spi->EVENTS_END = 0;
		hwc.spi->TASKS_START = 1;
		while(! (hwc.spi->EVENTS_END)) {}
		hwc.spi->TASKS_STOP = 1;
	}
} //spi


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


#if 0
void log_async_1(uint8_t x, char* const buf0)
{
	auto buf = buf0;
	buf = printf_byte(x, buf);
	*buf++ = '\r';
	*buf++ = '\n';
	*buf++ = 0;
	logger.log_async(buf0);
}
#endif

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


void task_function(void* arg)
{
	logger.log_async("LORA task started\r\n");
	const sx1276::hwconf_t* const hwp = reinterpret_cast<const sx1276::hwconf_t*>(arg);

	sx1276::spi_sx_init(*hwp, false /*slow*/);

	sx1276::reset_radio_pin(sx1276_reset);

	using spi_t = sx1276::spi_sx1276_t<spi::cb_spi_write, spi::cb_spi_nss>;
	spi_t spi;
	sx1276::lora_configure<sx1276::LoraConfig_1>(spi);

	{
		logger.log_async("--- version\r\n");
		static char buf[8];
		log_async_2(sx1276::regs_t::Version, spi.get_reg(sx1276::regs_t::Version), buf);
	}

	spi.set_reg(sx1276::regs_t::IrqFlagsMask, 0);

	const uint8_t reg_01 = spi.get_reg(sx1276::regs_t::OpMode);
	spi.set_reg(sx1276::regs_t::OpMode, sx1276::opmode(reg_01, sx1276::reg_val_t::OpMode_Mode_STDBY));
	for(;;) {
		static const std::array<uint8_t, 11> tx_buf{
			'B', 'E', 'E', 'F',
			'B', 'z', 'y', 'k', '\r', '\n', 0
		};
		vTaskDelay(configTICK_RATE_HZ*5);

		g_pin_led4.on();
		spi.fifo_write(tx_buf.data(), tx_buf.size());

		// perform CAD, wait for channel to be free to avoid collision while sending
		for (;;) {
			spi.set_reg(sx1276::regs_t::OpMode, sx1276::opmode(reg_01, sx1276::reg_val_t::OpMode_Mode_CAD));
			// wait for CadDone
			uint8_t f;
			while (!(f=spi.get_reg(sx1276::regs_t::IrqFlags) & sx1276::reg_val_t::IrqFlags_CadDone)) {
				vTaskDelay(1);
			}

			if (!(f & sx1276::reg_val_t::IrqFlags_CadDetected)) {
				// CAD not detected, channel is free, goto TX ASAP, clear flags after TX
				break;
			}

			// channel is busy, sleep for some time and retry
			spi.set_reg(
				sx1276::regs_t::IrqFlags,
				sx1276::reg_val_t::IrqFlags_CadDone | sx1276::reg_val_t::IrqFlags_CadDetected
			);
			vTaskDelay(configTICK_RATE_HZ/10); // TODO random
		}

		spi.set_reg(sx1276::regs_t::OpMode, sx1276::opmode(reg_01, sx1276::reg_val_t::OpMode_Mode_TX));
		uint8_t f;
		while (!(f=spi.get_reg(sx1276::regs_t::IrqFlags) & sx1276::reg_val_t::IrqFlags_TxDone)) {
			vTaskDelay(1);
		}
		// clear all used flags
		spi.set_reg(
			sx1276::regs_t::IrqFlags,
			sx1276::reg_val_t::IrqFlags_TxDone | sx1276::reg_val_t::IrqFlags_CadDone
		);
		g_pin_led4.off();
		g_pin_led3.pulse_once(configTICK_RATE_HZ/10);

		logger.log_async("TX - done\r\n");
	}
}


} // namespace


void lora::create_task(
	const char* task_name,
	UBaseType_t prio,
	lora::task_data_t& task_data,
	const sx1276::hwconf_t* hwp
)
{
	task_data.task_handle = xTaskCreateStatic(
		&task_function,
		task_name,
		task_data.stack.size(),
		const_cast<void*>(reinterpret_cast<const void*>(hwp)),
		prio,
		task_data.stack.data(),
		&task_data.task_buffer
	);
}

