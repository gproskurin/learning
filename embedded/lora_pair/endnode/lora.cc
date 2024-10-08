#include "FreeRTOS.h"
#include "task.h"

#include "freertos_utils.h"
#include "sx1276.h"
#include "bsp.h"

#include "logging.h"
#include "lora.h"

#include "logger_fwd.h"


freertos_utils::task_data_t<512> task_data_lora;

extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_green;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_blue;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_red;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_green2;

extern void perif_init_irq_dio0();

extern "C"
const sx1276::hwconf_t hwc_emb;
using emb_spi_dma_t = stm32_lib::dma::spi_dma_t<SPI1_BASE, DMA1_Channel3_BASE, DMA1_Channel2_BASE>;


namespace {


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


void task_function(void* arg)
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

	using spi_t = sx1276::spi_sx1276_t<spi::cb_spi_write, spi::cb_spi_nss>;
	spi_t spi;
	sx1276::lora_configure<sx1276::LoraConfig_1>(spi);

	spi.set_reg(sx1276::regs_t::IrqFlagsMask, 0);
	spi.set_reg(sx1276::regs_t::IrqFlags, 0xFF); // clear all flags
	//spi.set_reg(0x41, 0b00); // RegDioMapping2

	perif_init_irq_dio0();

	spi.set_reg(sx1276::regs_t::FifoTxBaseAddr, 0);

	const uint8_t reg_opmode = spi.get_reg(sx1276::regs_t::OpMode);
	spi.set_reg(sx1276::regs_t::OpMode, sx1276::opmode(reg_opmode, sx1276::reg_val_t::OpMode_Mode_STDBY));
	for(;;) {
		const std::array<uint8_t, 10> tx_buf{
			'B', 'z', 'y', 'c', 'h', 'e', 'k', '\r', '\n', 0
		};
		vTaskDelay(configTICK_RATE_HZ*5);

		g_pin_blue.on();
		spi.fifo_write(tx_buf.data(), tx_buf.size());

		// perform CAD, wait for channel to be free to avoid collision while sending
		for (;;) {
			spi.set_reg(0x40, 0b10000000); // map dio0 to CadDone
			spi.set_reg(sx1276::regs_t::OpMode, sx1276::opmode(reg_opmode, sx1276::reg_val_t::OpMode_Mode_CAD));
			bool err = false;
			// wait for CadDone
			if (xTaskNotifyWait(0, 0xffffffff, nullptr, configTICK_RATE_HZ) == pdFALSE) {
				// no notification
				logger.log_async("IRQ: CadDone timeout\r\n");
				err = true;
			} else {
				logger.log_async("IRQ: CadDone\r\n");
			}

			const uint8_t flags = spi.get_reg(sx1276::regs_t::IrqFlags);

			if (!(flags & sx1276::reg_val_t::IrqFlags_CadDone)) {
				logger.log_async("ERROR: no CadDone flags\r\n");
				err = true;
			}

			if (err) {
				g_pin_red.pulse_once(configTICK_RATE_HZ*2);
			} else {
				g_pin_red.pulse_once(configTICK_RATE_HZ/10);
			}

			if (!(flags & sx1276::reg_val_t::IrqFlags_CadDetected)) {
				// CAD not detected, channel is free, goto TX ASAP, clear flags after TX
				break;
			}

			logger.log_async("IRQ: CadDetected, busy\r\n");
			// channel is busy, sleep for some time and retry
			spi.set_reg(
				sx1276::regs_t::IrqFlags,
				sx1276::reg_val_t::IrqFlags_CadDone | sx1276::reg_val_t::IrqFlags_CadDetected
			);
			vTaskDelay(configTICK_RATE_HZ/10); // TODO random
		}

		spi.set_reg(0x40, 0b01000000); // map dio0 to TxDone
		spi.set_reg(sx1276::regs_t::OpMode, sx1276::opmode(reg_opmode, sx1276::reg_val_t::OpMode_Mode_TX));
		if (xTaskNotifyWait(0, 0xffffffff, nullptr, configTICK_RATE_HZ*10) == pdFALSE) {
			logger.log_async("IRQ: TxDone timeout\r\n");
		}
		if (! (spi.get_reg(sx1276::regs_t::IrqFlags) & sx1276::reg_val_t::IrqFlags_TxDone)) {
			logger.log_async("ERROR: Tx timeout\r\n");
			g_pin_red.pulse_once(configTICK_RATE_HZ*2);
		} else {
			logger.log_async("TX - ok\r\n");
		}

		// clear all used flags
		spi.set_reg(
			sx1276::regs_t::IrqFlags,
			sx1276::reg_val_t::IrqFlags_TxDone
				| sx1276::reg_val_t::IrqFlags_CadDetected
				| sx1276::reg_val_t::IrqFlags_CadDone
		);
		g_pin_blue.off();
		logger.log_async("TX - done\r\n");
	}
}


} // namespace


void lora::create_task(
	const char* task_name,
	UBaseType_t prio,
	const sx1276::hwconf_t* hwp
)
{
	task_data_lora.task_handle = xTaskCreateStatic(
		&task_function,
		task_name,
		task_data_lora.stack.size(),
		const_cast<void*>(reinterpret_cast<const void*>(hwp)),
		prio,
		task_data_lora.stack.data(),
		&task_data_lora.task_buffer
	);
}


extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI4_15()
{
	if (EXTI->PR & (1 << bsp::sx1276::pin_dio0.reg)) {
		EXTI->PR = (1 << bsp::sx1276::pin_dio0.reg);

		BaseType_t yield = pdFALSE;
		xTaskNotifyFromISR(task_data_lora.task_handle, 0, eSetBits, &yield);
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
		xTaskNotifyFromISR(task_data_lora.task_handle, events, eSetBits, nullptr);
	}
}

