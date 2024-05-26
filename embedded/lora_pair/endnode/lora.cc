#include "FreeRTOS.h"
#include "task.h"

#include "freertos_utils.h"
#include "sx1276.h"
#include "bsp.h"

#include "logging.h"
#include "lora.h"

freertos_utils::task_data_t<512> task_data_lora;

extern usart_logger_t logger;

extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_green;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_blue;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_red;
extern freertos_utils::pin_toggle_task_t<stm32_lib::gpio::pin_t> g_pin_green2;

extern void perif_init_irq_dio0();

extern "C"
const sx1276::hwconf_t hwc_emb;


enum spi_dma_flags_t {
	rx_tc = (1 << 0),
	rx_te = (1 << 1),
	//tx_tc = (1 << 2),
	//tx_te = (1 << 3),
};


namespace {


namespace spi {
	void wait_spi_complete()
	{
		// waiting for RX to complete
		for (;;) {
			uint32_t events = 0;
			if (
				(xTaskNotifyWait(0, 0xffffffff, &events, portMAX_DELAY) == pdTRUE)
				&& (events & (spi_dma_flags_t::rx_tc | spi_dma_flags_t::rx_te))
			)
			{
				return;
			}
		}
	}

	void cb_spi_nss(bool s)
	{
		hwc_emb.pin_spi_nss.set_state(s);
	}

	void cb_spi_write(const uint8_t* tx_data, size_t size, uint8_t* rx_data)
	{
		//stm32_lib::spi::write<uint8_t>(hwc_emb.spi, size, tx_data, rx_data);

		hwc_emb.dma_channel_rx->CNDTR = hwc_emb.dma_channel_tx->CNDTR = size;

		hwc_emb.dma_channel_rx->CMAR = reinterpret_cast<uint32_t>(rx_data);
		hwc_emb.dma_channel_tx->CMAR = reinterpret_cast<uint32_t>(tx_data);

		// enable TX first
		hwc_emb.dma_channel_tx->CCR = hwc_emb.dma_ccr_en_tx;
		hwc_emb.dma_channel_rx->CCR = hwc_emb.dma_ccr_en_rx;

		wait_spi_complete();

		hwc_emb.dma_channel_tx->CCR = hwc_emb.dma_ccr_tx;
		hwc_emb.dma_channel_rx->CCR = hwc_emb.dma_ccr_rx;
		// TODO: check BSY flag?
	}
} //spi


void task_function(void* arg)
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
		events |= spi_dma_flags_t::rx_te;
		ifcr_clear |= DMA_IFCR_CTEIF2 | DMA_IFCR_CGIF2;
	}
	if (isr & DMA_ISR_TCIF2) {
		events |= spi_dma_flags_t::rx_tc;
		ifcr_clear |= DMA_IFCR_CTCIF2 | DMA_IFCR_CGIF2;
	}

#if 0
	// channel3 - TX
	if (isr & DMA_ISR_TEIF3) {
		events |= spi_dma_flags_t::tx_te;
		ifcr_clear |= DMA_IFCR_CTEIF3 | DMA_IFCR_CGIF3;
	}
	if (isr & DMA_ISR_TCIF3) {
		events |= spi_dma_flags_t::tx_tc;
		ifcr_clear |= DMA_IFCR_CTCIF3 | DMA_IFCR_CGIF3;
	}
#endif

	if (events) {
		DMA1->IFCR = ifcr_clear;
		xTaskNotifyFromISR(task_data_lora.task_handle, events, eSetBits, nullptr);
	}
}

