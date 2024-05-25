#include "../common/utils.h"
#include "lib_stm32.h"
#include "bsp.h"
#include "freertos_utils.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>

#define PRIO_BLINK 1
#define PRIO_LORA_RECV 5
#define PRIO_IPCC_SEND 6
#define PRIO_LOGGER 8 // FIXME


#define USART_CON_BAUDRATE 115200

void* ipcc_mp; // pointer to mailbox being sent
constexpr uint8_t ipcc_ch_lora = 3;

usart_logger_t logger(USART_STLINK, "logger_cm0", PRIO_LOGGER);

//freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
//freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);


void bus_init()
{
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


namespace subghz {
	uint8_t Get_Status();
	void log_status(const char* msg);

	void nss_0()
	{
		PWR->SUBGHZSPICR = 0;
		for (volatile int i=0; i<4; ++i) {}
	}

	void nss_1()
	{
		for (volatile int i=0; i<4; ++i) {}
		PWR->SUBGHZSPICR = PWR_SUBGHZSPICR_NSS;
		for (volatile int i=0; i<4; ++i) {}
	}

	void init()
	{
		// reset radio
		RCC->CSR |= RCC_CSR_RFRST;
		while (!(RCC->CSR & RCC_CSR_RFRSTF)) {}
		for (volatile int i=0; i<100; ++i) {}
		RCC->CSR &= ~RCC_CSR_RFRST;
		while (RCC->CSR & RCC_CSR_RFRSTF) {}
		for (volatile int i=0; i<100; ++i) {}

		PWR->SCR = PWR_SCR_CWRFBUSYF;

		nss_0();
		vTaskDelay(configTICK_RATE_HZ/10); // TODO
		nss_1();

		//while (PWR->SR2 & PWR_SR2_RFBUSYMS) {}
		while (PWR->SR2 & PWR_SR2_RFBUSYS) {}

		constexpr uint32_t cr1 =
			SPI_CR1_SSM_Msk
			| SPI_CR1_MSTR_Msk
			| SPI_CR1_SSI_Msk
			| (0b001/*div4*/ << SPI_CR1_BR_Pos) // PCLK3 divider
			//| (0b101/*div64*/ << SPI_CR1_BR_Pos) // PCLK3 divider
			;

		SUBGHZSPI->CR1 = 0;
		SUBGHZSPI->CR1 = cr1;
		SUBGHZSPI->CR2 = SPI_CR2_FRXTH | (0b111/*8bit*/ << SPI_CR2_DS_Pos);

		SUBGHZSPI->CR1 = cr1 | SPI_CR1_SPE;
	}

	void spi_write(size_t size, const uint8_t* tx_buf, uint8_t* rx_buf)
	{
		vTaskDelay(1); // TODO check busy
		subghz::nss_0();
		stm32_lib::spi::write<uint8_t>(SUBGHZSPI, size, tx_buf, rx_buf);
		subghz::nss_1();
	}

	template <typename Container>
	void spi_write_array(const Container& tx_buf)
	{
		spi_write(tx_buf.size(), tx_buf.data(), nullptr);
	}

	uint8_t Read_Register(uint16_t addr)
	{
		const std::array<uint8_t, 5> tx_buf{
			0x1D,
			(addr >> 8) & 0xFF,
			addr & 0xFF,
			0, // status
			0
		};
		std::array<uint8_t, 5> rx_buf;
		spi_write(tx_buf.size(), tx_buf.data(), rx_buf.data());
		return rx_buf[4];
	}

	uint8_t Get_Status()
	{
		constexpr std::array<uint8_t, 2> tx_buf{0xC0, 0};
		std::array<uint8_t, 2> rx_buf;
		spi_write(tx_buf.size(), tx_buf.data(), rx_buf.data());
		return rx_buf[1];
	}

	void GetPacketStatus(uint8_t* rssi_pkt, uint8_t* snr_pkt, uint8_t* signal_rssi_pkt)
	{
		constexpr std::array<uint8_t, 5> tx_buf{0x14, 0, 0, 0, 0};
		std::array<uint8_t, 5> rx_buf;
		spi_write(tx_buf.size(), tx_buf.data(), rx_buf.data());
		*rssi_pkt = rx_buf[2];
		*snr_pkt = rx_buf[3];
		*signal_rssi_pkt = rx_buf[4];
	}

	void Set_BufferBaseAddress(uint8_t tx_addr, uint8_t rx_addr)
	{
		spi_write_array(std::array<uint8_t, 3>{0x8F, tx_addr, rx_addr});
	}

	void Set_PacketType(uint8_t pkt_type)
	{
		spi_write_array(std::array<uint8_t, 2>{0x8A, pkt_type});
	}

	void CalibrateImage( uint32_t freq )
	{
		std::array<uint8_t,2> calFreq;

		if( freq > 900000000 )
		{
			calFreq[0] = 0xE1;
			calFreq[1] = 0xE9;
		}
		else if( freq > 850000000 )
		{
			calFreq[0] = 0xD7;
			calFreq[1] = 0xDB;
		}
		else if( freq > 770000000 )
		{
			calFreq[0] = 0xC1;
			calFreq[1] = 0xC5;
		}
		else if( freq > 460000000 )
		{
			calFreq[0] = 0x75;
			calFreq[1] = 0x81;
		}
		else if( freq > 425000000 )
		{
			calFreq[0] = 0x6B;
			calFreq[1] = 0x6F;
		}
		spi_write_array(std::array<uint8_t, 3>{0x98, calFreq[0], calFreq[1]});
	}

	constexpr uint32_t freq()
	{
		constexpr uint32_t freqInHz = 866000000;
		#define SX126X_XTAL_FREQ		32000000UL
		#define SX126X_PLL_STEP_SHIFT_AMOUNT	( 14 )
		#define SX126X_PLL_STEP_SCALED		( SX126X_XTAL_FREQ >> ( 25 - SX126X_PLL_STEP_SHIFT_AMOUNT ) )
		constexpr uint32_t stepsInt = freqInHz / SX126X_PLL_STEP_SCALED;
		constexpr uint32_t stepsFrac = freqInHz - ( stepsInt * SX126X_PLL_STEP_SCALED );

		return ( stepsInt << SX126X_PLL_STEP_SHIFT_AMOUNT ) + 
           ( ( ( stepsFrac << SX126X_PLL_STEP_SHIFT_AMOUNT ) + ( SX126X_PLL_STEP_SCALED >> 1 ) ) /
             SX126X_PLL_STEP_SCALED );
	}

	void Set_RfFrequency()
	{
		constexpr uint32_t fr = freq(); // TODO
		//constexpr uint64_t fr64 = uint64_t(866000000) * uint64_t(32000000) / uint64_t (1ULL << 25);
		//static_assert(fr64 <= 0xFFFFFFFF);
		//constexpr uint32_t fr = static_cast<uint32_t>(fr64);
		static_assert(fr == 0x36200000);
		spi_write_array(std::array<uint8_t, 5>{
			0x86,
			(fr >> 24) & 0xFF,
			(fr >> 16) & 0xFF,
			(fr >> 8) & 0xFF,
			fr & 0xFF
		});
	}

	void Set_ModulationParams()
	{
		spi_write_array(std::array<uint8_t, 5>{0x8B, 0x7/*sf*/, 0x04/*bw125*/, 0x1/*ecr4_5*/, 0});
	}

	void Cfg_DioIrq()
	{
		spi_write_array(std::array<uint8_t, 9>{
			0x08,
			0xFF, 0xFF, // IRQ global
			0xFF, 0xFF, // IRQ1 line
			0x00, 0x00, // IRQ2 line
			0x00, 0x00 // IRQ3 line
		});
	}

	void Set_PacketParams()
	{
		constexpr bool iq_polarity_inverted = false;
		spi_write_array(std::array<uint8_t, 7>{
			0x8C,
			0x00, 8, // preample length, symbols
			0, // 0=explicit header
			255, // PayloadLength (unused?)
			0, // Crc
			(iq_polarity_inverted ? 1 : 0)
		});

		constexpr uint16_t reg_iq_polarity_lsb = 0x0736;
		constexpr uint8_t pol_mask = 1 << 2;
		const uint8_t pol_reg0 = Read_Register(reg_iq_polarity_lsb);
		const uint8_t pol_reg = (iq_polarity_inverted ? (pol_reg0 & ~pol_mask) : (pol_reg0 | pol_mask));
		if (pol_reg != 0x0D) { logger.log_async("POL_REG error\r\n"); }
		spi_write_array(std::array<uint8_t, 4>{
			0x0D,
			reg_iq_polarity_lsb >> 8,
			reg_iq_polarity_lsb & 0xFF,
			pol_reg
		});
	}

	void Set_Standby()
	{
		spi_write_array(std::array<uint8_t, 2>{0x80, 0});
	}

	void Set_Rx()
	{
		spi_write_array(std::array<uint8_t, 4>{
			0x82,
			0xFF, 0xFF, 0xFF // timeout disabled, continuous receive mode
			//0, 0, 0 // RxSingle
			//0, 0, 1
		});
	}

	uint16_t Get_IrqStatus()
	{
		constexpr std::array<uint8_t, 4> tx_buf{0x12, 0, 0, 0};
		std::array<uint8_t, 4> rx_buf;
		spi_write(tx_buf.size(), tx_buf.data(), rx_buf.data());
		return (uint16_t(rx_buf[2]) << 8) | uint16_t(rx_buf[3]); // convert last 2 bytes to uint16_t
	}

	void Clr_IrqStatus(uint16_t mask)
	{
		static_assert(sizeof(mask) == 2);
		spi_write_array(std::array<uint8_t, 3>{
			0x02,
			static_cast<uint8_t>(mask >> 8),
			static_cast<uint8_t>(mask & 0xFF)
		});
	}

	uint8_t Read_Fifo(uint8_t* rx_data)
	{
		std::array<uint8_t, 4> rx_stat;
		{
			constexpr std::array<uint8_t, 4> tx_stat{0x13, 0, 0, 0}; // GetRxBufferStatus
			spi_write(tx_stat.size(), tx_stat.data(), rx_stat.data());
		}

		auto const rx_size = rx_stat[2];
		auto const rx_off = rx_stat[3];

		const std::array<uint8_t, 3> tx_fifo_cmd{0x1E, rx_off, 0}; // ReadBuffer
		subghz::nss_0();
		stm32_lib::spi::write<uint8_t>(SUBGHZSPI, tx_fifo_cmd.size(), tx_fifo_cmd.data(), nullptr);
		stm32_lib::spi::write<uint8_t>(SUBGHZSPI, rx_size, nullptr, rx_data);
		subghz::nss_1();

		return rx_size;
	}


	void init_lora_recv()
	{
		Set_PacketType(1/*lora*/);

		bsp::sx1276::pin_fe_ctrl1.set_state(0);
		bsp::sx1276::pin_fe_ctrl1.set(stm32_lib::gpio::mode_t::output);
		bsp::sx1276::pin_fe_ctrl2.set_state(0);
		bsp::sx1276::pin_fe_ctrl2.set(stm32_lib::gpio::mode_t::output);
		vTaskDelay(configTICK_RATE_HZ/10);

		spi_write_array(std::array<uint8_t, 5>{0x97, 0x01, 0x00, 0x01, 0x40}); // TCXO voltage 1.7V
		vTaskDelay(configTICK_RATE_HZ/10);

		Set_Standby();
		vTaskDelay(configTICK_RATE_HZ/10);

		spi_write_array(std::array<uint8_t, 2>{0x89, 0x7F}); // Calibrate
		vTaskDelay(configTICK_RATE_HZ/10);

		Set_Standby();
		vTaskDelay(configTICK_RATE_HZ/10);

		spi_write_array(std::array<uint8_t, 2>{0x9D, 0}); // RfSwitchMode
		vTaskDelay(configTICK_RATE_HZ/10);

		spi_write_array(std::array<uint8_t, 2>{0x96, 1}); // regulator mode, 0=LDO, 1=LDO+
		vTaskDelay(configTICK_RATE_HZ/10);

		spi_write_array(std::array<uint8_t, 2>{0xA0, 0}); // symbol timeout
		vTaskDelay(configTICK_RATE_HZ/10);

		spi_write_array(std::array<uint8_t, 2>{0x9F, 1}); // timer stop on preamble?
		vTaskDelay(configTICK_RATE_HZ/10);

		spi_write_array(std::array<uint8_t, 3>{0x8E, 0, 4}); // SetTxParams(power, rampTime)
		vTaskDelay(configTICK_RATE_HZ/10);

		Get_Status();
		vTaskDelay(configTICK_RATE_HZ/10);

		Set_RfFrequency();
		vTaskDelay(configTICK_RATE_HZ/10);

		Set_PacketParams();
		vTaskDelay(configTICK_RATE_HZ/10);

		Get_Status();
		vTaskDelay(configTICK_RATE_HZ/10);

		CalibrateImage(866000000);
		Set_BufferBaseAddress(0, 0);
		Set_ModulationParams();
		Cfg_DioIrq(); // Enable RxDone and timeout interrupts by configuring IRQ
		spi_write_array(std::array<uint8_t, 5>{0x0D, 0x07, 0x40, 0x14, 0x24}); // Sync word
		Clr_IrqStatus(0xFFFF);
		Set_Rx();
	}

	void config_debug_pins()
	{
		bsp::pin_debug_subghzspi_nss.set(stm32_lib::gpio::mode_t::output, stm32_lib::gpio::af_t(SUBGHZSPI_DEBUG_AF));
		bsp::pin_debug_subghzspi_mosi.set(stm32_lib::gpio::mode_t::output, stm32_lib::gpio::af_t(SUBGHZSPI_DEBUG_AF));
		bsp::pin_debug_subghzspi_sck.set(stm32_lib::gpio::mode_t::output, stm32_lib::gpio::af_t(SUBGHZSPI_DEBUG_AF));
		bsp::pin_debug_subghzspi_miso.set(stm32_lib::gpio::mode_t::output, stm32_lib::gpio::af_t(SUBGHZSPI_DEBUG_AF));
	}
}


static const std::array<const char*, 16> flag_descr{
	"TxDone\r\n",
	"RxDone\r\n",
	"PreambleDetected\r\n",
	"SyncWordValid\r\n",
	"HeaderValid\r\n",
	"HeaderErr\r\n",
	"CrcErr\r\n",
	"CadDone\r\n",
	"CadDetected\r\n",
	"Timeout\r\n",
	"RFU10\r\n",
	"RFU11\r\n",
	"RFU12\r\n",
	"RFU13\r\n",
	"LrFhssHop\r\n",
	"RFU15\r\n"
};


freertos_utils::task_data_t<1024> task_data_lora_recv;

mailbox::mailbox_t<mailbox::string0> mb_string0;
mailbox::mailbox_t<mailbox::lora_packet> mb_lora_packet;

template <mailbox::mb_type Mt>
void mb_send(mailbox::mailbox_t<Mt>* mp)
{
	mailbox::set(reinterpret_cast<uint32_t>(&ipcc_mp), mp);
	// write mem barrier/flush write cache
	__DSB();
	stm32_lib::ipcc::c2_to_c1_send<ipcc_ch_lora>();
}

void task_function_lora_recv(void*)
{
	logger.log_async("CM0: LORA_RECV task started\r\n");
	vTaskDelay(configTICK_RATE_HZ/10);
	//subghz::config_debug_pins();

	subghz::init();
	logger.log_async("CM0: LORA_RECV subghz::init done\r\n");
	vTaskDelay(configTICK_RATE_HZ/10);

	subghz::init_lora_recv();
	logger.log_async("CM0: LORA_RECV subghz::init_lora done\r\n");

	subghz::log_status("CM0: ST before loop: ");

	logger.log_async("CM0: LORA_RECV loop\r\n");
	for(;;) {
		constexpr uint16_t irqmask_RxDone = (1 << 1);
		constexpr uint16_t irqmask_CrcErr = (1 << 6);
		//xTaskNotifyWait(0, 0xffffffff, nullptr, portMAX_DELAY);
		vTaskDelay(configTICK_RATE_HZ/3);
		auto const flags = subghz::Get_IrqStatus();

		if (!flags) {
			continue;
		}

		subghz::Clr_IrqStatus(flags); // clear all flags already set

		logger.log_async("CM0: RECV irqflags\r\n");
		{
			static std::array<char, 5> b;
			log_async_1(flags >> 8, b.data());
		}
		{
			static std::array<char, 5> b;
			log_async_1(flags & 0xFF, b.data());
		}

		logger.log_async("CM0: flags:\r\n");
		for (uint16_t i=0; i<16; ++i) {
			const uint16_t mask = (1 << i);
			if (flags & mask) {
				logger.log_async(flag_descr[i]);
			}
		}

		if (flags & irqmask_RxDone)
		{
			const bool crc_error = flags & irqmask_CrcErr;
			if (crc_error) {
				g_pin_red.pulse_once(configTICK_RATE_HZ*2);
				logger.log_async("CM0: LORA_RECV: recv crc error\r\n");
			}

			// show off
			//g_pin_blue.pulse_once(configTICK_RATE_HZ/2);
			logger.log_async("CM0: LORA_RECV: recv\r\n");

			static std::array<uint8_t, 256> rx_buf;
			auto const rx_size = subghz::Read_Fifo(rx_buf.data());
			{
				static char buf[5];
				logger.log_async("CM0: RX_SIZE\r\n");
				log_async_1(rx_size, buf);
			}

			uint8_t rssi_pkt, snr_pkt, signal_rssi_pkt;
			subghz::GetPacketStatus(&rssi_pkt, &snr_pkt, &signal_rssi_pkt);

			while (!stm32_lib::ipcc::c2_to_c1_tx_is_free<ipcc_ch_lora>()) {
				vTaskDelay(1);
			}
			mb_string0.s = "CM0->CM4 logging\r\n";
			mb_send(&mb_string0);

			while (!stm32_lib::ipcc::c2_to_c1_tx_is_free<ipcc_ch_lora>()) {
				vTaskDelay(1);
			}
			logger.log_async("CM0: sending\r\n");
			for (size_t i=0; i<rx_size; ++i) {
				mb_lora_packet.data[i] = rx_buf[i];
			}
			mb_lora_packet.data_len = rx_size;
			mb_lora_packet.rssi_pkt = -rssi_pkt >> 1;
			mb_lora_packet.snr_pkt = (int8_t(snr_pkt) + 2) >> 2;
			mb_lora_packet.signal_rssi_pkt = -signal_rssi_pkt >> 1;
			mb_send(&mb_lora_packet);

#if 0
			if (true || rx_size != 7) {
				logger.log_async(reinterpret_cast<const char*>(rx_buf.data()));
			}
#endif
		}
	}
}


void create_task_lora_recv()
{
	task_data_lora_recv.task_handle = xTaskCreateStatic(
		&task_function_lora_recv,
		"lora_recv",
		task_data_lora_recv.stack.size(),
		nullptr,
		PRIO_LORA_RECV,
		task_data_lora_recv.stack.data(),
		&task_data_lora_recv.task_buffer
	);
}


freertos_utils::task_data_t<1024> task_data_ipcc_send;

void task_function_ipcc_send(void*)
{
	logger.log_async("CM0: IPCC_SEND task started\r\n");
	constexpr uint8_t ch = 3;
	for (;;) {
		vTaskDelay(configTICK_RATE_HZ*5);
		if ((IPCC->C2TOC1SR & (1 << ch)) == 0) {
			logger.log_async("CM0: ipcc sending\r\n");
			IPCC->C2SCR = (1 << (ch+16));
		} else {
			logger.log_async("CM0: ipcc busy\r\n");
		}
	}
}

void create_task_ipcc_send()
{
	task_data_lora_recv.task_handle = xTaskCreateStatic(
		&task_function_ipcc_send,
		"ipcc_send_cm0",
		task_data_ipcc_send.stack.size(),
		nullptr,
		PRIO_IPCC_SEND,
		task_data_ipcc_send.stack.data(),
		&task_data_ipcc_send.task_buffer
	);
}


__attribute__ ((noreturn)) void main()
{
	for (volatile int i=0; i<100000; ++i) {}

	bus_init();
	logger.init_dma();

	logger.log_sync("\r\nLogger_cm0 initialized (sync)\r\n");

	//g_pin_blue.init_pin();
	g_pin_red.init_pin();
	g_pin_red.pulse_continuous(configTICK_RATE_HZ/30, configTICK_RATE_HZ/10);

	logger.log_sync("CM0: Creating LORA_RECV task...\r\n");
	create_task_lora_recv();
	logger.log_sync("CM0: Created LORA_RECV task\r\n");

	//create_task_ipcc_send();

	logger.log_sync("CM0: Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("CM0: Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}


void subghz::log_status(const char* msg)
{
	const auto st = Get_Status();
	logger.log_async(msg);
	static std::array<char, 16> buf;
	log_async_1(st, buf.data());
	vTaskDelay(configTICK_RATE_HZ/10);
}
