#ifndef _my_sx1276_h_included_
#define _my_sx1276_h_included_

#include "stm32_lib/lib_stm32.h"

#include <array>

namespace sx1276 {

struct hwconf_t {
	SPI_TypeDef* spi = nullptr;
	uint8_t spi_af; // TODO per spi pin

	stm32_lib::gpio::gpio_pin_t pin_spi_nss;
	stm32_lib::gpio::gpio_pin_t pin_spi_sck;
	stm32_lib::gpio::gpio_pin_t pin_spi_miso;
	stm32_lib::gpio::gpio_pin_t pin_spi_mosi;

	stm32_lib::gpio::gpio_pin_t pin_dio0;

	//stm32_lib::gpio::gpio_pin_t pin_radio_reset;
	//stm32_lib::gpio::gpio_pin_t pin_sx1276_reset;

	//stm32_lib::gpio::gpio_pin_t pin_radio_tcxo_vcc;
	//stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_rx;
	//stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_boost;
	//stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_rfo;
};


enum cf_t : uint8_t {
	// for RegModemConfig1
	bw_khz_7_8 = 0b0000,
	bw_khz_10_4 = 0b0001,
	bw_khz_15_6 = 0b0010,
	bw_khz_20_8 = 0b0011,
	bw_khz_31_25 = 0b0100,
	bw_khz_41_7 = 0b0101,
	bw_khz_62_5 = 0b0110,
	bw_khz_125 = 0b0111,
	bw_khz_250 = 0b1000,
	bw_khz_500 = 0b1001,
	bw_Msk = 0b11110000,
	bw_Pos = 4,

	// error coding rate
	ecr_4_5 = 0b001,
	ecr_4_6 = 0b010,
	ecr_4_7 = 0b011,
	ecr_4_8 = 0b100,
	ecr_Msk = 0b1110,
	ecr_Pos = 1,

	sf_Pos = 4,

	crc_Pos = 2,

	implicit_header_Pos = 0
};

struct lora_config_t {
	uint32_t freq;
	uint8_t sf;
	bool crc;
	cf_t bw;
	cf_t ecr;
	uint16_t preamble_length;
	bool implicit_header;
};

extern const hwconf_t hwc_emb;
extern const hwconf_t hwc_ext;


enum regs_t : uint8_t {
	Reg_Fifo = 0x00,
	Reg_OpMode = 0x01,
	Reg_FifoAddrPtr = 0x0D,
	Reg_FifoTxBaseAddr = 0x0E,
	Reg_FifoRxCurrentAddr = 0x10,
	Reg_IrqFlagsMask = 0x11,
	Reg_IrqFlags = 0x12,
	Reg_FifoRxBytesNb = 0x13,
	Reg_PayloadLength = 0x22,
	Reg_Version = 0x42,
};

enum reg_val_t : uint8_t {
	RegOpMode_LongRangeMode = 0b10000000,
	RegOpMode_AccessSharedReg = 0b01000000,
	RegOpMode_LowFrequencyModeOn = 0b00001000,
	RegOpMode_Mode_msk = 0b111,
	RegOpMode_Mode_pos = 0,
	RegOpMode_Mode_SLEEP = 0b000,
	RegOpMode_Mode_STDBY = 0b001,
	RegOpMode_Mode_FSTX = 0b010,
	RegOpMode_Mode_TX = 0b011,
	RegOpMode_Mode_FSRX = 0b100,
	RegOpMode_Mode_RXCONTINUOUS = 0b101,
	RegOpMode_Mode_RXSINGLE = 0b110,
	RegOpMode_Mode_CAD = 0b111,

	RegIrqFlags_RxDone = 1 << 6,
};


inline
void sleep_ns(int ns)
{
	for (volatile int i=0; i<(ns/10)+2; ++i) {} // TODO
}



void init_radio_pin(const stm32_lib::gpio::gpio_pin_t&);


void spi_sx_init(const hwconf_t&, bool fast);


class spi_sx1276_t {
	SPI_TypeDef* const spi_;
	const stm32_lib::gpio::gpio_pin_t pin_nss_;
public:
	spi_sx1276_t(SPI_TypeDef* spi, const stm32_lib::gpio::gpio_pin_t& pin_nss) : spi_(spi), pin_nss_(pin_nss) {}

	uint8_t get_reg(uint8_t reg)
	{
		nss_0();
		stm32_lib::spi::write<uint8_t>(spi_, reg);
		const auto r = stm32_lib::spi::write<uint8_t>(spi_, 0);
		nss_1();
		return r;
	}

	uint8_t set_reg(uint8_t reg, uint8_t val)
	{
		const std::array<uint8_t, 2> tx_buf{uint8_t(reg|0x80), val};
		std::array<uint8_t, 2> rx_buf;
		nss_0();
		stm32_lib::spi::write<uint8_t>(spi_, tx_buf.size(), tx_buf.data(), rx_buf.data());
		nss_1();
		return rx_buf[1];
	}

	void spi_write(const uint8_t* const tx_buf, size_t tx_size)
	{
		nss_0();
		stm32_lib::spi::write<uint8_t>(spi_, tx_size, tx_buf, nullptr);
		nss_1();
	}

	void send(const uint8_t* buf, size_t buf_size)
	{
		set_reg(0x01, (get_reg(0x01) & ~(0b111)) | 0b000); // mode: SLEEP
		set_reg(regs_t::Reg_PayloadLength, buf_size);
		set_reg(regs_t::Reg_FifoAddrPtr, 0x80);
		set_reg(regs_t::Reg_FifoTxBaseAddr, 0x80);
		set_reg(0x01, (get_reg(0x01) & ~(0b111)) | 0b001); // mode: STANDBY
		nss_0();
		stm32_lib::spi::write<uint8_t>(spi_, 0x80 | 0); // write to reg 0x00
		stm32_lib::spi::write<uint8_t>(spi_, buf_size, buf, nullptr);
		nss_1();
		set_reg(0x01, (get_reg(0x01) & ~(0b111)) | 0b011); // mode: TX
	}

	uint8_t recv(uint8_t* rx_buf)
	{
		const auto rx_size = get_reg(regs_t::Reg_FifoRxBytesNb);
		const auto fifo_addr = get_reg(regs_t::Reg_FifoRxCurrentAddr);
		set_reg(regs_t::Reg_FifoAddrPtr, fifo_addr);

		nss_0();
		stm32_lib::spi::write<uint8_t>(spi_, regs_t::Reg_Fifo); // read from fifo
		stm32_lib::spi::write<uint8_t>(spi_, rx_size, nullptr, rx_buf);
		nss_1();

		return rx_size;
	}

private:
	void nss_0()
	{
		stm32_lib::gpio::set_state(pin_nss_, 0);
		sleep_ns(30);
	}

	void nss_1()
	{
		sleep_ns(100);
		stm32_lib::gpio::set_state(pin_nss_, 1);
		sleep_ns(30);
	}
};


} // namespace

#endif

