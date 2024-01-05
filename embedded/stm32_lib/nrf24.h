#ifndef _my_nrf24_included_
#define _my_nrf24_included_

#include "lib_stm32.h"

#include <optional>


namespace nrf24 {


enum reg_t : uint8_t {
	CMD_R_REGISTER = 0b00000000,
	CMD_W_REGISTER = 0b00100000,
	CMD_R_RX_PAYLOAD = 0b01100001,
	CMD_W_TX_PAYLOAD = 0b10100000,
	CMD_W_TX_PAYLOAD_NOACK = 0b10110000,
	CMD_FLUSH_TX = 0b11100001,
	CMD_FLUSH_RX = 0b11100010,
	CMD_R_RX_PL_WID = 0b01100000,
	CMD_NOP = 0b11111111,

	REG_CONFIG = 0x00,
	REGV_CONFIG_MASK_RX_DR = 1 << 6,
	REGV_CONFIG_MASK_TX_DS = 1 << 5,
	REGV_CONFIG_MASK_MAX_RT = 1 << 4,
	REGV_CONFIG_EN_CRC = 1 << 3,
	REGV_CONFIG_CRCO = 1 << 2,
	REGV_CONFIG_PWR_UP = 1 << 1,
	REGV_CONFIG_PRIM_RX_PRX = 1 << 0,

	REG_EN_AA = 0x01,

	REG_EN_RXADDR = 0x02,

	REG_SETUP_AW = 0x03,
	REGV_SETUP_AW_3BYTES = 0b01,
	REGV_SETUP_AW_4BYTES = 0b10,
	REGV_SETUP_AW_5BYTES = 0b11,

	REG_SETUP_RETR = 0x04,
	REGV_SETUP_RETR_ARD_Pos = 4,
	REGV_SETUP_RETR_ARC_Pos = 0,

	REG_RF_CH = 0x05,

	REG_RF_SETUP = 0x06,
	REGV_RF_SETUP_DR_LOW = 1 << 5,
	REGV_RF_SETUP_DR_HIGH = 1 << 3,
	REGV_RF_SETUP_RF_PWR_Msk = 0b11 << 1,
	REGV_RF_SETUP_RF_PWR_00 = 0b00 << 1,
	REGV_RF_SETUP_RF_PWR_01 = 0b01 << 1,
	REGV_RF_SETUP_RF_PWR_10 = 0b10 << 1,
	REGV_RF_SETUP_RF_PWR_11 = 0b11 << 1,

	REG_STATUS = 0x07,
	REGV_STATUS_RX_DR = 1 << 6,
	REGV_STATUS_MAX_RT = 1 << 4,
	REGV_STATUS_RX_P_NO_Pos = 1,
	REGV_STATUS_RX_P_NO_Msk = 0b111 << REGV_STATUS_RX_P_NO_Pos,
	REGV_STATUS_TX_FULL = 1 << 0,

	REG_OBSERVE_TX = 0x08,

	REG_RX_ADDR_P0 = 0x0A,
	REG_RX_ADDR_P1 = 0x0B,
	REG_RX_ADDR_P2 = 0x0C,
	REG_RX_ADDR_P3 = 0x0D,
	REG_RX_ADDR_P4 = 0x0E,
	REG_RX_ADDR_P5 = 0x0F,

	REG_TX_ADDR = 0x10,

	REG_RX_PW_P0 = 0x11,
	REG_RX_PW_P1 = 0x12,
	REG_RX_PW_P2 = 0x13,
	REG_RX_PW_P3 = 0x14,
	REG_RX_PW_P4 = 0x15,
	REG_RX_PW_P5 = 0x16,

	REG_FIFO_STATUS = 0x17,
	REGV_FIFO_STATUS_RX_EMPTY = 1 << 0,

	REG_FEATURE = 0x1D,
	REGV_FEATURE_EN_DPL = 1 << 2,
	REGV_FEATURE_EN_ACK_PAY = 1 << 1,
	REGV_FEATURE_EN_DYN_ACK = 1 << 0
};


struct hw_conf_t {
	SPI_TypeDef* const spi;
	const uint8_t spi_af;
	const stm32_lib::gpio::gpio_pin_t pin_spi_nss;
	const stm32_lib::gpio::gpio_pin_t pin_spi_sck;
	const stm32_lib::gpio::gpio_pin_t pin_spi_miso;
	const stm32_lib::gpio::gpio_pin_t pin_spi_mosi;

	const std::optional<stm32_lib::gpio::gpio_pin_t> pin_nrf_vcc;
	const stm32_lib::gpio::gpio_pin_t pin_irq;
	const stm32_lib::gpio::gpio_pin_t pin_ce;
};


uint8_t spi_write(
	const hw_conf_t& hwc,
	uint8_t size,
	uint8_t cmd,
	const uint8_t* const mosi_buf,
	uint8_t* const miso_buf
)
{
	stm32_lib::gpio::set_state(hwc.pin_spi_nss, 0);
	for (volatile int i=0; i<20; ++i) {} // Tcc (CSN to SCK Setup), 2ns

	const uint8_t status = stm32_lib::spi::write<uint8_t>(hwc.spi, cmd);
	for (uint8_t i=0; i<size; ++i) {
		const uint8_t r = stm32_lib::spi::write<uint8_t>(hwc.spi, (mosi_buf ? mosi_buf[i] : 0));
		if (miso_buf) {
			miso_buf[i] = r;
		}
	}

	for (volatile int i=0; i<20; ++i) {} // Tcch (SCK to CSN Hold), 2ns
	stm32_lib::gpio::set_state(hwc.pin_spi_nss, 1);
	return status;
}


uint8_t cmd0(const hw_conf_t& hwc, reg_t cmd)
{
	return spi_write(hwc, 0, cmd, nullptr, nullptr);
}


uint8_t reg_get(const hw_conf_t& hwc, uint8_t reg)
{
	uint8_t v = 0;
	nrf24::spi_write(
		hwc,
		1,
		nrf24::reg_t::CMD_R_REGISTER | reg,
		nullptr,
		&v
	);
	return v;
}


void reg_set(const hw_conf_t& hwc, uint8_t reg, const uint8_t val)
{
	nrf24::spi_write(
		hwc,
		1,
		nrf24::reg_t::CMD_W_REGISTER | reg,
		&val,
		nullptr
	);
}


inline void sleep_50ns()
{
	for (volatile int i=0; i<50; ++i) {} // TODO better
}


void flush_tx(const hw_conf_t& hwc)
{
	cmd0(hwc, reg_t::CMD_FLUSH_TX);
	sleep_50ns();
}


void flush_rx(const hw_conf_t& hwc)
{
	cmd0(hwc, reg_t::CMD_FLUSH_RX);
	sleep_50ns();
}


} // namespace

#endif

