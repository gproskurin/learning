#ifndef _my_sx1276_h_included_
#define _my_sx1276_h_included_

#include "stm32_lib/lib_stm32.h"

#include <array>

namespace sx1276 {

// SX1276 SPI
#define SPI_SX1276_AF 0
#define SPI_SX1276 SPI1
extern const stm32_lib::gpio::gpio_pin_t pin_sxspi_nss;
extern const stm32_lib::gpio::gpio_pin_t pin_sxspi_sck;
extern const stm32_lib::gpio::gpio_pin_t pin_sxspi_miso;
extern const stm32_lib::gpio::gpio_pin_t pin_sxspi_mosi;

extern const std::array<stm32_lib::gpio::gpio_pin_t, 4> pins_dio;

extern const stm32_lib::gpio::gpio_pin_t pin_radio_reset;
//extern const stm32_lib::gpio::gpio_pin_t pin_sx1276_reset;

//extern const stm32_lib::gpio::gpio_pin_t pin_radio_tcxo_vcc;
extern const stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_rx;
extern const stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_boost;
extern const stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_rfo;


enum regs_t : uint8_t {
	RegFifo = 0x00,
	RegOpMode = 0x01
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
	RegOpMode_Mode_CAD = 0b111
};


inline
void sleep_ns(int ns)
{
	for (volatile int i=0; i<(ns/10)+2; ++i) {} // TODO
}



void init_radio_pin(const stm32_lib::gpio::gpio_pin_t&);


void spi_sx_init();


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
		nss_0();
		stm32_lib::spi::write<uint8_t>(spi_, 0b10000000 | reg);
		const auto r = stm32_lib::spi::write<uint8_t>(spi_, val);
		nss_1();
		return r;
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

