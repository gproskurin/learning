#include "sx1276.h"


namespace sx1276 {


const stm32_lib::gpio::gpio_pin_t pin_sxspi_nss(GPIOA, 15);
const stm32_lib::gpio::gpio_pin_t pin_sxspi_sck(GPIOB, 3);
const stm32_lib::gpio::gpio_pin_t pin_sxspi_miso(GPIOA, 6);
const stm32_lib::gpio::gpio_pin_t pin_sxspi_mosi(GPIOA, 7);

const std::array<stm32_lib::gpio::gpio_pin_t, 4> pins_dio{
	stm32_lib::gpio::gpio_pin_t(GPIOB, 4),
	stm32_lib::gpio::gpio_pin_t(GPIOB, 1),
	stm32_lib::gpio::gpio_pin_t(GPIOB, 0),
	stm32_lib::gpio::gpio_pin_t(GPIOC, 13)
};

const stm32_lib::gpio::gpio_pin_t pin_radio_reset(GPIOC, 0);
//const stm32_lib::gpio::gpio_pin_t pin_sx1276_reset(GPIOA, 11);

//const stm32_lib::gpio::gpio_pin_t pin_radio_tcxo_vcc(GPIOA, 12);
const stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_rx(GPIOA, 1);
const stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_boost(GPIOC, 1);
const stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_rfo(GPIOC, 2);


void init_radio_pin(const stm32_lib::gpio::gpio_pin_t& pin)
{
	pin.set(
		stm32_lib::gpio::mode_t::output,
		stm32_lib::gpio::otype_t::push_pull,
		stm32_lib::gpio::pupd_t::no_pupd,
		stm32_lib::gpio::speed_t::bits_11
	);
}


void spi_sx_init()
{
	stm32_lib::spi::init_pin_nss(pin_sxspi_nss);
	stm32_lib::spi::init_pins(
		pin_sxspi_mosi, SPI_SX1276_AF,
		pin_sxspi_miso, SPI_SX1276_AF,
		pin_sxspi_sck, SPI_SX1276_AF
	);

	// CPOL=0 CPHA=0, Motorola mode
	SPI_SX1276->CR1 = 0;
	constexpr uint32_t cr1 =
		(0b011 << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR_Msk
		| SPI_CR1_SSM_Msk
		;
	SPI_SX1276->CR1 = cr1;

	SPI_SX1276->CR2 = SPI_CR2_SSOE;

	SPI_SX1276->CR1 = cr1 | SPI_CR1_SPE;
}


} // namespace

