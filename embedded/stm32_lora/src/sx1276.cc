#include "sx1276.h"


namespace sx1276 {


const hwconf_t hwc = {
	.spi = SPI1,
	.spi_af = 0,

	.pin_spi_nss{GPIOA, 15},
	.pin_spi_sck{GPIOB, 3},
	.pin_spi_miso{GPIOA, 6},
	.pin_spi_mosi{GPIOA, 7},

	.pins_dio{
		stm32_lib::gpio::gpio_pin_t(GPIOB, 4),
		stm32_lib::gpio::gpio_pin_t(GPIOB, 1),
		stm32_lib::gpio::gpio_pin_t(GPIOB, 0),
		stm32_lib::gpio::gpio_pin_t(GPIOC, 13)
	},
	.pin_radio_reset{GPIOC, 0},

	//.pin_sx1276_reset{GPIOA, 11},

	//.pin_radio_tcxo_vcc{GPIOA, 12},
	.pin_radio_ant_sw_rx{GPIOA, 1},
	.pin_radio_ant_sw_tx_boost{GPIOC, 1},
	.pin_radio_ant_sw_tx_rfo{GPIOC, 2}

};


void init_radio_pin(const stm32_lib::gpio::gpio_pin_t& pin)
{
	pin.set(
		stm32_lib::gpio::mode_t::output,
		stm32_lib::gpio::otype_t::push_pull,
		stm32_lib::gpio::pupd_t::no_pupd,
		stm32_lib::gpio::speed_t::bits_11
	);
}


void spi_sx_init(const hwconf_t& hwc)
{
	stm32_lib::spi::init_pin_nss(hwc.pin_spi_nss);
	stm32_lib::spi::init_pins(
		hwc.pin_spi_mosi, hwc.spi_af,
		hwc.pin_spi_miso, hwc.spi_af,
		hwc.pin_spi_sck, hwc.spi_af
	);

	// CPOL=0 CPHA=0, Motorola mode
	hwc.spi->CR1 = 0;
	constexpr uint32_t cr1 =
		(0b011 << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR_Msk
		| SPI_CR1_SSM_Msk
		;
	hwc.spi->CR1 = cr1;

	hwc.spi->CR2 = SPI_CR2_SSOE;

	hwc.spi->CR1 = cr1 | SPI_CR1_SPE;
}


} // namespace

