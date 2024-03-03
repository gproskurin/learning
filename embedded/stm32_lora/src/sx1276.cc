#include "bsp.h"

#include "sx1276.h"


namespace sx1276 {


constexpr hwconf_t hwc_emb = {
	.spi = SPI1,
	.spi_af = 0,
	.pin_spi_nss = bsp::sx1276::pin_spi_nss,
	.pin_spi_sck = bsp::sx1276::pin_spi_sck,
	.pin_spi_miso = bsp::sx1276::pin_spi_miso,
	.pin_spi_mosi = bsp::sx1276::pin_spi_mosi,
	.pin_dio0 = bsp::sx1276::pin_dio0,
	//.pin_radio_reset{GPIOC_BASE, 0},
	//.pin_sx1276_reset{GPIOA_BASE, 11},
	//.pin_radio_tcxo_vcc{GPIOA_BASE, 12},
};


const hwconf_t hwc_ext = {
	.spi = SPI2,
	.spi_af = 0,
	.pin_spi_nss{GPIOB_BASE, 12},
	.pin_spi_sck{GPIOB_BASE, 13},
	.pin_spi_miso{GPIOB_BASE, 14},
	.pin_spi_mosi{GPIOB_BASE, 15},
	.pin_dio0{GPIOB_BASE, 4}, // FIXME, not used yet
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


void spi_sx_init(const hwconf_t& hwc, bool fast)
{
	stm32_lib::spi::init_pin_nss(hwc.pin_spi_nss);
	stm32_lib::spi::init_pins(
		hwc.pin_spi_mosi, hwc.spi_af,
		hwc.pin_spi_miso, hwc.spi_af,
		hwc.pin_spi_sck, hwc.spi_af
	);

	// CPOL=0 CPHA=0, Motorola mode
	hwc.spi->CR1 = 0;
	const uint32_t cr1 =
		((fast ? 0b001 : 0b111) << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR_Msk
		| SPI_CR1_SSM_Msk
		;
	hwc.spi->CR1 = cr1;

	hwc.spi->CR2 = SPI_CR2_SSOE;

	hwc.spi->CR1 = cr1 | SPI_CR1_SPE;
}


} // namespace

