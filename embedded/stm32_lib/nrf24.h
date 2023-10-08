#ifndef _my_nrf24_included_
#define _my_nrf24_included_

#include "lib_stm32.h"

#include <optional>


namespace nrf24 {


enum reg_t : uint8_t {
	CMD_R_REGISTER = 0b00000000,
	CMD_W_REGISTER = 0b00100000,
	CMD_NOP = 0b11111111,

	REG_CONFIG = 0x00,
	REGV_CONFIG_PWR_UP = 0b10,
	REGV_CONFIG_PRIM_RX_PRX = 0b1,
	//REGV_CONFIG_PRIM_RX_PTX = 0

	REG_FIFO_STATUS = 0x17
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
	for (volatile int i=0; i<10; ++i) {} // 2ns

	const uint8_t status = stm32_lib::spi::write<uint8_t>(hwc.spi, cmd);
	for (uint8_t i=0; i<size; ++i) {
		const uint8_t r = stm32_lib::spi::write<uint8_t>(hwc.spi, (mosi_buf ? mosi_buf[i] : 0));
		if (miso_buf) {
			miso_buf[i] = r;
		}
	}

	for (volatile int i=0; i<10; ++i) {} // 2ns
	stm32_lib::gpio::set_state(hwc.pin_spi_nss, 1);
	return status;
}


} // namespace

#endif

