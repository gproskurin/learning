#ifndef _my_bsp_h_included_
#define _my_bsp_h_included_

#if defined(TARGET_NRF52DK) || defined(TARGET_NRF5340DK_APP)
#include "lib_nrf5.h"
#elif defined(TARGET_RP2040)
#include "lib_rp2040.h"
#else
#include "lib_stm32.h"
#endif

namespace bsp {


#ifdef TARGET_RP2040
constexpr rp2040_lib::gpio::pin_t pin_led{25};
#endif


#ifdef TARGET_STM32WB55

#define USART_STLINK USART1
#define USART_STLINK_PIN_TX_AF 7
constexpr stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx{GPIOB_BASE, 6};

constexpr stm32_lib::gpio::gpio_pin_t pin_led_blue_dongle{GPIOA_BASE, 4};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_blue{GPIOB_BASE, 5};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_green{GPIOB_BASE, 0};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_red{GPIOB_BASE, 1};
constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton1{GPIOC_BASE, 4};
constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton2{GPIOD_BASE, 0};
constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton3{GPIOD_BASE, 1};

#endif


#ifdef TARGET_STM32WL55

#define USART_STLINK LPUART1
#define USART_STLINK_PIN_TX_AF 8
constexpr stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx{GPIOA_BASE, 2};
constexpr stm32_lib::gpio::gpio_pin_t usart_stlink_pin_rx{GPIOA_BASE, 3};

constexpr stm32_lib::gpio::gpio_pin_t pin_led_blue{GPIOB_BASE, 15};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_green{GPIOB_BASE, 9};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_red{GPIOB_BASE, 11};
constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton1{GPIOA_BASE, 0};
constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton2{GPIOA_BASE, 1};
constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton3{GPIOC_BASE, 6};

#define SUBGHZSPI_DEBUG_AF 13
constexpr stm32_lib::gpio::gpio_pin_t pin_debug_subghzspi_nss{GPIOA_BASE, 4};
constexpr stm32_lib::gpio::gpio_pin_t pin_debug_subghzspi_sck{GPIOA_BASE, 5};
constexpr stm32_lib::gpio::gpio_pin_t pin_debug_subghzspi_miso{GPIOA_BASE, 6};
constexpr stm32_lib::gpio::gpio_pin_t pin_debug_subghzspi_mosi{GPIOA_BASE, 7};

namespace sx1276 {
	constexpr stm32_lib::gpio::gpio_pin_t pin_dio0{GPIOB_BASE, 12};
	constexpr stm32_lib::gpio::gpio_pin_t pin_dio1{GPIOB_BASE, 13};
	constexpr stm32_lib::gpio::gpio_pin_t pin_dio2{GPIOB_BASE, 14};
	//constexpr stm32_lib::gpio::gpio_pin_t pin_reset{GPIOC_BASE, 0};

	//  Connection between STM32 and RF switch
	constexpr stm32_lib::gpio::gpio_pin_t pin_fe_ctrl1{GPIOC_BASE, 4};
	constexpr stm32_lib::gpio::gpio_pin_t pin_fe_ctrl2{GPIOC_BASE, 5};
	constexpr stm32_lib::gpio::gpio_pin_t pin_fe_ctrl3{GPIOC_BASE, 3};

	constexpr stm32_lib::gpio::gpio_pin_t pin_tcxo32_vdd{GPIOB_BASE, 0};

	//constexpr stm32_lib::gpio::gpio_pin_t pin_vctl1{GPIOA_BASE, 1};
	//constexpr stm32_lib::gpio::gpio_pin_t pin_vctl2{GPIOC_BASE, 2};
	//constexpr stm32_lib::gpio::gpio_pin_t pin_vctl3{GPIOC_BASE, 1};
	//constexpr stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_rx{pin_vctl1};
	//constexpr stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_boost{pin_vctl2};
	//constexpr stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_rfo{pin_vctl3};
} // namespace
#endif


#ifdef TARGET_STM32L072

#define USART_STLINK USART2
#define USART_STLINK_PIN_TX_AF 4
constexpr stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx{GPIOA_BASE, 2};

constexpr stm32_lib::gpio::gpio_pin_t pin_led_green{GPIOB_BASE, 5};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_blue{GPIOB_BASE, 6};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_red{GPIOB_BASE, 7};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_green2{GPIOA_BASE, 5};

constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton{GPIOB_BASE, 2};

constexpr stm32_lib::gpio::gpio_pin_t pin_usb_dm{GPIOA_BASE, 11};
constexpr stm32_lib::gpio::gpio_pin_t pin_usb_dp{GPIOA_BASE, 12};

namespace sx1276 {
	// Connection between STM32 and Semtech IC
	constexpr stm32_lib::gpio::gpio_pin_t pin_dio0{GPIOB_BASE, 4};
	constexpr stm32_lib::gpio::gpio_pin_t pin_dio1{GPIOB_BASE, 1};
	constexpr stm32_lib::gpio::gpio_pin_t pin_dio2{GPIOB_BASE, 0};
	constexpr stm32_lib::gpio::gpio_pin_t pin_dio3{GPIOC_BASE, 13};
	//constexpr stm32_lib::gpio::gpio_pin_t pin_dio4{GPIOA_BASE, 5};
	//constexpr stm32_lib::gpio::gpio_pin_t pin_dio5{GPIOA_BASE, 4};
	constexpr stm32_lib::gpio::gpio_pin_t pin_reset{GPIOC_BASE, 0};
	constexpr stm32_lib::gpio::gpio_pin_t pin_spi_sck{GPIOB_BASE, 3};
	constexpr stm32_lib::gpio::gpio_pin_t pin_spi_miso{GPIOA_BASE, 6};
	constexpr stm32_lib::gpio::gpio_pin_t pin_spi_mosi{GPIOA_BASE, 7};
	constexpr stm32_lib::gpio::gpio_pin_t pin_spi_nss{GPIOA_BASE, 15};

	//  Connection between STM32 and RF switch
	constexpr stm32_lib::gpio::gpio_pin_t pin_vctl1{GPIOA_BASE, 1};
	constexpr stm32_lib::gpio::gpio_pin_t pin_vctl2{GPIOC_BASE, 2};
	constexpr stm32_lib::gpio::gpio_pin_t pin_vctl3{GPIOC_BASE, 1};
	constexpr stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_rx{pin_vctl1};
	constexpr stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_boost{pin_vctl2};
	constexpr stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_rfo{pin_vctl3};
} // namespace

#endif


#ifdef TARGET_STM32F103

// FIXME f103 doesn't have built-in stlink, check it
#define USART_STLINK USART1
#define USART_STLINK_PIN_TX_AF 4
constexpr stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx{GPIOA_BASE, 9};

constexpr stm32_lib::gpio::pin_inverted_t pin_led{GPIOB_BASE, 12};

#endif


#if defined TARGET_STM32H747_CM4 || defined TARGET_STM32H747_CM7

// TODO use/check stlink pin
#define USART_STLINK USART3
#define USART_STLINK_PIN_TX_AF 7
constexpr stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx{GPIOB_BASE, 10};

constexpr stm32_lib::gpio::gpio_pin_t pin_btn{GPIOC_BASE, 13};

constexpr stm32_lib::gpio::gpio_pin_t pin_joy_sel{GPIOK_BASE, 2};
constexpr stm32_lib::gpio::gpio_pin_t pin_joy_down{GPIOK_BASE, 3};
constexpr stm32_lib::gpio::gpio_pin_t pin_joy_left{GPIOK_BASE, 4};
constexpr stm32_lib::gpio::gpio_pin_t pin_joy_right{GPIOK_BASE, 5};
constexpr stm32_lib::gpio::gpio_pin_t pin_joy_up{GPIOK_BASE, 6};

constexpr stm32_lib::gpio::gpio_pin_t pin_led_green{GPIOI_BASE, 12};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_orange{GPIOI_BASE, 13};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_red{GPIOI_BASE, 14};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_blue{GPIOI_BASE, 15};

constexpr stm32_lib::gpio::gpio_pin_t pin_usb_dm{GPIOA_BASE, 11};
constexpr stm32_lib::gpio::gpio_pin_t pin_usb_dp{GPIOA_BASE, 12};
#define USB_AF 10

//constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton{GPIOD_BASE, 1};

namespace lcd {
	constexpr stm32_lib::gpio::gpio_pin_t pin_disp{GPIOD_BASE, 7};
	constexpr stm32_lib::gpio::gpio_pin_t pin_int{GPIOG_BASE, 2};

	// i2c4
	constexpr uint8_t af_i2c = 4;
	constexpr stm32_lib::gpio::gpio_pin_t pin_scl{GPIOD_BASE, 12};
	constexpr stm32_lib::gpio::gpio_pin_t pin_sda{GPIOD_BASE, 13};

	constexpr uint8_t af = 14;

	constexpr stm32_lib::gpio::gpio_pin_t pin_de{GPIOK_BASE, 7};
	constexpr stm32_lib::gpio::gpio_pin_t pin_vsync{GPIOI_BASE, 9};
	constexpr stm32_lib::gpio::gpio_pin_t pin_hsync{GPIOI_BASE, 12};
	constexpr stm32_lib::gpio::gpio_pin_t pin_clk{GPIOI_BASE, 14};

	constexpr stm32_lib::gpio::gpio_pin_t pin_r0{GPIOI_BASE, 15};
	constexpr stm32_lib::gpio::gpio_pin_t pin_r1{GPIOJ_BASE, 0};
	constexpr stm32_lib::gpio::gpio_pin_t pin_r2{GPIOJ_BASE, 1};
	constexpr stm32_lib::gpio::gpio_pin_t pin_r3{GPIOH_BASE, 9};
	constexpr stm32_lib::gpio::gpio_pin_t pin_r4{GPIOJ_BASE, 3};
	constexpr stm32_lib::gpio::gpio_pin_t pin_r5{GPIOJ_BASE, 4};
	constexpr stm32_lib::gpio::gpio_pin_t pin_r6{GPIOJ_BASE, 5};
	constexpr stm32_lib::gpio::gpio_pin_t pin_r7{GPIOJ_BASE, 6};

	constexpr stm32_lib::gpio::gpio_pin_t pin_g0{GPIOJ_BASE, 7};
	constexpr stm32_lib::gpio::gpio_pin_t pin_g1{GPIOJ_BASE, 8};
	constexpr stm32_lib::gpio::gpio_pin_t pin_g2{GPIOJ_BASE, 9};
	constexpr stm32_lib::gpio::gpio_pin_t pin_g3{GPIOJ_BASE, 10};
	constexpr stm32_lib::gpio::gpio_pin_t pin_g4{GPIOJ_BASE, 11};
	constexpr stm32_lib::gpio::gpio_pin_t pin_g5{GPIOI_BASE, 0};
	constexpr stm32_lib::gpio::gpio_pin_t pin_g6{GPIOI_BASE, 1};
	constexpr stm32_lib::gpio::gpio_pin_t pin_g7{GPIOK_BASE, 2};

	constexpr stm32_lib::gpio::gpio_pin_t pin_b0{GPIOJ_BASE, 12};
	constexpr stm32_lib::gpio::gpio_pin_t pin_b1{GPIOJ_BASE, 13};
	constexpr stm32_lib::gpio::gpio_pin_t pin_b2{GPIOJ_BASE, 14};
	constexpr stm32_lib::gpio::gpio_pin_t pin_b3{GPIOJ_BASE, 15};
	constexpr stm32_lib::gpio::gpio_pin_t pin_b4{GPIOK_BASE, 3};
	constexpr stm32_lib::gpio::gpio_pin_t pin_b5{GPIOK_BASE, 4};
	constexpr stm32_lib::gpio::gpio_pin_t pin_b6{GPIOK_BASE, 5};
	constexpr stm32_lib::gpio::gpio_pin_t pin_b7{GPIOK_BASE, 6};
}

#endif


#if defined TARGET_NRF52DK
constexpr nrf5_lib::gpio::pin_t pin_button_1{NRF_P0_BASE, 13};
constexpr nrf5_lib::gpio::pin_t pin_button_2{NRF_P0_BASE, 14};
constexpr nrf5_lib::gpio::pin_t pin_button_3{NRF_P0_BASE, 15};
constexpr nrf5_lib::gpio::pin_t pin_button_4{NRF_P0_BASE, 16};

constexpr nrf5_lib::gpio::pin_inverted_t pin_led_1{NRF_P0_BASE, 17};
constexpr nrf5_lib::gpio::pin_inverted_t pin_led_2{NRF_P0_BASE, 18};
constexpr nrf5_lib::gpio::pin_inverted_t pin_led_3{NRF_P0_BASE, 19};
constexpr nrf5_lib::gpio::pin_inverted_t pin_led_4{NRF_P0_BASE, 20};

constexpr nrf5_lib::gpio::pin_t pin_reset{NRF_P0_BASE, 21};

#define UARTE_VCOM NRF_UARTE0
//constexpr nrf5_lib::gpio::pin_t pin_vcom_rts{NRF_P0_BASE, 5};
constexpr nrf5_lib::gpio::pin_t pin_vcom_txd{NRF_P0_BASE, 6};
//constexpr nrf5_lib::gpio::pin_t pin_vcom_cts{NRF_P0_BASE, 7};
//constexpr nrf5_lib::gpio::pin_t pin_vcom_rxd{NRF_P0_BASE, 8};
#endif

#if defined TARGET_NRF5340DK_APP
constexpr nrf5_lib::gpio::pin_t pin_button_1{NRF_P0_S_BASE, 23};
constexpr nrf5_lib::gpio::pin_t pin_button_2{NRF_P0_S_BASE, 24};
constexpr nrf5_lib::gpio::pin_t pin_button_3{NRF_P0_S_BASE, 8};
constexpr nrf5_lib::gpio::pin_t pin_button_4{NRF_P0_S_BASE, 9};

constexpr nrf5_lib::gpio::pin_inverted_t pin_led_1{NRF_P0_S_BASE, 28};
constexpr nrf5_lib::gpio::pin_inverted_t pin_led_2{NRF_P0_S_BASE, 29};
constexpr nrf5_lib::gpio::pin_inverted_t pin_led_3{NRF_P0_S_BASE, 30};
constexpr nrf5_lib::gpio::pin_inverted_t pin_led_4{NRF_P0_S_BASE, 31};

#define UARTE_VCOM0 NRF_UARTE0_S
constexpr nrf5_lib::gpio::pin_t pin_vcom0_txd{NRF_P1_S_BASE, 1};

#define UARTE_VCOM1 NRF_UARTE1_S
constexpr nrf5_lib::gpio::pin_t pin_vcom1_txd{NRF_P0_S_BASE, 20};

#endif


} // namespace

#endif

