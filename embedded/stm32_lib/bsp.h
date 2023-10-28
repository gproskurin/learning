#ifndef _my_bsp_h_included_
#define _my_bsp_h_included_

#ifdef TARGET_NRF52DK
#include "lib_nrf5.h"
#else
#include "lib_stm32.h"
#endif

namespace bsp {


#ifdef TARGET_STM32WB55

#define USART_STLINK USART1
#define USART_STLINK_PIN_TX_AF 7
constexpr stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx{GPIOB_BASE, 6};

constexpr stm32_lib::gpio::gpio_pin_t pin_led_blue{GPIOB_BASE, 5};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_green{GPIOB_BASE, 0};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_red{GPIOB_BASE, 1};
constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton1{GPIOC_BASE, 4};
constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton2{GPIOD_BASE, 0};
constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton3{GPIOD_BASE, 1};

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

#endif


#ifdef TARGET_STM32F103

// FIXME f103 doesn't have built-in stlink, check it
#define USART_STLINK USART1
#define USART_STLINK_PIN_TX_AF 4
constexpr stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx{GPIOA_BASE, 9};

constexpr stm32_lib::gpio::pin_inverted_t pin_led{GPIOB, 12};

#endif


#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7

// TODO use/check stlink pin
#define USART_STLINK USART3
#define USART_STLINK_PIN_TX_AF 7
constexpr stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx{GPIOB_BASE, 10};

constexpr stm32_lib::gpio::gpio_pin_t pin_led_red{GPIOI_BASE, 13};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_green{GPIOJ_BASE, 2};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_green_arduino{GPIOD_BASE, 3};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_green_vbus_usb_fs{GPIOA_BASE, 9};
constexpr stm32_lib::gpio::gpio_pin_t pin_led_red_otg_overcurrent{GPIOH_BASE, 11};

//constexpr stm32_lib::gpio::gpio_pin_t pin_userbutton{GPIOD_BASE, 1};

#endif


#if defined TARGET_NRF52DK
constexpr nrf5_lib::gpio::pin_inverted_t pin_led_1{17};
constexpr nrf5_lib::gpio::pin_inverted_t pin_led_2{18};
constexpr nrf5_lib::gpio::pin_inverted_t pin_led_3{19};
constexpr nrf5_lib::gpio::pin_inverted_t pin_led_4{20};
#endif


} // namespace

#endif

