#ifndef _my_bsp_h_included_
#define _my_bsp_h_included_

#include "lib_stm32.h"

namespace bsp {


#ifdef TARGET_STM32WB55

#define USART_STLINK USART1
#define USART_STLINK_PIN_TX_AF 7
extern const stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx;

extern const stm32_lib::gpio::gpio_pin_t pin_led_blue;
extern const stm32_lib::gpio::gpio_pin_t pin_led_green;
extern const stm32_lib::gpio::gpio_pin_t pin_led_red;
extern const stm32_lib::gpio::gpio_pin_t pin_userbutton1;
extern const stm32_lib::gpio::gpio_pin_t pin_userbutton2;
extern const stm32_lib::gpio::gpio_pin_t pin_userbutton3;
#endif


#ifdef TARGET_STM32L072

#define USART_STLINK USART2
#define USART_STLINK_PIN_TX_AF 4
extern const stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx;

extern const stm32_lib::gpio::gpio_pin_t pin_led_green;
extern const stm32_lib::gpio::gpio_pin_t pin_led_blue;
extern const stm32_lib::gpio::gpio_pin_t pin_led_red;
extern const stm32_lib::gpio::gpio_pin_t pin_led_green2;
#endif
}


#endif

