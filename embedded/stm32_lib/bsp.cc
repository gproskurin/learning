#include "bsp.h"

namespace bsp {

#ifdef TARGET_STM32WB55
const stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx(GPIOB, 6);
const stm32_lib::gpio::gpio_pin_t pin_led_blue(GPIOB, 5);
const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 0);
const stm32_lib::gpio::gpio_pin_t pin_led_red(GPIOB, 1);
const stm32_lib::gpio::gpio_pin_t pin_userbutton1(GPIOC, 4);
const stm32_lib::gpio::gpio_pin_t pin_userbutton2(GPIOD, 0);
const stm32_lib::gpio::gpio_pin_t pin_userbutton3(GPIOD, 1);
#endif

#ifdef TARGET_STM32L072
const stm32_lib::gpio::gpio_pin_t usart_stlink_pin_tx(GPIOA, 2);
const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 5);
const stm32_lib::gpio::gpio_pin_t pin_led_blue(GPIOB, 6);
const stm32_lib::gpio::gpio_pin_t pin_led_red(GPIOB, 7);
const stm32_lib::gpio::gpio_pin_t pin_led_green2(GPIOA, 5);
#endif

}

