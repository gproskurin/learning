#if defined TARGET_STM32L152
#include <cmsis_device_l1/Include/stm32l152xe.h>
#elif defined TARGET_STM32F103
#include <cmsis_device_f1/Include/stm32f103xb.h>
#else
#error "Unsupported target"
#endif

