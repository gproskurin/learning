#if defined TARGET_STM32L072
#include <cmsis_device_l0/Include/stm32l072xx.h>
#elif defined TARGET_STM32WL55
#include <cmsis_device_wl/Include/stm32wl55xx.h>
#else
#error "Unsupported target"
#endif

