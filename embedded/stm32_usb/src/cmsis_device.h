#if defined(TARGET_STM32L072)
#include <cmsis_device_l0/Include/stm32l072xx.h>
#elif defined(TARGET_STM32H747_CM7) || defined(TARGET_STM32H747_CM4)
#include <cmsis_device_h7/Include/stm32h747xx.h>
#else
#error "Unsupported target"
#endif

