#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifdef TARGET_STM32L072
	#define configUSE_PORT_OPTIMISED_TASK_SELECTION		0
#else
	#define configUSE_PORT_OPTIMISED_TASK_SELECTION		1
	#define configUSE_TICKLESS_IDLE				1
#endif

#ifdef TARGET_STM32F103
	#define configCPU_CLOCK_HZ			8000000
#elif defined TARGET_STM32L072
	#define configCPU_CLOCK_HZ			2100000
#endif

#define configMAX_PRIORITIES	3

#include "FreeRTOSConfig-default.h"

#endif /* FREERTOS_CONFIG_H */
