#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configCPU_CLOCK_HZ	(64000000UL)
#define configMAX_PRIORITIES	5
#define configUSE_PORT_OPTIMISED_TASK_SELECTION		1
#define configUSE_TICKLESS_IDLE				0

#include "FreeRTOSConfig-default.h"

#endif /* FREERTOS_CONFIG_H */
