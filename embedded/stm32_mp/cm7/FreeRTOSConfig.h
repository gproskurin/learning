#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PORT_OPTIMISED_TASK_SELECTION		1
#define configUSE_TICKLESS_IDLE				1

#if defined TARGET_STM32H745_CM4
	#define configCPU_CLOCK_HZ			(64000000UL)
#elif defined TARGET_STM32H745_CM7
	#define configCPU_CLOCK_HZ			(64000000UL)
#endif

#define configMAX_PRIORITIES				4

#define configPRIO_BITS					(4U) // TODO from cmsis include
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY		15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5

#define configKERNEL_INTERRUPT_PRIORITY			(configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY		(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#include "FreeRTOSConfig-default.h"

#endif /* FREERTOS_CONFIG_H */
