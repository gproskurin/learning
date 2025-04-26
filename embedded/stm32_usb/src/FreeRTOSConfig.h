#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#if defined TARGET_STM32L072
	#define configCPU_CLOCK_HZ	(16000000UL)
#endif
#if defined TARGET_STM32H747_CM4
	#define configCPU_CLOCK_HZ	(64000000UL)

	#define configUSE_PORT_OPTIMISED_TASK_SELECTION		1
	#define configUSE_TICKLESS_IDLE				1

	#define configPRIO_BITS					(4U)
	#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY		((1 << configPRIO_BITS) - 1)
	#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	2
	#define configKERNEL_INTERRUPT_PRIORITY			(configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
	#define configMAX_SYSCALL_INTERRUPT_PRIORITY		(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#endif

//#define configTICK_RATE_HZ		1000
#define configMAX_PRIORITIES		4

#define configUSE_IDLE_HOOK		0
#define configKERNEL_PROVIDED_STATIC_MEMORY	1

#include "FreeRTOSConfig-default.h"

#endif /* FREERTOS_CONFIG_H */
