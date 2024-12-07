#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#if defined TARGET_STM32WL55_CPU2
#define configCPU_CLOCK_HZ    ( ( unsigned long ) 48000000 )
#endif

#define configMAX_PRIORITIES                       4

#include "FreeRTOSConfig-default.h"

#endif /* FREERTOS_CONFIG_H */
