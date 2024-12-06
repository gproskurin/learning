#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#if defined TARGET_STM32WL55_CPU2
	#define configUSE_PORT_OPTIMISED_TASK_SELECTION		0
	#define configUSE_TICKLESS_IDLE				0
#endif

#if defined TARGET_STM32WL55_CPU2
#define configCPU_CLOCK_HZ    ( ( unsigned long ) 48000000 )
#endif

#define configTICK_RATE_HZ                         100
#define configUSE_PREEMPTION                       1
#define configUSE_TIME_SLICING                     0
#define configMAX_PRIORITIES                       4
#define configMINIMAL_STACK_SIZE                   128
#define configMAX_TASK_NAME_LEN                    16
#define configTICK_TYPE_WIDTH_IN_BITS              TICK_TYPE_WIDTH_32_BITS
#define configTASK_NOTIFICATION_ARRAY_ENTRIES      1
#define configENABLE_BACKWARD_COMPATIBILITY        0
#define configUSE_STREAM_BUFFERS    0

#define configSUPPORT_STATIC_ALLOCATION              1
#define configSUPPORT_DYNAMIC_ALLOCATION             0

#define configKERNEL_INTERRUPT_PRIORITY          0
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     0
#define configMAX_API_CALL_INTERRUPT_PRIORITY    0

#define configUSE_IDLE_HOOK                   1
#define configUSE_TICK_HOOK                   0
#define configUSE_MALLOC_FAILED_HOOK          0
#define configUSE_DAEMON_TASK_STARTUP_HOOK    0

#define configCHECK_FOR_STACK_OVERFLOW        0

#define configASSERT( x )         \
    if( ( x ) == 0 )              \
    {                             \
        taskDISABLE_INTERRUPTS(); \
        for( ; ; )                \
        ;                         \
    }

#define configPROTECTED_KERNEL_OBJECT_POOL_SIZE                   10

#define configSYSTEM_CALL_STACK_SIZE                              128

#define secureconfigMAX_SECURE_CONTEXTS        5

#define configENABLE_TRUSTZONE            0
#define configENABLE_MPU                  0
#define configENABLE_FPU                  0
#define configENABLE_MVE                  0

#define configCHECK_HANDLER_INSTALLATION    0

#define configUSE_TASK_NOTIFICATIONS           1
#define configUSE_MUTEXES                      0
#define configUSE_RECURSIVE_MUTEXES            0
#define configUSE_COUNTING_SEMAPHORES          0
#define configUSE_QUEUE_SETS                   0
#define configUSE_APPLICATION_TASK_TAG         0

#define configUSE_POSIX_ERRNO                  0

#define INCLUDE_vTaskPrioritySet               0
#define INCLUDE_uxTaskPriorityGet              0
#define INCLUDE_vTaskDelete                    0
#define INCLUDE_vTaskSuspend                   0
#define INCLUDE_xResumeFromISR                 0
#define INCLUDE_vTaskDelayUntil                0
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_xTaskGetSchedulerState         0
#define INCLUDE_xTaskGetCurrentTaskHandle      0
#define INCLUDE_uxTaskGetStackHighWaterMark    0
#define INCLUDE_xTaskGetIdleTaskHandle         0
#define INCLUDE_eTaskGetState                  0
#define INCLUDE_xEventGroupSetBitFromISR       0
#define INCLUDE_xTimerPendFunctionCall         0
#define INCLUDE_xTaskAbortDelay                0
#define INCLUDE_xTaskGetHandle                 0
#define INCLUDE_xTaskResumeFromISR             0

#endif /* FREERTOS_CONFIG_H */
