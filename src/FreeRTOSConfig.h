
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "system_LPC17xx.h"

#define configUSE_NEWLIB_REENTRANT      1
#define configUSE_PREEMPTION            1
#define configUSE_IDLE_HOOK	            1
#define configUSE_TICK_HOOK             0
#define configUSE_TIMERS                1
#define configUSE_COUNTING_SEMAPHORES   1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 2   // 0: the second tick, 1: a block finished
#define configTIMER_TASK_STACK_DEPTH    192
#define configTIMER_QUEUE_LENGTH        16
#define configCPU_CLOCK_HZ              ( ( unsigned long ) SystemCoreClock )
#define configTICK_RATE_HZ              ( ( portTickType ) 1000 )
#define configMINIMAL_STACK_SIZE        ( ( unsigned short ) 40 )
#define configMAX_TASK_NAME_LEN	        ( 12 )
#define configUSE_TRACE_FACILITY        1
#define configIDLE_SHOULD_YIELD         0
#define configUSE_CO_ROUTINES           0
#define configUSE_MUTEXES               1
#define configUSE_RECURSIVE_MUTEXES     0
#define configCHECK_FOR_STACK_OVERFLOW  2

// without this every internal check is off, and a bad handle or index runs on until something
// unrelated crashes
#ifdef __cplusplus
extern "C"
#endif
void vAssertCalled(const char *file, int line);
#define configASSERT(x) if((x) == 0) vAssertCalled(__FILE__, __LINE__)
#define configTICK_TYPE_WIDTH_IN_BITS   TICK_TYPE_WIDTH_32_BITS
#define configSUPPORT_STATIC_ALLOCATION 1

#define configTIMER_TASK_PRIORITY       4
#define configMAX_PRIORITIES			( 5 )
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )
#define configQUEUE_REGISTRY_SIZE		0

#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           0
#define INCLUDE_vTaskDelete	                1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_uxTaskGetStackHighWaterMark	1
#define INCLUDE_xTaskGetSchedulerState      1

#define configKERNEL_INTERRUPT_PRIORITY 		( 7 << 5 )	/* Priority 7, or 255 as only the top three bits are implemented.  This is the lowest priority. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( 5 << 5 )  /* Priority 5, or 160 as only the top three bits are implemented. */

/*
 * Use the Cortex-M3 optimisations, rather than the generic C implementation.
 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

#endif /* FREERTOS_CONFIG_H */
