#pragma once
// Minimal FreeRTOS-like API mapped to the kernel.
// Static-only: no heap. Keep binary small.

#include <stdint.h>
#include <stddef.h>

// ---- Types ----
typedef void (*TaskFunction_t)(void *);

// Your kernel's Thread and priority map
typedef struct Thread Thread;

typedef Thread* TaskHandle_t;
typedef uint32_t StackType_t;
typedef uint32_t UBaseType_t;
typedef int32_t  BaseType_t;

// ---- Configuration ----
#ifndef SHIM_MAX_TASK_NAME_LEN
#define SHIM_MAX_TASK_NAME_LEN 16
#endif

// ---- Kernel hooks it already has ----
#ifdef __cplusplus
extern "C" {
#endif

void Start_Scheduler(void); // your kernel entry
void Yield(void);           // SVC-backed
void Thread_Sleep(uint32_t ms); // SVC-backed

void Create_Thread(Thread *t, void (*entry)(void*), void *arg,
                   uint32_t *stack, uint32_t stackBytes, uint32_t priority);

void Thread_Exit(void);

#ifdef __cplusplus
}
#endif

// ---- FreeRTOS-like API ----

// vTaskStartScheduler -> Start_Scheduler
static inline void vTaskStartScheduler(void) {
    Start_Scheduler();
}

// vTaskDelay: ticks to ms (assume 1ms tick to keep it tiny)
#ifndef configTICK_RATE_HZ
#define configTICK_RATE_HZ 1000u
#endif

static inline void vTaskDelay(uint32_t ticks) {
    // Convert scheduler ticks to ms. For 1 kHz, this is a no-op.
    uint32_t ms = (ticks * 1000u) / configTICK_RATE_HZ;
    Thread_Sleep(ms);
}

// taskYIELD -> Yield
static inline void taskYIELD(void) {
    Yield();
}

// xTaskCreateStatic: user provides TCB and stack
typedef struct {
    // FreeRTOS keeps these, but we map minimally
    uint8_t dummy;
} StaticTask_t;

static inline TaskHandle_t xTaskCreateStatic(TaskFunction_t pxTaskCode,
                                             const char * const pcName,
                                             const uint32_t ulStackDepth, // in StackType_t units
                                             void *pvParameters,
                                             UBaseType_t uxPriority,
                                             StackType_t * const puxStackBuffer,
                                             StaticTask_t * const pxTaskBuffer) {
    (void)pcName; (void)pxTaskBuffer;

    // Convert FreeRTOS stack depth (words) to bytes
    uint32_t stackBytes = ulStackDepth * sizeof(StackType_t);

    // In this shim, caller supplies a Thread* via pxTaskBuffer or elsewhere.
    // To stay minimal, reuse the user-provided StaticTask_t as a holder for Thread.
    // Easiest small-footprint approach: require caller to pass a Thread* in pcName or pvParameters is ugly.
    // Cleaner: define a tiny struct that overlays StaticTask_t with a Thread.
    typedef struct { Thread tcb; } _ShimTaskStore;
    _ShimTaskStore *store = (_ShimTaskStore*)pxTaskBuffer;

    Create_Thread(&store->tcb, (void(*)(void*))pxTaskCode, pvParameters,
                  (uint32_t*)puxStackBuffer, stackBytes, (uint32_t)uxPriority);

    return (TaskHandle_t)&store->tcb;
}

// vTaskDelete: only support deleting self to keep it tiny
static inline void vTaskDelete(TaskHandle_t xTaskToDelete) {
    (void)xTaskToDelete; // only self-delete supported minimally
    Thread_Exit();
}