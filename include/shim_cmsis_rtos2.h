#pragma once
// Minimal CMSIS-RTOS v2-like API mapped to this kernel.

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void Start_Scheduler(void);
void Yield(void);
void Thread_Sleep(uint32_t ms);
void Create_Thread(struct Thread *t, void (*entry)(void*), void *arg,
                   uint32_t *stack, uint32_t stackBytes, uint32_t priority);
void Thread_Exit(void);

#ifdef __cplusplus
}
#endif

// ---- Types ----
typedef void (*osThreadFunc_t)(void *argument);
typedef struct Thread* osThreadId_t;

typedef struct {
    const char   *name;
    uint32_t      attr_bits;
    void         *cb_mem;      // storage for Thread (required)
    uint32_t      cb_size;     // sizeof(Thread storage)
    void         *stack_mem;   // stack buffer (required)
    uint32_t      stack_size;  // bytes
    int32_t       priority;    // map directly
    uint32_t      tz_module;
    uint32_t      reserved;
} osThreadAttr_t;

// ---- Kernel control ----
static inline int32_t osKernelInitialize(void) { return 0; }
static inline int32_t osKernelStart(void) { Start_Scheduler(); return 0; }

// ---- Time ----
static inline int32_t osDelay(uint32_t ms) { Thread_Sleep(ms); return 0; }

// ---- Thread ----
static inline osThreadId_t osThreadNew(osThreadFunc_t func, void *argument,
                                       const osThreadAttr_t *attr) {
    if (!attr || !attr->cb_mem || !attr->stack_mem) return NULL;
    if (attr->cb_size < sizeof(struct Thread)) return NULL;

    struct Thread *t = (struct Thread*)attr->cb_mem;
    Create_Thread(t, (void(*)(void*))func, argument,
                  (uint32_t*)attr->stack_mem, attr->stack_size,
                  (uint32_t)((attr->priority < 0) ? 0 : attr->priority));
    return (osThreadId_t)t;
}

static inline void osThreadExit(void) {
    Thread_Exit();
}

// Yield (optional)
static inline void osThreadYield(void) { Yield(); }