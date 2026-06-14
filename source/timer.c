#include "thread.h"
#include "timer.h"

API_FUNCTION(Timer_Create)
uint8_t Timer_Create(uint32_t ms)
{
    register uint32_t r0 __asm__("r0") = ms;

    __asm volatile("svc %0" ::"i"(SVC_TIMER_CREATE), "r"(r0) : "memory");

    uint8_t id;
    __asm volatile("mov %0, r0" : "=r"(id));
    return id;
}

API_FUNCTION(Timer_IsDone)
bool Timer_IsDone(uint8_t timerId)
{
    register uint32_t r0 __asm__("r0") = timerId;

    __asm volatile("svc %0" ::"i"(SVC_TIMER_IS_DONE), "r"(r0) : "memory");

    bool done;
    __asm volatile("mov %0, r0" : "=r"(done));
    return done;
}

API_FUNCTION(Timer_Reset)
bool Timer_Reset(uint8_t timerId, uint32_t ms)
{
    register uint32_t r0 __asm__("r0") = timerId;
    register uint32_t r1 __asm__("r1") = ms;

    __asm volatile("svc %0" ::"i"(SVC_TIMER_RESET), "r"(r0), "r"(r1) : "memory");

    bool result;
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}

API_FUNCTION(Timer_Cancel)
bool Timer_Cancel(uint8_t timerId)
{
    register uint32_t r0 __asm__("r0") = timerId;

    __asm volatile("svc %0" ::"i"(SVC_TIMER_CANCEL), "r"(r0) : "memory");

    bool result;
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}

API_FUNCTION(Timer_Remaining)
uint32_t Timer_Remaining(uint8_t timerId)
{
    register uint32_t r0 __asm__("r0") = timerId;

    __asm volatile("svc %0" ::"i"(SVC_TIMER_REMAINING), "r"(r0) : "memory");

    uint32_t remaining;
    __asm volatile("mov %0, r0" : "=r"(remaining));
    return remaining;
}

