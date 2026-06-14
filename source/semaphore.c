#include "thread.h"
#include "semaphore.h"

API_FUNCTION(Semaphore_Signal)
void Semaphore_Signal(Semaphore_T *s)
{
    register Semaphore_T *r0 __asm__("r0") = s;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_SEMAPHORE_SIGNAL) : "memory");
}

API_FUNCTION(Semaphore_Wait)
void Semaphore_Wait(Semaphore_T *s)
{
    register Semaphore_T *r0 __asm__("r0") = s;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_SEMAPHORE_WAIT) : "memory");
}

