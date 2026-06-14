#include "thread.h"
#include "mutex.h"

API_FUNCTION(Mutex_Create)
Mutex *Mutex_Create(Thread *maker)
{
    register Thread *r0 __asm__("r0") = maker;
    register Mutex *ret __asm__("r0");
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_MUTEX_CREATE) : "memory");
    return ret;
}

API_FUNCTION(Mutex_Lock)
void Mutex_Lock(Mutex *m)
{
    register mutex *r0 __asm__("r0") = m;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_MUTEX_LOCK) : "memory");
}

API_FUNCTION(Mutex_Unlock)
void Mutex_Unlock(Mutex *m)
{
    register mutex *r0 __asm__("r0") = m;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_MUTEX_UNLOCK) : "memory");
}

