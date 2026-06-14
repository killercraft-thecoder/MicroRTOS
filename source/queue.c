#include "thread.h"
#include "queue.h"

API_FUNCTION(Queue_Create)
void Queue_Create(MessageQueue *q, void *buffer,
                  uint16_t msgSize, uint16_t capacity)
{
    Queue_CreateArgs args = {
        .q = q,
        .buffer = buffer,
        .msgSize = msgSize,
        .capacity = capacity};

    register Queue_CreateArgs *r0 __asm__("r0") = &args;
    __asm volatile("svc %0" ::"i"(SVC_QUEUE_CREATE), "r"(r0) : "memory");
}

API_FUNCTION(Queue_Send)
void Queue_Send(MessageQueue *q, const void *msg)
{
    register MessageQueue *r0 __asm__("r0") = q;
    register const void *r1 __asm__("r1") = msg;

    __asm volatile("svc %0" ::"i"(SVC_QUEUE_SEND), "r"(r0), "r"(r1) : "memory");
}

API_FUNCTION(Queue_Receive)
void Queue_Receive(MessageQueue *q, void *msgOut)
{
    register MessageQueue *r0 __asm__("r0") = q;
    register void *r1 __asm__("r1") = msgOut;

    __asm volatile("svc %0" ::"i"(SVC_QUEUE_RECEIVE), "r"(r0), "r"(r1) : "memory");
}

API_FUNCTION(Queue_TrySend)
QueueStatus Queue_TrySend(MessageQueue *q, const void *msg)
{
    register MessageQueue *r0 __asm__("r0") = q;
    register const void *r1 __asm__("r1") = msg;

    __asm volatile("svc %0" ::"i"(SVC_QUEUE_TRY_SEND), "r"(r0), "r"(r1) : "memory");

    QueueStatus result;
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}

API_FUNCTION(Queue_TryReceive)
QueueStatus Queue_TryReceive(MessageQueue *q, void *msgOut)
{
    register MessageQueue *r0 __asm__("r0") = q;
    register void *r1 __asm__("r1") = msgOut;

    __asm volatile("svc %0" ::"i"(SVC_QUEUE_TRY_RECEIVE), "r"(r0), "r"(r1) : "memory");

    QueueStatus result;
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}

