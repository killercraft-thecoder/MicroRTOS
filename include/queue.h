#ifndef QUEUE_H_INCLUDED
#define QUEUE_H_INCLUDED

#include <stdint.h>
#include "thread.h"

API_FUNCTION(Queue_Create)
void Queue_Create(MessageQueue *q, void *buffer,
                  uint16_t msgSize, uint16_t capacity);

API_FUNCTION(Queue_Send)
void Queue_Send(MessageQueue *q, const void *msg);

API_FUNCTION(Queue_Receive)
void Queue_Receive(MessageQueue *q, void *msgOut);

API_FUNCTION(Queue_TrySend)
QueueStatus Queue_TrySend(MessageQueue *q, const void *msg);

API_FUNCTION(Queue_TryReceive)
QueueStatus Queue_TryReceive(MessageQueue *q, void *msgOut);

#endif /* QUEUE_H_INCLUDED */
