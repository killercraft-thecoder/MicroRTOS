#ifndef TIMER_H_INCLUDED
#define TIMER_H_INCLUDED

#include <stdint.h>
#include "thread.h"

API_FUNCTION(Timer_Create)
uint8_t Timer_Create(uint32_t ms);

API_FUNCTION(Timer_IsDone)
bool Timer_IsDone(uint8_t timerId);

API_FUNCTION(Timer_Reset)
bool Timer_Reset(uint8_t timerId, uint32_t ms);

API_FUNCTION(Timer_Cancel)
bool Timer_Cancel(uint8_t timerId);

API_FUNCTION(Timer_Remaining)
uint32_t Timer_Remaining(uint8_t timerId);

#endif /* TIMER_H_INCLUDED */
