#ifndef SEMAPHORE_H_INCLUDED
#define SEMAPHORE_H_INCLUDED

#include <stdint.h>
#include "thread.h"

API_FUNCTION(Semaphore_Signal)
void Semaphore_Signal(Semaphore_T *s);

API_FUNCTION(Semaphore_Wait)
void Semaphore_Wait(Semaphore_T *s);

#endif /* SEMAPHORE_H_INCLUDED */
