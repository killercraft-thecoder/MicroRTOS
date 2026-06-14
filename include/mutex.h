#ifndef MUTEX_H_INCLUDED
#define MUTEX_H_INCLUDED

#include <stdint.h>
#include "thread.h"

API_FUNCTION(Mutex_Create)
Mutex *Mutex_Create(Thread *maker);

API_FUNCTION(Mutex_Lock)
void Mutex_Lock(Mutex *m);

API_FUNCTION(Mutex_Unlock)
void Mutex_Unlock(Mutex *m);

#endif /* MUTEX_H_INCLUDED */
