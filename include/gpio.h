#ifndef GPIO_H_INCLUDED
#define GPIO_H_INCLUDED

#include <stdint.h>
#include "thread.h"

API_FUNCTION(GPIO_WritePin)
void GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);

API_FUNCTION(GPIO_ReadPin)
GPIO_PinState GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

#endif /* GPIO_H_INCLUDED */
