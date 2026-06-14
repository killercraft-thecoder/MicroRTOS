#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED

#include <stdint.h>
#include "thread.h"

API_FUNCTION(UART_TRANSMIT)
HAL_StatusTypeDef UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

API_FUNCTION(UART_Receive)
HAL_StatusTypeDef UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif /* UART_H_INCLUDED */
