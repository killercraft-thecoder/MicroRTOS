#include "thread.h"
#include "uart.h"

API_FUNCTION(UART_TRANSMIT)
HAL_StatusTypeDef UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    UART_Args args = {huart, pData, Size, Timeout};
    register UART_Args *r0 __asm__("r0") = &args;
    register HAL_StatusTypeDef ret __asm__("r0");
    __asm volatile("svc %[imm]\n" : "=r"(ret) : [imm] "I"(SVC_UART_TRANSMIT), "r"(r0) : "memory");
    return ret;
}

API_FUNCTION(UART_Receive)
HAL_StatusTypeDef UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    UART_Args args = {huart, pData, Size, Timeout};
    register UART_Args *r0 __asm__("r0") = &args;
    register HAL_StatusTypeDef ret __asm__("r0");
    __asm volatile("svc %[imm]\n" : "=r"(ret) : [imm] "I"(SVC_UART_RECEIVE), "r"(r0) : "memory");
    return ret;
}

