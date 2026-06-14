#include "thread.h"
#include "gpio.h"

API_FUNCTION(GPIO_WritePin)
// GPIO
void GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    register GPIO_TypeDef *r0 __asm__("r0") = port;
    register uint32_t r1 __asm__("r1") = pin;
    register uint32_t r2 __asm__("r2") = state;
    __asm volatile("svc %[imm]" ::[imm] "I"(SVC_GPIO_WRITE), "r"(r0), "r"(r1), "r"(r2) : "memory");
}

API_FUNCTION(GPIO_ReadPin)
GPIO_PinState GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    register GPIO_TypeDef *r0 __asm__("r0") = port;
    register uint32_t r1 __asm__("r1") = pin;
    register uint32_t ret __asm__("r0");
    __asm volatile("svc %[imm]\n" : "=r"(ret) : [imm] "I"(SVC_GPIO_READ), "r"(r0), "r"(r1) : "memory");
    return (GPIO_PinState)ret;
}

