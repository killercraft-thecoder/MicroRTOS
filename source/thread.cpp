#include "thread.h"
#include "process.h" // for g_kernel
#include <stddef.h>
#include <stdint.h>
#include "core_cm4.h" // or core_cm3.h / core_cm7.h
#include "cmsis_gcc.h"
#include "include/process.h"
#include "mpu.h"

#define MAX_THREADS 8
#define TICK_HZ 1000U // 1 ms tick

// Cortex-M initial xPSR value: Thumb bit set
static const uint32_t INITIAL_XPSR = 0x01000000UL;

// -----------------------------------------------------------------------------
// Internal: Build initial stack frame for a new thread (PSP-based, unprivileged)
// -----------------------------------------------------------------------------
static void Init_Thread_Stack(Thread *t, void (*entry)(void *), void *arg)
{
    uint32_t *stackTop = (uint32_t *)(((uintptr_t)t->stackBase + t->stackSize) & ~0x7);

    // Hardware-stacked frame: R0-R3, R12, LR, PC, xPSR
    stackTop -= 8;
    stackTop[0] = (uint32_t)arg;   // R0
    stackTop[1] = 0x01010101;      // R1
    stackTop[2] = 0x02020202;      // R2
    stackTop[3] = 0x03030303;      // R3
    stackTop[4] = 0x12121212;      // R12
    stackTop[5] = 0xFFFFFFFD;      // LR = EXC_RETURN: return to Thread mode, use PSP
    stackTop[6] = (uint32_t)entry; // PC
    stackTop[7] = INITIAL_XPSR;    // xPSR (T-bit set)

    // Software-saved R4-R11
    stackTop -= 8;
    for (int i = 0; i < 8; i++)
    {
        stackTop[i] = 0;
    }

    t->psp = stackTop;

    // Optional bookkeeping
    t->context.R0 = (uint32_t)arg;
    t->context.R1 = 0x01010101;
    t->context.R2 = 0x02020202;
    t->context.R3 = 0x03030303;
    t->context.R12 = 0x12121212;
    t->context.SP = (uint32_t)stackTop;
    t->context.LR = 0xFFFFFFFD;
    t->stack_base = (uint32_t)t->stackBase;
}

// -----------------------------------------------------------------------------
// Low-level context helpers (Cortex-M, PSP). Save R4-R11 and PSP; restore them.
// -----------------------------------------------------------------------------
static inline void Save_Context(Thread *t)
{
    // Save callee-saved regs to current PSP
    register uint32_t *psp_reg __asm("r0");
    __asm volatile(
        "mrs   r0, psp           \n" // r0 = PSP
        "stmdb r0!, {r4-r11}     \n" // push R4-R11
        : "=r"(psp_reg)
        :
        : "memory");

    t->psp = psp_reg;
    t->context.SP = (uint32_t)psp_reg;

    // Save special registers (thread execution state)
    t->primask = __get_PRIMASK();
    t->basepri = __get_BASEPRI();
    t->faultmask = __get_FAULTMASK();
    t->control = __get_CONTROL();

#if defined(__FPU_PRESENT) && (__FPU_PRESENT == 1)
    // If lazily stacking FP context, also capture FP state when active.
    // Optionally detect FPCA bit in CONTROL before touching FP regs.
    // t->fpscr = __get_FPSCR(); // if it expose an intrinsic or inline asm
    __get_FPSCR
#endif
}

static inline void Restore_Context(Thread *t)
{
    // Restore interrupt mask state first (safe while still privileged/MSP)
    __set_PRIMASK(t->primask);
    __set_BASEPRI(t->basepri);
    __set_FAULTMASK(t->faultmask);

#if defined(__FPU_PRESENT) && (__FPU_PRESENT == 1)
    // Restore FP state here if it save's it (and FPCA is set for the thread)
    // __set_FPSCR(t->fpscr);
#endif

    // Restore R4–R11 and PSP
    register uint32_t *psp_reg __asm("r0") = t->psp;
    __asm volatile(
        "ldmia r0!, {r4-r11}     \n"
        "msr   psp, r0           \n"
        :
        : "r"(psp_reg)
        : "memory");

    // Restore CONTROL last for robustness (privilege + SPSEL + FPCA)
    __set_CONTROL(t->control);
    __ISB(); // ensure CONTROL takes effect before exception return
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

// Init the Skeduler , Then Start It.
void Init_Scheduler(void)
{
    g_kernel.threadCount = 0;      // Reset thread table
    g_kernel.currentIndex = 0;     // Reset scheduler index
    g_kernel.currentThread = NULL; // No thread is running yet

    MPU_Init();   // Configure MPU regions (privileged/unprivileged memory)
    MPU_Enable(); // Turn on MPU + fault generation

    Start_Scheduler(); // Enter the kernel's main scheduling loop
}

static inline void Yield(void)
{
    __asm volatile("svc %[imm]" ::[imm] "I"(SVC_YIELD) : "memory");
}

static inline void Thread_Exit(int code)
{
    register int r0 __asm("r0") = code;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_EXIT) : "memory");
}

static inline void Create_Thread(Thread *t, void (*entry)(void *), void *arg,
                                 uint32_t *stack, uint32_t stackBytes, uint32_t priority)
{
    register Thread *r0 __asm__("r0") = t;
    register void (*r1)(void *) __asm__("r1") = entry;
    register void *r2 __asm__("r2") = arg;
    register uint32_t *r3 __asm__("r3") = stack;
    __asm volatile(
        "push {r4}\n" // save r4 for stackBytes
        "mov r4, %[stackBytes]\n"
        "svc %[imm]\n"
        "pop {r4}\n"
        :
        : "r"(r0), "r"(r1), "r"(r2), "r"(r3),
          [stackBytes] "r"(stackBytes),
          [imm] "I"(SVC_CREATE_THREAD)
        : "memory");
}

void Kernal_Create_Thread(Thread *t, void (*entry)(void *), void *arg,
                          uint32_t *stack, uint32_t stackBytes, uint8_t priority)
{
    if (!t || !entry || !stack || g_kernel.threadCount >= MAX_THREADS)
    {
        return;
    }

    t->stackBase = stack;
    t->stackSize = stackBytes;
    t->priority = priority;
    t->usesFPU = 0;
    t->state = THREAD_READY;
    t->entry = entry;
    t->arg = arg;
    t->periodTicks = 10;
    t->nextReleaseTick = g_kernel.systemTicks + t->periodTicks;

    // Force unprivileged, PSP
    t->control = 0x03; // nPRIV=1, SPSEL=1

    Init_Thread_Stack(t, entry, arg);

    g_kernel.threadList[g_kernel.threadCount++] = t;
}

static void Init_SysTick(void)
{
    // Make sure SystemCoreClock is up to date
    SystemCoreClockUpdate();

    // Configure SysTick to interrupt at TICK_HZ
    if (SysTick_Config(SystemCoreClock / TICK_HZ))
    {
        // Reload value too large for SysTick
        while (1)
        {              /* error */
            __BKPT(10) // Error Code 10 (Unable to boot.)
        }
    }

    uint32_t lowest = (1UL << __NVIC_PRIO_BITS) - 1UL;
    NVIC_SetPriority(PendSV_IRQn, lowest);
    NVIC_SetPriority(SysTick_IRQn, lowest - 1U);
    NVIC_SetPriority(SVCall_IRQn, lowest - 2U);
}

extern "C"
{
    void SysTick_Handler(void)
    {
        Scheduler_Tick(); // 1ms tick
    }
}

// Start the scheduler: set current thread and pend PendSV to start via handler
void Start_Scheduler(void)
{
    if (g_kernel.threadCount == 0)
        return;

    // Set up SysTick for 1 ms ticks using CMSIS clock info
    Init_SysTick();

    // Pick first thread
    g_kernel.currentIndex = 0;
    g_kernel.currentThread = g_kernel.threadList[g_kernel.currentIndex];
    g_kernel.currentThread->state = THREAD_RUNNING;

    // Trigger a context switch path to start first thread properly
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;

    // Idle loop: sleep until next interrupt
    for (;;)
    {
        __WFI();
    }
}

void Kernal_Yield(void)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

void Kernel_Thread_Exit(int code)
{
    Thread *t = g_kernel.currentThread;
    t->exit_code = code; // store for debugging
    t->state = THREAD_TERMINATED;
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk; // trigger context switch
}

void Scheduler_Tick(void)
{
    uint32_t now = g_kernel.systemTicks++; // increment global tick counter

    for (uint8_t i = 0; i < g_kernel.threadCount; i++)
    {
        Thread *t = g_kernel.threadList[i];

        // Periodic releases (BLOCKED + period)
        if (t->state == THREAD_BLOCKED && t->periodTicks > 0)
        {
            if (now >= t->nextReleaseTick)
            {
                t->state = THREAD_READY;
                t->nextReleaseTick = now + t->periodTicks;
            }
        }

        // Sleep wakeups
        if (t->state == THREAD_SLEEPING)
        {
            if ((int32_t)(g_kernel.systemTicks - t->nextReleaseTick) >= 0)
            {
                t->state = THREAD_READY;
            }
        }
    }

    // Quantum countdown for current thread
    if (g_kernel.currentThread)
    {
        if (g_kernel.currentThread->timeSliceRemaining > 0)
        {
            g_kernel.currentThread->timeSliceRemaining--;
        }
        if (g_kernel.currentThread->timeSliceRemaining == 0)
        {
            SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
        }
    }
}

Thread *Scheduler_GetNextThread(void)
{
    if (g_kernel.threadCount == 0)
    {
        return NULL;
    }

    uint8_t highestPrio = 0;
    int8_t bestIndex = -1;

    // Find the highest-priority READY thread
    for (uint8_t i = 0; i < g_kernel.threadCount; i++)
    {
        Thread *t = g_kernel.threadList[i];
        if (t->state == THREAD_READY || t->state == THREAD_RUNNING)
        {
            if (bestIndex == -1 || t->priority > highestPrio)
            {
                highestPrio = t->priority;
                bestIndex = i;
            }
        }
    }

    if (bestIndex < 0)
    {
        return NULL; // no runnable threads
    }

    // If multiple threads share the same highest priority, round-robin them
    uint8_t startIndex = (uint8_t)((g_kernel.currentIndex + 1U) % g_kernel.threadCount);
    for (uint8_t offset = 0; offset < g_kernel.threadCount; offset++)
    {
        uint8_t idx = (startIndex + offset) % g_kernel.threadCount;
        Thread *t = g_kernel.threadList[idx];
        if ((t->state == THREAD_READY || t->state == THREAD_RUNNING) &&
            t->priority == highestPrio)
        {
            g_kernel.currentIndex = idx;
            return t;
        }
    }

    // Fallback: return the bestIndex found
    g_kernel.currentIndex = (uint8_t)bestIndex;
    return g_kernel.threadList[bestIndex];
}

// -----------------------------------------------------------------------------
// PendSV context switch handler glue
// -----------------------------------------------------------------------------
static inline Thread *pick_next_thread(void)
{
    return Scheduler_GetNextThread();
}

__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile(
        "mrs   r0, psp                 \n"
        "cbz   r0, .Lfirst_switch      \n"
        "push  {lr}                    \n"
        "bl    PendSV_Save             \n"
        "pop   {lr}                    \n"
        ".Lfirst_switch:               \n"
        "push  {lr}                    \n"
        "bl    PendSV_Restore          \n"
        "pop   {lr}                    \n"
        "bx    lr                      \n");
}

extern "C"
{

    void PendSV_Save(void)
    {
        if (g_kernel.currentThread)
        {
            Save_Context(g_kernel.currentThread);
            g_kernel.currentThread->state = THREAD_READY;
        }
    }

    void PendSV_Restore(void)
    {
        Thread *next = pick_next_thread();
        if (!next || next == NULL)
            return;

        g_kernel.currentThread = next;
        g_kernel.currentThread->state = THREAD_RUNNING;

        Restore_Context(g_kernel.currentThread);
        // Exception return will restore R0–R3, R12, LR, PC, xPSR from PSP.
    }
}
__attribute__((always_inline)) static inline void Thread_Sleep(time_t ms)
{
    register uint32_t r0 __asm__("r0") = ms;
    __asm volatile(
        "svc %[imm]\n"
        :
        : [imm] "I"(SVC_THREAD_SLEEP), "r"(r0)
        : "memory");
}

/**
 * @brief Public API to get the current system tick count.
 * @return uint32_t Current tick count.
 */
__attribute__((always_inline)) static inline time_t OS_GetTick(void)
{
    uint32_t result;
    __asm volatile(
        "svc %1       \n"
        "mov %0, r0   \n"
        : "=r"(result)
        : "I"(SVC_GET_TICK)
        : "r0");
    return result;
}

// Get the stacked frame depending on EXC_RETURN in LR
static inline uint32_t *get_stacked_frame(uint32_t lr)
{
    // Bit 2 of EXC_RETURN (LR) = 0 => MSP used, 1 => PSP used
    if (lr & 0x4U)
    {
        return (uint32_t *)__get_PSP();
    }
    else
    {
        return (uint32_t *)__get_MSP();
    }
}

static inline uint32_t Kernel_GetTick(void)
{
    return g_kernal->systemTicks; // Read Out that data.
}

extern "C"
{
    HAL_StatusTypeDef Kernel_SPI_Transmit(SPI_Args *args)
    {
        return HAL_SPI_Transmit(args->hspi, args->txData, args->size, args->timeout);
    }

    HAL_StatusTypeDef Kernel_SPI_Receive(SPI_Args *args)
    {
        return HAL_SPI_Receive(args->hspi, args->rxData, args->size, args->timeout);
    }

    HAL_StatusTypeDef Kernel_SPI_TransmitReceive(SPI_Args *args)
    {
        return HAL_SPI_TransmitReceive(args->hspi, args->txData, args->rxData, args->size, args->timeout);
    }

    // Extract SVC immediate from the SVC instruction at (PC - 2)
    static inline uint8_t read_svc_number(uint32_t stacked_pc)
    {
        uint16_t *svc_instr = (uint16_t *)(stacked_pc - 2U);
        return (uint8_t)(*svc_instr & 0xFFU);
    }

    // Naked SVC handler to preserve LR and access stacked frame
    __attribute__((naked)) void SVC_Handler(void)
    {
        __asm volatile(
            "tst lr, #4            \n" // which stack? set Z=1 if MSP
            "ite eq                \n"
            "mrseq r0, msp         \n" // r0 = stacked frame
            "mrsne r0, psp         \n"
            "mov   r1, lr          \n" // r1 = lr (EXC_RETURN)
            "b     SVC_Handler_C   \n");
    }

    // C part: r0 = stacked frame pointer, r1 = EXC_RETURN
    void SVC_Handler_C(uint32_t *frame, uint32_t lr)
    {
        uint32_t stacked_pc = frame[6];
        uint8_t svc_no = read_svc_number(stacked_pc);

        switch (svc_no)
        {
        case SVC_THREAD_SLEEP:
            Kernel_Thread_Sleep(frame[0]);
            break;
        case SVC_YIELD:
            Kernel_Yield();
            break;
        case SVC_CREATE_THREAD:
            Kernel_Create_Thread((Thread *)frame[0],
                                 (void (*)(void *))frame[1],
                                 (void *)frame[2],
                                 (uint32_t *)frame[3],
                                 *((uint32_t *)&frame[4]), // stackBytes
                                 *((uint32_t *)&frame[5])  // priority
            );
            break;
        case SVC_ADD_PROCESS:
            Kernel_Add_Process((Process *)frame[0]);
            break;
        case SVC_REMOVE_PROCESS:
            Kernel_Remove_Process((Process *)frame[0]);
            break;
        case SVC_EXIT:
            Kernal_Thread_Exit(frame[0]);
            break;
        case SVC_GPIO_WRITE:
            Kernel_GPIO_Write((GPIO_TypeDef *)frame[0], (uint16_t)frame[1], (GPIO_PinState)frame[2]);
            break;
        case SVC_GPIO_READ:
            frame[0] = Kernel_GPIO_Read((GPIO_TypeDef *)frame[0], (uint16_t)frame[1]);
            break;

        case SVC_UART_TRANSMIT:
            frame[0] = Kernel_UART_Transmit((UART_Args *)frame[0]);
            break;
        case SVC_UART_RECEIVE:
            frame[0] = Kernel_UART_Receive((UART_Args *)frame[0]);
            break;

        case SVC_I2C_MASTER_TXRX:
            frame[0] = Kernel_I2C_Master_TransmitReceive((I2C_Args *)frame[0]);
            break;
        case SVC_GET_TICK:
            frame[0] = Kernel_GetTick();
            break;
        case SVC_SPI_TRANSMIT:
            frame[0] = Kernel_SPI_Transmit((SPI_Args *)frame[0]);
            break;
        case SVC_SPI_RECEIVE:
            frame[0] = Kernel_SPI_Receive((SPI_Args *)frame[0]);
            break;
        case SVC_SPI_TRANSMITRECV:
            frame[0] = Kernel_SPI_TransmitReceive((SPI_Args *)frame[0]);
            break;


        default:
            break;
        }
    }
}

static void Kernel_Thread_Sleep(uint32_t ms)
{
    Thread *t = g_kernel.currentThread;

    // Set wake time
    t->periodTicks = ms; // optional: reuse as "requested delay"
    t->nextReleaseTick = g_kernel.systemTicks + ms;
    t->state = THREAD_SLEEPING;

    // Preempt so another thread can run
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

// GPIO
static inline void GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    register GPIO_TypeDef *r0 __asm__("r0") = port;
    register uint32_t r1 __asm__("r1") = pin;
    register uint32_t r2 __asm__("r2") = state;
    __asm volatile("svc %[imm]" ::[imm] "I"(SVC_GPIO_WRITE), "r"(r0), "r"(r1), "r"(r2) : "memory");
}

static inline GPIO_PinState GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    register GPIO_TypeDef *r0 __asm__("r0") = port;
    register uint32_t r1 __asm__("r1") = pin;
    register uint32_t ret __asm__("r0");
    __asm volatile("svc %[imm]\n" : "=r"(ret) : [imm] "I"(SVC_GPIO_READ), "r"(r0), "r"(r1) : "memory");
    return (GPIO_PinState)ret;
}

// UART
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *pData;
    uint16_t Size;
    uint32_t Timeout;
} UART_Args;

static inline HAL_StatusTypeDef UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    UART_Args args = {huart, pData, Size, Timeout};
    register UART_Args *r0 __asm__("r0") = &args;
    register HAL_StatusTypeDef ret __asm__("r0");
    __asm volatile("svc %[imm]\n" : "=r"(ret) : [imm] "I"(SVC_UART_TRANSMIT), "r"(r0) : "memory");
    return ret;
}

static inline HAL_StatusTypeDef UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    UART_Args args = {huart, pData, Size, Timeout};
    register UART_Args *r0 __asm__("r0") = &args;
    register HAL_StatusTypeDef ret __asm__("r0");
    __asm volatile("svc %[imm]\n" : "=r"(ret) : [imm] "I"(SVC_UART_RECEIVE), "r"(r0) : "memory");
    return ret;
}

// I2C
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t DevAddress;
    uint8_t *pTxData;
    uint16_t TxSize;
    uint8_t *pRxData;
    uint16_t RxSize;
    uint32_t Timeout;
} I2C_Args;

static inline HAL_StatusTypeDef I2C_Master_TransmitReceive(I2C_HandleTypeDef *hi2c,
                                                           uint16_t DevAddress,
                                                           uint8_t *pTxData, uint16_t TxSize,
                                                           uint8_t *pRxData, uint16_t RxSize,
                                                           uint32_t Timeout)
{
    I2C_Args args = {hi2c, DevAddress, pTxData, TxSize, pRxData, RxSize, Timeout};
    register I2C_Args *r0 __asm__("r0") = &args;
    register HAL_StatusTypeDef ret __asm__("r0");
    __asm volatile("svc %[imm]\n" : "=r"(ret) : [imm] "I"(SVC_I2C_MASTER_TXRX), "r"(r0) : "memory");
    return ret;
}

static void Kernel_GPIO_Write(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port == NULL)
        return; // invalid port
    if (state != GPIO_PIN_RESET && state != GPIO_PIN_SET)
        return; // invalid state
    // Optional: validate pin mask against MCU's valid pins
    HAL_GPIO_WritePin(port, pin, state);
}

static GPIO_PinState Kernel_GPIO_Read(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == NULL)
        return GPIO_PIN_RESET;
    // Optional: validate pin mask
    return HAL_GPIO_ReadPin(port, pin);
}

static HAL_StatusTypeDef Kernel_UART_Transmit(UART_Args *a)
{
    if (a == NULL || a->huart == NULL || a->pData == NULL)
        return HAL_ERROR;
    if (a->Size == 0)
        return HAL_OK; // nothing to send
    return HAL_UART_Transmit(a->huart, a->pData, a->Size, a->Timeout);
}

static HAL_StatusTypeDef Kernel_UART_Receive(UART_Args *a)
{
    if (a == NULL || a->huart == NULL || a->pData == NULL)
        return HAL_ERROR;
    if (a->Size == 0)
        return HAL_OK; // nothing to receive
    return HAL_UART_Receive(a->huart, a->pData, a->Size, a->Timeout);
}

static HAL_StatusTypeDef Kernel_I2C_Master_TransmitReceive(I2C_Args *a)
{
    if (a == NULL || a->hi2c == NULL)
        return HAL_ERROR;
    if ((a->TxSize > 0 && a->pTxData == NULL) ||
        (a->RxSize > 0 && a->pRxData == NULL))
        return HAL_ERROR;

    HAL_StatusTypeDef ret = HAL_OK;
    if (a->TxSize > 0)
    {
        ret = HAL_I2C_Master_Transmit(a->hi2c, a->DevAddress, a->pTxData, a->TxSize, a->Timeout);
        if (ret != HAL_OK)
            return ret;
    }
    if (a->RxSize > 0)
    {
        ret = HAL_I2C_Master_Receive(a->hi2c, a->DevAddress, a->pRxData, a->RxSize, a->Timeout);
    }
    return ret;
}