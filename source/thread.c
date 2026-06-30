#include <thread.h>
#include <process.h>
#include <stddef.h>
#include <stdint.h>
#include <mpu.h>
#include "mutex.h"
#include "semaphore.h"
#include "gpio.h"
#include "uart.h"
#include "queue.h"
#include "timer.h"
#include "fs.h"

#define MAX_THREADS 8
#define TICK_HZ 1000U // 1 ms tick

// Cortex-M initial xPSR value: Thumb bit set
static const uint32_t INITIAL_XPSR = 0x01000000UL;

// -----------------------------------------------------------------------------
// Atomic section helpers for critical kernel updates
// Use these macros to protect short critical regions that update global
// kernel metadata (counts, bitmasks, lists). They disable interrupts and
// restore the previous PRIMASK on exit. TODO: Consider nesting support.
#ifndef ATOMIC_SECTION_BEGIN
#define ATOMIC_SECTION_BEGIN() do { uint32_t __prim = __get_PRIMASK(); __disable_irq();
#define ATOMIC_SECTION_END()   if (!__prim) __enable_irq(); } while (0)
#endif

// TODOs (high priority):
// - Add per-thread capability/capability-bitfield in `Thread` to restrict which
//   SVC calls unprivileged threads may invoke (prevent privilege escalation).
// - Add lightweight resource reservation fields to `Thread` (reserved bytes,
//   small kernel quotas) and a revocation watchdog to revoke after 5 minutes
//   of inactivity. Track access timestamps and revoke on timeout.
// - Add FPU save/restore (FPCA) handling in Save_Context/Restore_Context.
// - Add comprehensive pointer/range validation for stack buffers passed to
//   thread creation and all user-provided buffers used in HAL/syscall paths.

// Helper: check whether a thread has a capability bit set
static inline int Thread_HasCap(Thread *t, uint32_t cap)
{
    return (t && ((t->capabilities & cap) != 0));
}

// -----------------------------------------------------------------------------
// Internal: Build initial stack frame for a new thread (PSP-based, unprivileged)
// -----------------------------------------------------------------------------
static void Init_Thread_Stack(Thread *t, void (*entry)(void *), void *arg)
{
    uint32_t *stackTop = (uint32_t *)(((uintptr_t)t->stackBase + t->stackSize) & ~0x7);

    // TODO: validate the `t->stackBase` and `t->stackSize` values here to
    // ensure (uintptr_t)t->stackBase + t->stackSize does not overflow and the
    // range belongs to user-provided stack memory. Malicious user-provided
    // stack values can cause the kernel to write out-of-bounds when building
    // the initial stack frame.

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

    // bookkeeping
    t->context.R0 = (uint32_t)arg;
    t->context.R1 = 0x01010101;
    t->context.R2 = 0x02020202;
    t->context.R3 = 0x03030303;
    t->context.R4 = 0x04040404;
    t->context.R5 = 0x05050505;
    t->context.R6 = 0x06060606;
    t->context.R7 = 0x07070707;
    t->context.R8 = 0x08080808;
    t->context.R9 = 0x09090909;
    t->context.R10 = 0x10101010;
    t->context.R11 = 0x11111111;
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
    // If the thread uses the FPU (usesFPU) or the FP context active bit is
    // set in CONTROL, save a minimal FP context (s0-s15 + FPSCR) into the
    // thread's TCB. This avoids depending on lazy stacking to preserve
    // FP registers across context switches.
    if (t->usesFPU || (t->control & 0x04)) // CONTROL.FPCA == bit 2
    {
        /* Save S0-S15 into t->fpu_regs[0..15]. We intentionally save a
           limited subset (S0-S15) because the hardware lazy stacking for
           exceptions typically covers these registers; this reduces overhead
           compared to saving all 32 single-precision regs. */
        __asm volatile("vstmia %0, {s0-s15}" : : "r"(&t->fpu_regs[0]) : "memory");
        t->fpscr = __get_FPSCR();
    }
#endif
}

static inline void Restore_Context(Thread *t)
{
    // Restore interrupt mask state first (safe while still privileged/MSP)
    __set_PRIMASK(t->primask);
    __set_BASEPRI(t->basepri);
    __set_FAULTMASK(t->faultmask);

#if defined(__FPU_PRESENT) && (__FPU_PRESENT == 1)
    // Restore FP state here if it was saved in the TCB. Match the save above
    // by restoring S0-S15 and FPSCR when this thread had floating-point use.
    if (t->usesFPU || (t->control & 0x04))
    {
        __asm volatile("vldmia %0, {s0-s15}" : : "r"(&t->fpu_regs[0]) : "memory");
        __set_FPSCR(t->fpscr);
    }
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

API_FUNCTION(Yield)
void Yield(void)
{
    __asm volatile("svc %[imm]" ::[imm] "I"(SVC_YIELD) : "memory");
}
API_FUNCTION(Thread_Exit)
void Thread_Exit(status_t code)
{
    register int r0 __asm("r0") = code;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_EXIT) : "memory");
}

API_FUNCTION(Create_Thread)
void Create_Thread(Thread *t, void (*entry)(void *), void *arg,
                   uint32_t *stack, uint32_t stackBytes,
                   priority_t priority, char name[8])
{
    _ThreadPartialArgs extra;
    extra.stack = stack;
    // TODO: thread name handling is inconsistent (char[5] vs char[8]).
    // Use a canonical size and ensure NUL-termination here to avoid
    // non-terminated strings or buffer overruns when copying names from
    // user code into kernel structures.
    strncpy(extra.name, name, 5);
    extra.name[4] = '\0';

    register Thread *r0 __asm__("r0") = t;
    register void (*r1)(void *) __asm__("r1") = entry;
    register void *r2 __asm__("r2") = arg;
    register _ThreadPartialArgs *r3 __asm__("r3") = &extra;

    __asm volatile(
        "push {r4}\n"
        "mov r4, %[stackBytes]\n"
        "svc %[imm]\n"
        "pop {r4}\n"
        :
        : "r"(r0), "r"(r1), "r"(r2), "r"(r3),
          [stackBytes] "r"(stackBytes),
          [imm] "I"(SVC_CREATE_THREAD)
        : "memory");
}

API_FUNCTION(Mutex_Lock)
void Mutex_Lock(Mutex *m)
{
    register mutex *r0 __asm__("r0") = m;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_MUTEX_LOCK) : "memory");
}
API_FUNCTION(Mutex_Unlock)
void Mutex_Unlock(Mutex *m)
{
    register Mutex *r0 __asm__("r0") = m;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_MUTEX_UNLOCK) : "memory");
}
API_FUNCTION(Get_Semaphore)
Get_Mutex_Result Get_Semaphore(char name[5], int id)
{
    return Kernal_Get_Mutex(name, id);
}

API_FUNCTION(Semaphore_Signal)
void Semaphore_Signal(Semaphore_T *s)
{
    register Semaphore_T *r0 __asm__("r0") = s;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_SEMAPHORE_SIGNAL) : "memory");
}
API_FUNCTION(Semaphore_Wait)
void Semaphore_Wait(Semaphore_T *s)
{
    register Semaphore_T *r0 __asm__("r0") = s;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_SEMAPHORE_WAIT) : "memory");
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

// Start the scheduler: set current thread and pend PendSV to start via handler
void Start_Scheduler(void)
{
    if (g_kernel.threadCount == 0)
        return;

    // Set up SysTick for 1 ms ticks using CMSIS clock info
    Init_SysTick();

    // Initialize TLSF using linker-provided heap region
    TLSF_Init(&g_kernel.tlsf);

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

// -----------------------------------------------------------------------------
// PendSV context switch handler glue
// -----------------------------------------------------------------------------
static inline Thread *pick_next_thread(void)
{
    return Scheduler_GetNextThread();
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

void Scheduler_Tick(void)
{
    g_kernel.systemTicks++; // increment global tick counter

    for (uint8_t i = 0; i < g_kernel.threadCount; i++)
    {
        Thread *t = g_kernel.threadList[i];

        // Periodic releases (BLOCKED + period)
        if (t->state == THREAD_BLOCKED && t->periodTicks > 0)
        {
            if (g_kernel.systemTicks >= t->nextReleaseTick)
            {
                t->state = THREAD_READY;
                t->nextReleaseTick = g_kernel.systemTicks + t->periodTicks;
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

        // Semaphore waiting wakeups
        if (t->state == THREAD_BLOCKED_SEMAPHORE)
        {
            if (g_kernel.semaphoreList[t->semaphoreIndex].value > 0)
            {
                t->state = THREAD_READY;
                g_kernel.semaphoreList[t->semaphoreIndex].value--;
                t->semaphoreIndex = 0;
            }
        }

        // Queue receive wakeups
        if (t->state == THREAD_BLOCKED_QUEUE_RECV)
        {
            MessageQueue *q = &g_kernel.queueList[t->queueIndex];
            if (q->count > 0)
            {
                t->state = THREAD_READY;
                t->queueIndex = 0;
            }
        }

        // Queue send wakeups
        if (t->state == THREAD_BLOCKED_Queue_Send)
        {
            MessageQueue *q = &g_kernel.queueList[t->queueIndex];
            if (q->count < q->capacity)
            {
                t->state = THREAD_READY;
                t->queueIndex = 0;
            }
        }
        {
            MessageQueue *q = &g_kernel.queueList[t->queueIndex];
            if (q->count > 0)
            {
                t->state = THREAD_READY;
                t->queueIndex = 0;
            }
        }

        // Queue send wakeups
        if (t->state == THREAD_BLOCKED_Queue_Send)
        {
            MessageQueue *q = &g_kernel.queueList[t->queueIndex];
            if (q->count < q->capacity)
            {
                t->state = THREAD_READY;
                t->queueIndex = 0;
            }
        }

        for (int j = 0; j < MAX_TIMERS_PER_THREAD; j++)
        {
            SoftTimer *tm = &t->timers[j];

            if (tm->active && !tm->finished)
            {
                if (tm->ms_left > 0)
                {
                    tm->ms_left--;
                    if (tm->ms_left == 0)
                    {
                        tm->finished = true;
                    }
                }
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

KERNAL_FUNCTION
void Kernal_Wipe_Thread(Thread *t)
{
    if (!t)
        return;

    // Reset important registers and bookkeeping
    t->psp = 0;
    t->stackBase = 0;
    t->stackSize = 0;
    t->priority = 0;
    t->entry = 0;

    // Clear saved context and name safely
    memset(&t->context, 0, sizeof(t->context));
    memset(t->name, 0, sizeof(t->name));
    return;
}

KERNAL_FUNCTION
void Kernal_Yield(void)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}
KERNAL_FUNCTION
void Kernel_Thread_Exit(status_t code)
{
    Thread *t = g_kernel.currentThread;
    t->exit_code = code; // store for debugging
    t->state = THREAD_TERMINATED;
    Kernal_Wipe_Thread(t);
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk; // trigger context switch
}
KERNAL_FUNCTION
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
KERNAL_FUNCTION
void PendSV_Save(void)
{
    if (g_kernel.currentThread)
    {
        Save_Context(g_kernel.currentThread);
        g_kernel.currentThread->state = THREAD_READY;
    }
}
KERNAL_FUNCTION
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

__attribute__((always_inline)) static inline void Thread_Sleep(time_t ms)
{
    register uint32_t r0 __asm__("r0") = ms;
    __asm volatile(
        "svc %[imm]\n"
        :
        : [imm] "I"(SVC_THREAD_SLEEP), "r"(r0)
        : "r0", "memory");
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
        : "r0", "r1", "r2", "r3", "r12", "lr", "memory");
    return result;
}

static inline uint32_t Kernel_GetTick(void)
{
    return g_kernel->systemTicks; // Read Out that data.
}

KERNAL_FUNCTION
void Kernal_Create_Thread(Thread *t, void (*entry)(void *), void *arg,
                          uint32_t *stack, uint32_t stackBytes, status_t priority)
{
    if (!t || !entry || !stack || g_kernel.threadCount >= MAX_THREADS || stackBytes < 64)
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

    // TODO: validate `stack` pointer and `stackBytes` range here to avoid
    // stackTop overflow/underflow or maliciously crafted user stacks.
    ATOMIC_SECTION_BEGIN();
    g_kernel.threadList[g_kernel.threadCount] = t;
    g_kernel.threadCount++;
    ATOMIC_SECTION_END();
}

static void enable_uart_clock(USART_TypeDef *i)
{
#ifdef __HAL_RCC_USART1_CLK_ENABLE
    if (i == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        return;
    }
#endif
#ifdef __HAL_RCC_USART2_CLK_ENABLE
    if (i == USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();
        return;
    }
#endif
#ifdef __HAL_RCC_USART3_CLK_ENABLE
    if (i == USART3)
    {
        __HAL_RCC_USART3_CLK_ENABLE();
        return;
    }
#endif
#ifdef __HAL_RCC_USART6_CLK_ENABLE
    if (i == USART6)
    {
        __HAL_RCC_USART6_CLK_ENABLE();
        return;
    }
#endif
}
static void enable_i2c_clock(I2C_TypeDef *i)
{
#ifdef __HAL_RCC_I2C1_CLK_ENABLE
    if (i == I2C1)
    {
        __HAL_RCC_I2C1_CLK_ENABLE();
        return;
    }
#endif
#ifdef __HAL_RCC_I2C2_CLK_ENABLE
    if (i == I2C2)
    {
        __HAL_RCC_I2C2_CLK_ENABLE();
        return;
    }
#endif
#ifdef __HAL_RCC_I2C3_CLK_ENABLE
    if (i == I2C3)
    {
        __HAL_RCC_I2C3_CLK_ENABLE();
        return;
    }
#endif
}
static void enable_spi_clock(SPI_TypeDef *i)
{
#ifdef __HAL_RCC_SPI1_CLK_ENABLE
    if (i == SPI1)
    {
        __HAL_RCC_SPI1_CLK_ENABLE();
        return;
    }
#endif
#ifdef __HAL_RCC_SPI2_CLK_ENABLE
    if (i == SPI2)
    {
        __HAL_RCC_SPI2_CLK_ENABLE();
        return;
    }
#endif
#ifdef __HAL_RCC_SPI3_CLK_ENABLE
    if (i == SPI3)
    {
        __HAL_RCC_SPI3_CLK_ENABLE();
        return;
    }
#endif
#ifdef __HAL_RCC_SPI4_CLK_ENABLE
    if (i == SPI4)
    {
        __HAL_RCC_SPI4_CLK_ENABLE();
        return;
    }
#endif
#ifdef __HAL_RCC_SPI5_CLK_ENABLE
    if (i == SPI5)
    {
        __HAL_RCC_SPI5_CLK_ENABLE();
        return;
    }
#endif
#ifdef __HAL_RCC_SPI6_CLK_ENABLE
    if (i == SPI6)
    {
        __HAL_RCC_SPI6_CLK_ENABLE();
        return;
    }
#endif
}

KERNAL_FUNCTION
static HAL_StatusTypeDef Kernel_GPIO_New(const GPIO_NewArgs *a)
{
    if (!a || !a->port)
        return HAL_ERROR;
    GPIO_InitTypeDef init = {.Pin = a->pin, .Mode = a->mode, .Pull = a->pull, .Speed = a->speed};
    HAL_GPIO_Init(a->port, &init);
    return HAL_OK;
}

KERNAL_FUNCTION
static HAL_StatusTypeDef Kernel_UART_New(const UART_NewArgs *a)
{
    if (!a || !a->huart || !a->instance)
        return HAL_ERROR;
    enable_uart_clock(a->instance);

    memset(a->huart, 0, sizeof(*a->huart));
    a->huart->Instance = a->instance;
    a->huart->Init.BaudRate = a->baudrate;
    a->huart->Init.WordLength = a->wordLength;
    a->huart->Init.StopBits = a->stopBits;
    a->huart->Init.Parity = a->parity;
    a->huart->Init.Mode = a->mode;
    a->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    a->huart->Init.OverSampling = UART_OVERSAMPLING_16;

    return HAL_UART_Init(a->huart);
}

KERNAL_FUNCTION
static HAL_StatusTypeDef Kernel_I2C_New(const I2C_NewArgs *a)
{
    if (!a || !a->hi2c || !a->instance)
        return HAL_ERROR;
    enable_i2c_clock(a->instance);

    memset(a->hi2c, 0, sizeof(*a->hi2c));
    a->hi2c->Instance = a->instance;
    a->hi2c->Init.Timing = a->timing;
    a->hi2c->Init.AddressingMode = a->addressingMode;
    a->hi2c->Init.OwnAddress1 = a->ownAddress;
    a->hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    a->hi2c->Init.OwnAddress2 = 0;
    a->hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    a->hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    return HAL_I2C_Init(a->hi2c);
}

KERNAL_FUNCTION
static HAL_StatusTypeDef Kernel_SPI_New(const SPI_NewArgs *a)
{
    if (!a || !a->hspi || !a->instance)
        return HAL_ERROR;
    enable_spi_clock(a->instance);

    memset(a->hspi, 0, sizeof(*a->hspi));
    a->hspi->Instance = a->instance;
    a->hspi->Init.Mode = a->mode;
    a->hspi->Init.Direction = a->direction;
    a->hspi->Init.DataSize = a->datasize;
    a->hspi->Init.CLKPolarity = a->clkpolarity;
    a->hspi->Init.CLKPhase = a->clkphase;
    a->hspi->Init.NSS = a->nss;
    a->hspi->Init.BaudRatePrescaler = a->baudratePrescaler;
    a->hspi->Init.FirstBit = a->firstBit;
    a->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    a->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    a->hspi->Init.CRCPolynomial = 7;

    return HAL_SPI_Init(a->hspi);
}

KERNAL_FUNCTION
HAL_StatusTypeDef Kernel_SPI_Transmit(SPI_Args *args)
{
    return HAL_SPI_Transmit(args->hspi, args->txData, args->size, args->timeout);
}
KERNAL_FUNCTION
HAL_StatusTypeDef Kernel_SPI_Receive(SPI_Args *args)
{
    return HAL_SPI_Receive(args->hspi, args->rxData, args->size, args->timeout);
}
KERNAL_FUNCTION
HAL_StatusTypeDef Kernel_SPI_TransmitReceive(SPI_Args *args)
{
    return HAL_SPI_TransmitReceive(args->hspi, args->txData, args->rxData, args->size, args->timeout);
}
KERNAL_FUNCTION
static void Kernel_GPIO_Write(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port == NULL)
        return; // invalid port
    if (state != GPIO_PIN_RESET && state != GPIO_PIN_SET)
        return; // invalid state
    // Optional: validate pin mask against MCU's valid pins
    HAL_GPIO_WritePin(port, pin, state);
}
KERNAL_FUNCTION
static GPIO_PinState Kernel_GPIO_Read(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == NULL)
        return GPIO_PIN_RESET;
    // Optional: validate pin mask
    return HAL_GPIO_ReadPin(port, pin);
}
KERNAL_FUNCTION
static HAL_StatusTypeDef Kernel_UART_Transmit(UART_Args *a)
{
    if (a == NULL || a->huart == NULL || a->pData == NULL)
        return HAL_ERROR;
    if (a->Size == 0)
        return HAL_OK; // nothing to send
    return HAL_UART_Transmit(a->huart, a->pData, a->Size, a->Timeout);
}
KERNAL_FUNCTION
static HAL_StatusTypeDef Kernel_UART_Receive(UART_Args *a)
{
    if (a == NULL || a->huart == NULL || a->pData == NULL)
        return HAL_ERROR;
    if (a->Size == 0)
        return HAL_OK; // nothing to receive
    return HAL_UART_Receive(a->huart, a->pData, a->Size, a->Timeout);
}
KERNAL_FUNCTION
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

KERNAL_FUNCTION
Mutex *Kernal_Create_Mutex(Thread *maker)
{
    if (!maker || g_kernel.mutexCount >= MAX_MUTEXES)
        return NULL;

    Mutex *m = &g_kernel.mutexes[g_kernel.mutexCount++];

    ATOMIC_SECTION_BEGIN();
    Mutex *m = &g_kernel.mutexes[g_kernel.mutexCount];
    g_kernel.mutexCount++;
    ATOMIC_SECTION_END();

    m->locked = false;
    m->owner = NULL;

    // TODO: consider protecting maker->ownedMutexes with atomic section or
    // per-thread lock if multiple threads can manipulate another thread's
    // bookkeeping concurrently.
    if (maker->ownedCount < 4)
    {
        maker->ownedMutexes[maker->ownedCount++] = m;
    }

    return m;
}

void Kernal_Lock_Mutex(Mutex *m)
{
    if (!m->locked)
    {
        m->locked = true;
        m->owner = g_kernel.currentThread;
    }
}

void Kernal_Unlock_Mutex(Mutex *m)
{
    if (m->owner == g_kernel.currentThread)
    {
        m->locked = false;
        m->owner = NULL;
    }
}

void *Kernal_Malloc(size_t size)
{
    if (size == 0)
    {
        debug_printf("[KERNEL] Malloc(0) ignored\n");
        return NULL;
    }

    void *ptr = TLSF_Malloc(&g_kernel.tlsf, size);

    if (!ptr)
    {
        debug_printf("[KERNEL] Malloc failed: size=%u\n", (unsigned)size);
    }
    return ptr;
}

void Kernal_Free(void *ptr)
{
    if (!ptr)
    {
        debug_printf("[KERNEL] Free(NULL) ignored\n");
        return;
    }

    // Prevent freeing non-heap pointers coming from user code which could
    // corrupt TLSF metadata. TODO: strengthen to check alignment/allocator
    // metadata if TLSF provides an API for validation.
    if (!Kernal_IsValidPointer(ptr))
    {
        debug_printf("[KERNEL] Free(invalid) ignored: %p\n", ptr);
        return;
    }

    TLSF_Free(&g_kernel.tlsf, ptr);
}

void *Kernal_Calloc(size_t n, size_t size)
{
    if (n == 0 || size == 0)
    {
        debug_printf("[KERNEL] Calloc(%u, %u) ignored\n",
                     (unsigned)n, (unsigned)size);
        return NULL;
    }


    void *ptr = TLSF_Calloc(&g_kernel.tlsf, n, size);

    if (!ptr)
    {
        debug_printf("[KERNEL] Calloc failed: n=%u size=%u\n",
                     (unsigned)n, (unsigned)size);
    }
    return ptr;
}


void *Kernal_Realloc(void *ptr, size_t newSize)
{
    if (!ptr)
        return Kernal_Malloc(newSize);

    if (newSize == 0)
    {
        Kernal_Free(ptr);
        return NULL;
    }


    void *newPtr = TLSF_Realloc(&g_kernel.tlsf, ptr, newSize);

    if (!newPtr)
    {
        debug_printf("[KERNEL] Realloc failed: ptr=%p newSize=%u\n",
                     ptr, (unsigned)newSize);
    }
    return newPtr;
}



static inline Thread *findThreadByName(const char *target)
{
    // Only iterate active threads to avoid reading uninitialized entries.
    for (int i = 0; i < (int)g_kernel.threadCount && i < MAX_THREADS; i++)
    {
        // TODO: thread name length is inconsistent across code (5 vs 8).
        // Ensure `Thread::name` has a single canonical size and is NUL-terminated.
        if (strncmp(g_kernel.threadList[i].name, target, 5) == 0)
        {
            return &g_kernel.threadList[i]; // Found
        }
    }
    return NULL; // Not found
}

KERNAL_FUNCTION
Get_Mutex_Result Kernal_Get_Mutex(char name[5], int id)
{
    Thread *t = findThreadByName(name);
    if (t && id >= 0 && id < 4)
    {
        Mutex *m = t->ownedMutexes[id];
        if (m != NULL)
        {
            // TODO: map `m->locked` explicitly to `Get_Mutex_Result` values
            // instead of casting a bool to the enum; enum values may change
            // and casting is brittle.
            return (Get_Mutex_Result)m->locked;
        }
    }
    return FAIL; // thread not found, invalid id, or no mutex
}

// Find a free semaphore slot in the bitmask.
// Returns index [0..MAX_SEMAPHORES-1] if found, -1 if none free.
static inline int Semaphore_Alloc(uint32_t *bitmap)
{
    for (int i = 0; i < MAX_SEMAPHORES; i++)
    {
        uint32_t mask = (1u << i);
        ATOMIC_SECTION_BEGIN();
        if ((*bitmap & mask) == 0)
        {
            *bitmap |= mask; // mark as used
            ATOMIC_SECTION_END();
            return i;
        }
        ATOMIC_SECTION_END();
    }
    return -1; // no free semaphore
}

// Release a semaphore slot by index.
// Safe: ignores invalid indices or already-free slots.
static inline void Semaphore_Release(uint32_t *bitmap, int id)
{
    if (id >= 0 && id < MAX_SEMAPHORES)
    {
        ATOMIC_SECTION_BEGIN();
        *bitmap &= ~(1u << id); // mark as free
        ATOMIC_SECTION_END();
    }
}

KERNAL_FUNCTION
int Semaphore_Allocate()
{
    int id = Semaphore_Alloc(g_kernel.seamphoreBitMask);
    return id;
}
KERNAL_FUNCTION
void Semaphore_Free(Semaphore_T *s)
{
    Semaphore_Release(g_kernel.seamphoreBitMask, s.id);
}
KERNAL_FUNCTION
void Semaphore_Signal(Semaphore_T *s)
{
    // Increment semaphore value atomically to avoid races.
    ATOMIC_SECTION_BEGIN();
    g_kernel.semaphoreList[s.index].value++;
    ATOMIC_SECTION_END();
}
KERNAL_FUNCTION
void Semaphore_Wait(Semaphore_T *s)
{
    // Try to grab semaphore atomically
    ATOMIC_SECTION_BEGIN();
    if (g_kernel.semaphoreList[s.index].value > 0)
    {
        g_kernel.semaphoreList[s.index].value--;
        ATOMIC_SECTION_END();
        return;
    }
    ATOMIC_SECTION_END();

    // Block current thread on semaphore (update state atomically)
    ATOMIC_SECTION_BEGIN();
    g_kernel.currentThread.state = THREAD_BLOCKED_SEMAPHORE;
    g_kernel.currentThread.semaphoreIndex = s.index;
    ATOMIC_SECTION_END();
}

KERNAL_FUNCTION
void Kernel_Queue_Create(MessageQueue *q, void *buffer,
                         uint16_t msgSize, uint16_t capacity)
{
    q->buffer = (uint8_t *)buffer;
    q->msgSize = msgSize;
    q->capacity = capacity;

    q->head = 0;
    q->tail = 0;
    q->count = 0;

    q->waitSend = NULL;
    q->waitRecv = NULL;

    q->inUse = 1;
}

KERNAL_FUNCTION
void Kernel_Queue_Send(MessageQueue *q, const void *msg)
{
    // Fast path: space available
    if (q->count < q->capacity)
    {
        // TODO: validate that (q->tail * q->msgSize) cannot overflow and that
        // dst..dst+q->msgSize lies within the queue buffer bounds to avoid
        // OOB writes from malicious msgSize/capacity values.
        uint8_t *dst = q->buffer + (q->tail * q->msgSize);
        memcpy(dst, msg, q->msgSize);

        q->tail = (q->tail + 1) % q->capacity;
        q->count++;

        return;
    }

    // Full: block current thread
    Thread *t = g_kernel.currentThread;
    if (!t)
        return; // or panic/reset if you want

    // Find queue index for wakeup logic in Scheduler_Tick
    uint8_t idx = 0xFF;
    for (uint8_t i = 0; i < MAX_QUEUES; i++)
    {
        if (&g_kernel.queueList[i] == q)
        {
            idx = i;
            break;
        }
    }

    t->queueIndex = idx;
    t->state = THREAD_BLOCKED_QUEUE_Send;

    // Trigger context switch
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

KERNAL_FUNCTION
void Kernel_Queue_Receive(MessageQueue *q, void *msgOut)
{
    // Fast path: data available
    if (q->count > 0)
    {
        // TODO: validate multiplication and range as above before copying.
        uint8_t *src = q->buffer + (q->head * q->msgSize);
        memcpy(msgOut, src, q->msgSize);

        q->head = (q->head + 1) % q->capacity;
        q->count--;

        return;
    }

    // Empty: block current thread
    Thread *t = g_kernel.currentThread;
    if (!t)
        return; // or panic/reset

    uint8_t idx = 0xFF;
    for (uint8_t i = 0; i < MAX_QUEUES; i++)
    {
        if (&g_kernel.queueList[i] == q)
        {
            idx = i;
            break;
        }
    }

    t->queueIndex = idx;
    t->state = THREAD_BLOCKED_QUEUE_RECV;

    // Trigger context switch
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

KERNAL_FUNCTION
QueueStatus Kernel_Queue_TrySend(MessageQueue *q, const void *msg)
{
    if (q->count >= q->capacity)
        return QUEUE_FULL;

    // TODO: validate multiplication and range before copying.
    uint8_t *dst = q->buffer + (q->tail * q->msgSize);
    memcpy(dst, msg, q->msgSize);

    q->tail = (q->tail + 1) % q->capacity;
    q->count++;

    return QUEUE_OK;
}

KERNAL_FUNCTION
QueueStatus Kernel_Queue_TryReceive(MessageQueue *q, void *msgOut)
{
    if (q->count == 0)
        return QUEUE_EMPTY;

    // TODO: validate multiplication and range before copying.
    uint8_t *src = q->buffer + (q->head * q->msgSize);
    memcpy(msgOut, src, q->msgSize);

    q->head = (q->head + 1) % q->capacity;
    q->count--;

    return QUEUE_OK;
}

KERNEL_FUNCTION
uint8_t Kernel_Timer_Create(Thread *t, uint32_t ms)
{
    for (uint8_t i = 0; i < MAX_TIMERS_PER_THREAD; i++)
    {
        SoftTimer *tm = &t->timers[i];

        if (!tm->active)
        {
            tm->active = true;
            tm->finished = false;
            tm->ms_left = ms;
            return i; // timerId
        }
    }

    return 0xFF; // no free timer slots
}

KERNEL_FUNCTION
bool Kernel_Timer_IsDone(Thread *t, uint8_t timerId)
{
    if (timerId >= MAX_TIMERS_PER_THREAD)
        return false;

    SoftTimer *tm = &t->timers[timerId];
    return tm->active && tm->finished;
}

KERNEL_FUNCTION
bool Kernel_Timer_Reset(Thread *t, uint8_t id, uint32_t ms)
{
    if (id >= MAX_TIMERS_PER_THREAD)
        return false;

    SoftTimer *tm = &t->timers[id];

    if (!tm->active)
        return false;

    tm->ms_left = ms;
    tm->finished = false;
    return true;
}

KERNEL_FUNCTION
bool Kernel_Timer_Cancel(Thread *t, uint8_t id)
{
    if (id >= MAX_TIMERS_PER_THREAD)
        return false;

    SoftTimer *tm = &t->timers[id];

    tm->active = false;
    tm->finished = false;
    tm->ms_left = 0;
    return true;
}

KERNEL_FUNCTION
uint32_t Kernel_Timer_Remaining(Thread *t, uint8_t id)
{
    if (id >= MAX_TIMERS_PER_THREAD)
        return 0;

    SoftTimer *tm = &t->timers[id];

    if (!tm->active)
        return 0;

    return tm->ms_left;
}

extern void *__kernel_text_start__;
extern void *__kernel_text_end__;
extern void *__api_table_start__;
extern void *__api_table_end__;
extern void *__heap_start__;
extern void *__heap_end__;
extern void *__stack_start__;
extern void *__stack_end__;

static inline bool Kernal_IsValidPointer(const void *ptr)
{
    if (!ptr)
        return false;

    uintptr_t p = (uintptr_t)ptr;

    uintptr_t text_start = (uintptr_t)&__kernel_text_start__;
    uintptr_t text_end = (uintptr_t)&__kernel_text_end__;
    uintptr_t api_start = (uintptr_t)&__api_table_start__;
    uintptr_t api_end = (uintptr_t)&__api_table_end__;
    uintptr_t heap_start = (uintptr_t)&__heap_start__;
    uintptr_t heap_end = (uintptr_t)&__heap_end__;
    uintptr_t stack_start = (uintptr_t)&__stack_start__;
    uintptr_t stack_end = (uintptr_t)&__stack_end__;

    // Normal ascending ranges
    if (p >= text_start && p < text_end)
        return true;

    if (p >= api_start && p < api_end)
        return true;

    if (p >= heap_start && p < heap_end)
        return true;

    // Stack may grow downward or upward depending on your linker script.
    // We handle both cases safely:
    if (stack_start < stack_end)
    {
        if (p >= stack_start && p < stack_end)
            return true;
    }
    else
    {
        if (p >= stack_end && p < stack_start)
            return true;
    }

    return false;
}

int Kernal_VFS_RegisterDriver(FileSystemDriver *driver)
{
    if (!driver)
        return -1;

    // Validate all function pointers
    if (!Kernal_IsValidPointer(driver->open)  ||
        !Kernal_IsValidPointer(driver->close) ||
        !Kernal_IsValidPointer(driver->read)  ||
        !Kernal_IsValidPointer(driver->write) ||
        !Kernal_IsValidPointer(driver->list))
    {
        return -1;
    }

    return VFS_RegisterDriver(driver);
}

int Kernal_FS_Open(const char *path, int flags)
{
    return VFS_Open(path, flags);
}

int Kernal_FS_Close(int fd)
{
    return VFS_Close(fd);
}

int Kernal_FS_Read(int fd, void *buffer, int size)
{
    return VFS_Read(fd, buffer, size);
}

int Kernal_FS_Write(int fd, const void *buffer, int size)
{
    return VFS_Write(fd, buffer, size);
}

int Kernal_FS_List(const char *path, char *outBuffer, int maxLen)
{
    return VFS_List(path, outBuffer, maxLen);
}

// Extract SVC immediate from the SVC instruction at (PC - 2)
static inline uint8_t read_svc_number(uint32_t stacked_pc)
{
    // Validate the address before dereferencing. If the stacked PC is not a
    // valid pointer (or the preceding halfword is not readable), return an
    // invalid svc number so the caller can safely ignore it.
    if (!Kernal_IsValidPointer((const void *)(stacked_pc - 2U)))
        return 0xFFU;

    uint16_t *svc_instr = (uint16_t *)(stacked_pc - 2U);
    return (uint8_t)(*svc_instr & 0xFFU);
}

static inline void set_return_r0(uint32_t *frame, uint32_t value)
{
    frame[0] = value; // R0 in stacked frame
}

// Naked SVC handler to preserve LR and access stacked frame
KERNAL_FUNCTION
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
KERNAL_FUNCTION
void SVC_Handler_C(uint32_t *frame, uint32_t lr)
{
    uint32_t stacked_pc = frame[6];
    uint8_t svc_no = read_svc_number(stacked_pc);
    Thread *caller = g_kernel.currentThread;

    switch (svc_no)
    {
    case SVC_THREAD_SLEEP:
        Kernel_Thread_Sleep(frame[0]);
        break;
    case SVC_YIELD:
        Kernel_Yield();
        break;
    case SVC_CREATE_THREAD:
    {
        _ThreadPartialArgs *extra = (_ThreadPartialArgs *)frame[3];
        Thread *t = (Thread *)frame[0];

        if (!Kernal_IsValidRange(extra, sizeof(_ThreadPartialArgs)))
            break;
        if (!Kernal_IsValidPointer(t) || !Kernal_IsValidRange(t, sizeof(Thread)))
            break;

        /* Copy user-supplied partial args into a kernel-owned stack buffer
           to avoid relying on a user stack-local pointer after return. */
        _ThreadPartialArgs kextra;
        memcpy(&kextra, extra, sizeof(kextra));

        void *stack_ptr = kextra.stack;
        if (stack_ptr && !Kernal_IsValidPointer(stack_ptr))
            break;

        Kernal_Create_Thread(t,
                             (void (*)(void *))frame[1],
                             (void *)frame[2],
                             stack_ptr,           // stack from copied partial args
                             (uint32_t)frame[4],  // stackBytes
                             (status_t)frame[5]); // priority

        /* Copy the name from the kernel-owned copy into the Thread struct */
        for (int i = 0; i < 8; i++)
        {
            t->name[i] = kextra.name[i];
        }
        break;
    }
    case SVC_ADD_PROCESS:
        Kernel_Add_Process((Process *)frame[0]);
        break;
    case SVC_EXIT:
        Kernal_Thread_Exit(frame[0]);
        break;
    case SVC_GPIO_WRITE:
        if (!Thread_HasCap(caller, CAP_IO))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        if (!Kernal_IsValidPointer((const void *)frame[0]))
            break;
        Kernel_GPIO_Write((GPIO_TypeDef *)frame[0], (uint16_t)frame[1], (GPIO_PinStnate)frame[2]);
        break;
    case SVC_GPIO_READ:
        if (!Thread_HasCap(caller, CAP_IO))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        if (!Kernal_IsValidPointer((const void *)frame[0]))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        frame[0] = Kernel_GPIO_Read((GPIO_TypeDef *)frame[0], (uint16_t)frame[1]);
        break;

    case SVC_UART_TRANSMIT:
        if (!Thread_HasCap(caller, CAP_IO))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        if (!Kernal_IsValidRange((void *)frame[0], sizeof(UART_Args)))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        frame[0] = (uint32_t)Kernel_UART_Transmit((UART_Args *)frame[0]);
        break;
    case SVC_UART_RECEIVE:
        if (!Thread_HasCap(caller, CAP_IO))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        if (!Kernal_IsValidRange((void *)frame[0], sizeof(UART_Args)))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        frame[0] = (uint32_t)Kernel_UART_Receive((UART_Args *)frame[0]);
        break;

    case SVC_I2C_MASTER_TXRX:
        if (!Thread_HasCap(caller, CAP_IO))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        if (!Kernal_IsValidRange((void *)frame[0], sizeof(I2C_Args)))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        frame[0] = (uint32_t)Kernel_I2C_Master_TransmitReceive((I2C_Args *)frame[0]);
        break;
    case SVC_GET_TICK:
        frame[0] = Kernel_GetTick();
        break;
    case SVC_SPI_TRANSMIT:
        if (!Thread_HasCap(caller, CAP_IO))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        if (!Kernal_IsValidRange((void *)frame[0], sizeof(SPI_Args)))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        frame[0] = (uint32_t)Kernel_SPI_Transmit((SPI_Args *)frame[0]);
        break;
    case SVC_SPI_RECEIVE:
        if (!Thread_HasCap(caller, CAP_IO))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        if (!Kernal_IsValidRange((void *)frame[0], sizeof(SPI_Args)))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        frame[0] = (uint32_t)Kernel_SPI_Receive((SPI_Args *)frame[0]);
        break;
    case SVC_SPI_TRANSMITRECV:
        if (!Thread_HasCap(caller, CAP_IO))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        if (!Kernal_IsValidRange((void *)frame[0], sizeof(SPI_Args)))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        frame[0] = Kernel_SPI_TransmitReceive((SPI_Args *)frame[0]);
        break;
    case SVC_GPIO_NEW:
    {
        const GPIO_NewArgs *a = (const GPIO_NewArgs *)frame[0];
        if (!Kernal_IsValidRange(a, sizeof(GPIO_NewArgs)))
        {
            set_return_r0(frame, (uint32_t)-1);
            break;
        }
        if (!Thread_HasCap(caller, CAP_IO))
        {
            set_return_r0(frame, (uint32_t)-1);
            break;
        }
        set_return_r0(frame, (uint32_t)Kernel_GPIO_New(a));
        break;
    }
    case SVC_UART_NEW:
    {
        const UART_NewArgs *a = (const UART_NewArgs *)frame[0];
        if (!Kernal_IsValidRange(a, sizeof(UART_NewArgs)))
        {
            set_return_r0(frame, (uint32_t)-1);
            break;
        }
        set_return_r0(frame, (uint32_t)Kernel_UART_New(a));
        break;
    }
    case SVC_I2C_NEW:
    {
        const I2C_NewArgs *a = (const I2C_NewArgs *)frame[0];
        if (!Kernal_IsValidRange(a, sizeof(I2C_NewArgs)))
        {
            set_return_r0(frame, (uint32_t)-1);
            break;
        }
        set_return_r0(frame, (uint32_t)Kernel_I2C_New(a));
        break;
    }
    case SVC_SPI_NEW:
    {
        const SPI_NewArgs *a = (const SPI_NewArgs *)frame[0];
        if (!Kernal_IsValidRange(a, sizeof(SPI_NewArgs)))
        {
            set_return_r0(frame, (uint32_t)-1);
            break;
        }
        set_return_r0(frame, (uint32_t)Kernel_SPI_New(a));
        break;
    }
    case SVC_MUTEX_CREATE:
    {
        Thread *maker = (Thread *)frame[0]; // r0 holds the thread pointer
        if (maker && !Kernal_IsValidPointer(maker))
        {
            frame[0] = (uint32_t)0;
            break;
        }
        Mutex *m = Kernal_Create_Mutex(maker);
        frame[0] = (uint32_t)m; // return mutex pointer in r0
        break;
    }
    case SVC_MUTEX_LOCK:
    {
        Mutex *m = (Mutex *)frame[0];
        if (!Kernal_IsValidPointer(m))
            break;
        Kernal_Lock_Mutex(m);
        break;
    }
    case SVC_MUTEX_UNLOCK:
    {
        Mutex *m = (Mutex *)frame[0];
        if (!Kernal_IsValidPointer(m))
            break;
        Kernal_Unlock_Mutex(m);
        break;
    }
    case SVC_MUTEX_READ_FROM_THREAD:
    {
        const void *buf = (const void *)frame[0];
        if (!Kernal_IsValidRange(buf, 5))
        {
            set_return_r0(frame, (uint32_t)-1);
            break;
        }
        Get_Mutex_Result result = Kernal_Get_Mutex((char (*)[5])frame[0], frame[1]);
        set_return_r0(frame, result);
        break;
    }
    case SVC_SEMAPHORE_GET:
    {
        int result = Semaphore_Allocate();
        set_return_r0(frame, result);
        break;
    }
    case SVC_SEMAPHORE_SIGNAL:
    {
        if (!Kernal_IsValidPointer((const void *)frame[0]))
            break;
        Semaphore_Signal((Semaphore_T *)frame[0]);
        break;
    }
    case SVC_SEMAPHORE_WAIT:
    {
        if (!Kernal_IsValidPointer((const void *)frame[0]))
            break;
        Semaphore_Wait((Semaphore_T *)frame[0]);
        break;
    }

    case SVC_QUEUE_CREATE:
    {
        const Queue_CreateArgs *a = (const Queue_CreateArgs *)frame[0]; // r0
        if (!Kernal_IsValidRange(a, sizeof(Queue_CreateArgs)))
            break;
        if (!Kernal_IsValidPointer(a->q))
            break;
        if (a->buffer && !Kernal_IsValidPointer(a->buffer))
            break;
        Kernel_Queue_Create(a->q, a->buffer, a->msgSize, a->capacity);
        break;
    }

    case SVC_QUEUE_SEND:
    {
        MessageQueue *q = (MessageQueue *)frame[0]; // r0
        const void *msg = (const void *)frame[1];   // r1
        if (!Kernal_IsValidPointer(q))
            break;
        if (msg && !Kernal_IsValidPointer(msg))
            break;
        Kernel_Queue_Send(q, msg);
        break;
    }

    case SVC_QUEUE_RECEIVE:
    {
        MessageQueue *q = (MessageQueue *)frame[0]; // r0
        void *msgOut = (void *)frame[1];            // r1
        if (!Kernal_IsValidPointer(q))
            break;
        if (msgOut && !Kernal_IsValidPointer(msgOut))
            break;
        Kernel_Queue_Receive(q, msgOut);
        break;
    }

    case SVC_QUEUE_TRY_SEND:
    {
        MessageQueue *q = (MessageQueue *)frame[0];
        const void *msg = (const void *)frame[1];
        if (!Kernal_IsValidPointer(q) || (msg && !Kernal_IsValidPointer(msg)))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        frame[0] = Kernel_Queue_TrySend(q, msg);
        break;
    }

    case SVC_QUEUE_TRY_RECEIVE:
    {
        MessageQueue *q = (MessageQueue *)frame[0];
        void *msgOut = (void *)frame[1];
        if (!Kernal_IsValidPointer(q) || (msgOut && !Kernal_IsValidPointer(msgOut)))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        frame[0] = Kernel_Queue_TryReceive(q, msgOut);
        break;
    }

    case SVC_TIMER_CREATE:
    {
        Thread *t = g_kernel.currentThread;
        uint32_t ms = frame[0];
        frame[0] = Kernel_Timer_Create(t, ms);
        break;
    }

    case SVC_TIMER_IS_DONE:
    {
        Thread *t = g_kernel.currentThread;
        uint8_t id = (uint8_t)frame[0];
        frame[0] = Kernel_Timer_IsDone(t, id);
        break;
    }

    case SVC_TIMER_RESET:
    {
        Thread *t = g_kernel.currentThread;
        uint8_t id = (uint8_t)frame[0];
        uint32_t ms = frame[1];
        frame[0] = Kernel_Timer_Reset(t, id, ms);
        break;
    }

    case SVC_TIMER_CANCEL:
    {
        Thread *t = g_kernel.currentThread;
        uint8_t id = (uint8_t)frame[0];
        frame[0] = Kernel_Timer_Cancel(t, id);
        break;
    }

    case SVC_TIMER_REMAINING:
    {
        Thread *t = g_kernel.currentThread;
        uint8_t id = (uint8_t)frame[0];
        frame[0] = Kernel_Timer_Remaining(t, id);
        break;
    }
    case SVC_MALLOC:
    {
        if (!Thread_HasCap(caller, CAP_MEM_MANAGE))
        {
            frame[0] = (uint32_t)0;
            break;
        }
        size_t size = (size_t)frame[0]; // r0
        void *ptr = Kernal_Malloc(size);
        frame[0] = (uint32_t)ptr; // return in r0
        break;
    }

    case SVC_FREE:
    {
        if (!Thread_HasCap(caller, CAP_MEM_MANAGE))
        {
            break;
        }
        void *ptr = (void *)frame[0]; // r0
        Kernal_Free(ptr);
        // no return value
        break;
    }

    case SVC_CALLOC:
    {
        if (!Thread_HasCap(caller, CAP_MEM_MANAGE))
        {
            frame[0] = (uint32_t)0;
            break;
        }
        size_t n = (size_t)frame[0];    // r0
        size_t size = (size_t)frame[1]; // r1
        void *ptr = Kernal_Calloc(n, size);
        frame[0] = (uint32_t)ptr;
        break;
    }

    case SVC_REALLOC:
    {
        if (!Thread_HasCap(caller, CAP_MEM_MANAGE))
        {
            frame[0] = (uint32_t)0;
            break;
        }
        void *old = (void *)frame[0];   // r0
        size_t size = (size_t)frame[1]; // r1
        void *ptr = Kernal_Realloc(old, size);
        frame[0] = (uint32_t)ptr;
        break;
    }

    case SVC_FS_OPEN:
    {
        if (!Thread_HasCap(caller, CAP_FILESYSTEM))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        const char *path = (const char *)frame[0]; // r0
        int flags = (int)frame[1];                 // r1

        int fd = Kernal_FS_Open(path, flags);
        frame[0] = (uint32_t)fd; // return in r0
        break;
    }

    case SVC_FS_CLOSE:
    {
        if (!Thread_HasCap(caller, CAP_FILESYSTEM))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        int fd = (int)frame[0]; // r0

        int result = Kernal_FS_Close(fd);
        frame[0] = (uint32_t)result; // return in r0
        break;
    }

    case SVC_FS_READ:
    {
        if (!Thread_HasCap(caller, CAP_FILESYSTEM))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        int fd = (int)frame[0];          // r0
        void *buffer = (void *)frame[1]; // r1
        int size = (int)frame[2];        // r2
        if (size < 0 || (size > 0 && !Kernal_IsValidRange(buffer, (size_t)size)))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        int bytes = Kernal_FS_Read(fd, buffer, size);
        frame[0] = (uint32_t)bytes; // return in r0
        break;
    }

    case SVC_FS_WRITE:
    {
        if (!Thread_HasCap(caller, CAP_FILESYSTEM))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        int fd = (int)frame[0];                      // r0
        const void *buffer = (const void *)frame[1]; // r1
        int size = (int)frame[2];                    // r2
        if (size < 0 || (size > 0 && !Kernal_IsValidRange(buffer, (size_t)size)))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        int bytes = Kernal_FS_Write(fd, buffer, size);
        frame[0] = (uint32_t)bytes; // return in r0
        break;
    }

    case SVC_FS_LIST:
    {
        if (!Thread_HasCap(caller, CAP_FILESYSTEM))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        const char *path = (const char *)frame[0]; // r0
        char *outBuffer = (char *)frame[1];        // r1
        int maxLen = (int)frame[2];                // r2
        if (path && !Kernal_IsValidPointer(path))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        if (maxLen < 0 || (maxLen > 0 && (outBuffer == NULL || !Kernal_IsValidRange(outBuffer, (size_t)maxLen))))
        {
            frame[0] = (uint32_t)-1;
            break;
        }

        int bytes = Kernal_FS_List(path, outBuffer, maxLen);
        frame[0] = (uint32_t)bytes; // return in r0
        break;
    }

    case SVC_VFS_REGISTER_DRIVER:
    {
        if (!Thread_HasCap(caller, CAP_ADMIN))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        FileSystemDriver *drv = (FileSystemDriver *)frame[0]; // r0
        int result = Kernal_VFS_RegisterDriver(drv);
        frame[0] = (uint32_t)result;
        break;
    }
    case SVC_DUMP_FAULT_TRACE:
    {
        char *out = (char *)frame[0];
        int maxLen = (int)frame[1];
        if (!out || maxLen <= 0 || !Kernal_IsValidRange(out, (size_t)maxLen))
        {
            frame[0] = (uint32_t)-1;
            break;
        }
        extern int Kernel_Dump_FaultTrace(char *outBuffer, int maxLen);
        int written = Kernel_Dump_FaultTrace(out, maxLen);
        frame[0] = (uint32_t)written;
        break;
    }

    default:
        break;
    }
}
KERNAL_FUNCTION
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

API_FUNCTION(Malloc)
void *Malloc(size_t size)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)size;

    __asm volatile("svc %0" ::"i"(SVC_MALLOC), "r"(r0) : "memory");

    void *ptr;
    __asm volatile("mov %0, r0" : "=r"(ptr));
    return ptr;
}

API_FUNCTION(Free)
void Free(void *ptr)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)ptr;

    __asm volatile("svc %0" ::"i"(SVC_FREE), "r"(r0) : "memory");
}

API_FUNCTION(Calloc)
void *Calloc(size_t n, size_t size)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)n;
    register uint32_t r1 __asm__("r1") = (uint32_t)size;

    __asm volatile("svc %0" ::"i"(SVC_CALLOC), "r"(r0), "r"(r1) : "memory");

    void *ptr;
    __asm volatile("mov %0, r0" : "=r"(ptr));
    return ptr;
}

API_FUNCTION(Realloc)
void *Realloc(void *ptr, size_t newSize)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)ptr;
    register uint32_t r1 __asm__("r1") = (uint32_t)newSize;

    __asm volatile("svc %0" ::"i"(SVC_REALLOC), "r"(r0), "r"(r1) : "memory");

    void *newPtr;
    __asm volatile("mov %0, r0" : "=r"(newPtr));
    return newPtr;
}


API_FUNCTION(FS_Open)
int FS_Open(const char *path, int flags)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)path;
    register uint32_t r1 __asm__("r1") = (uint32_t)flags;

    __asm volatile("svc %0" ::"i"(SVC_FS_OPEN), "r"(r0), "r"(r1) : "memory");

    int fd;
    __asm volatile("mov %0, r0" : "=r"(fd));
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}

API_FUNCTION(FS_Read)
int FS_Read(int fd, void *buffer, int size)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)fd;
    register uint32_t r1 __asm__("r1") = (uint32_t)buffer;
    register uint32_t r2 __asm__("r2") = (uint32_t)size;

    register uint32_t r1 __asm__("r1") = (uint32_t)buffer;
    register uint32_t r2 __asm__("r2") = (uint32_t)size;

    __asm volatile("svc %0" ::"i"(SVC_FS_WRITE), "r"(r0), "r"(r1), "r"(r2) : "memory");

    int bytes;
    __asm volatile("mov %0, r0" : "=r"(bytes));
    return bytes;
}

API_FUNCTION(FS_List)
int FS_List(const char *path, char *outBuffer, int maxLen)
API_FUNCTION(VFS_RegisterDriver)
int VFS_RegisterDriver(FileSystemDriver *driver)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)driver;

    __asm volatile("svc %0" ::"i"(SVC_VFS_REGISTER_DRIVER), "r"(r0) : "memory");

    int result;
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}

// Validate that a contiguous range [ptr, ptr+len) is within a valid region.
static inline bool Kernal_IsValidRange(const void *ptr, size_t len)
{
    if (!ptr)
        return false;

    if (len == 0)
        return true;

    uintptr_t start = (uintptr_t)ptr;
    uintptr_t end = start + (uintptr_t)len;

    // overflow check
    if (end < start)
        return false;

    uintptr_t text_start = (uintptr_t)&__kernel_text_start__;
    uintptr_t text_end = (uintptr_t)&__kernel_text_end__;
    uintptr_t api_start = (uintptr_t)&__api_table_start__;
    uintptr_t api_end = (uintptr_t)&__api_table_end__;
    uintptr_t heap_start = (uintptr_t)&__heap_start__;
    uintptr_t heap_end = (uintptr_t)&__heap_end__;
    uintptr_t stack_start = (uintptr_t)&__stack_start__;
    uintptr_t stack_end = (uintptr_t)&__stack_end__;

    // Check if fully contained in text, api, or heap ranges
    if (start >= text_start && end <= text_end)
        return true;
    if (start >= api_start && end <= api_end)
        return true;
    if (start >= heap_start && end <= heap_end)
        return true;

    // Stack may grow up or down depending on linker script. Accept either.
    if (stack_start < stack_end)
    {
        if (start >= stack_start && end <= stack_end)
            return true;
    }
    else
    {
        if (start >= stack_end && end <= stack_start)
            return true;
    }

    return false;
}

