#include <thread.h>
#include <process.h>
#include <stddef.h>
#include <stdint.h>
#include <mpu.h>

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
    // If lazily stacking FP context, also capture FP state when active.
    // Optionally detect FPCA bit in CONTROL before touching FP regs.
    // t->fpscr = __get_FPSCR(); // GET FPSCR
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
                   priority_t priority, char name[5])
{
    _ThreadPartialArgs extra;
    extra.stack = stack;
    strncpy(extra.name, name, 5);

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

API_FUNCTION(Mutex_Create)
Mutex *Mutex_Create(Thread *maker)
{
    register Thread *r0 __asm__("r0") = maker;
    register Mutex *ret __asm__("r0");
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_MUTEX_CREATE) : "memory");
    return ret;
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
    register mutex *r0 __asm__("r0") = m;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_MUTEX_UNLOCK) : "memory");
}
API_FUNCTION(Get_Mutex)
Get_Mutex_Result Get_Mutex(char name[5], int id)
{
    register char(*r0) __asm__("r0") = &name;
    register int __asm__("r1") = id;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_MUTEX_READ_FROM_THREAD) : "memory");
}
API_FUNCTION(Get_Semaphore)
Semaphore_T Get_Semaphore(void)
{
    register int ret __asm__("r0");
    __asm volatile("svc %[imm]" ::[imm] "I"(SVC_SEMAPHORE_GET) : "memory");
    return (Semaphore_T){.id = ret};
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

void SysTick_Handler(void)
{
    Scheduler_Tick(); // 1ms tick
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
    uint32_t g_kernel->ticks = g_kernel.systemTicks++; // increment global tick counter

    for (uint8_t i = 0; i < g_kernel.threadCount; i++)
    {
        Thread *t = g_kernel.threadList[i];

        // Periodic releases (BLOCKED + period)
        if (t->state == THREAD_BLOCKED && t->periodTicks > 0)
        {
            if (g_kernel->systemTicks >= t->nextReleaseTick)
            {
                t->state = THREAD_READY;
                t->nextReleaseTick = g_kernel->ticks + t->periodTicks;
            }
        }

        // Sleep wakeups
        if (t->state == THREAD_SLEEPING)
        {
            if ((int32_t)(g_kernel->systemTicks - t->nextReleaseTick) >= 0)
            {
                t->state = THREAD_READY;
            }
        }

        // Semaphore waiting wakeups
        if (t->state == THREAD_BLOCKED_SEMAPHORE)
        {
            if (g_kernel->semaphoreList[t->semaphoreIndex].value > 0)
            {
                t->state = THREAD_READY;
                g_kernel->semaphoreList[t->semaphoreIndex].value--;
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
        if (t->state == THREAD_BLOCKED_QUEUE_SEND)
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
    // Reset Super Imporant Registers
    t->psp = 0;
    t->stackBase = 0;
    t->stackSize = 0;
    t->priority = 0;
    t->entry = 0;
    // clear imporant registers efficently
    memset(&tcb->context, 0, sizeof(tcb->context));

    // Clear Name
    memset(p->name, 0, sizeof(p->name));
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

    g_kernel.threadList[g_kernel.threadCount++] = t;
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

    m->locked = false;
    m->owner = NULL;

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
    return malloc(size);
}

void Kernal_Free(void *ptr)
{
    free(ptr);
}

void *Kernal_Calloc(size_t n, size_t size)
{
    return calloc(n, size);
}

void *Kernal_Realloc(void *ptr, size_t newSize)
{
    return realloc(ptr, newSize);
}

static inline Thread *findThreadByName(const char *target)
{
    for (int i = 0; i < MAX_THREADS; i++)
    {
        // Compare up to 5 chars (since name is char[5])
        if (strncmp(g_kernel.threadList[i].name, target, 5) == 0)
        {
            return &threadList[i]; // Found
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
            return (Get_Mutex_Result)m->locked; // this only works as long as Get_Mutex_Result enum has UNLOCKED=0 and LOCKED=1 otherwise needs diffrent approch
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
        if ((*bitmap & mask) == 0)
        {
            *bitmap |= mask; // mark as used
            return i;
        }
    }
    return -1; // no free semaphore
}

// Release a semaphore slot by index.
// Safe: ignores invalid indices or already-free slots.
static inline void Semaphore_Release(uint32_t *bitmap, int id)
{
    if (id >= 0 && id < MAX_SEMAPHORES)
    {
        *bitmap &= ~(1u << id); // mark as free
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
    g_kernel.semaphoreList[s.index].value++;
}
KERNAL_FUNCTION
void Semaphore_Wait(Semaphore_T *s)
{
    if (g_kernel.semaphoreList[s.index].value > 0)
    {
        g_kernel.semaphoreList[s.index].value--;
    }
    else
    {
        g_kernel.currentThread.state = THREAD_BLOCKED_SEMAPHORE;
        g_kernel.currentThread.semaphoreIndex = s.index;
    }
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
    t->state = THREAD_BLOCKED_QUEUE_SEND;

    // Trigger context switch
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

KERNAL_FUNCTION
void Kernel_Queue_Receive(MessageQueue *q, void *msgOut)
{
    // Fast path: data available
    if (q->count > 0)
    {
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

// Extract SVC immediate from the SVC instruction at (PC - 2)
static inline uint8_t read_svc_number(uint32_t stacked_pc)
{
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

        Kernal_Create_Thread((Thread *)frame[0],
                             (void (*)(void *))frame[1],
                             (void *)frame[2],
                             extra->stack,        // stack from partial args
                             (uint32_t)frame[4],  // stackBytes
                             (status_t)frame[5]); // priority

        // Copy the name into the Thread struct
        Thread *t = (Thread *)frame[0];
        for (int i = 0; i < 5; i++)
        {
            t->name[i] = extra->name[i];
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
        Kernel_GPIO_Write((GPIO_TypeDef *)frame[0], (uint16_t)frame[1], (GPIO_PinState)frame[2]);
        break;
    case SVC_GPIO_READ:
        frame[0] = Kernel_GPIO_Read((GPIO_TypeDef *)frame[0], (uint16_t)frame[1]);
        break;

    case SVC_UART_TRANSMIT:
        frame[0] = (uint32_t)Kernel_UART_Transmit((UART_Args *)frame[0]);
        break;
    case SVC_UART_RECEIVE:
        frame[0] = (uint32_t)Kernel_UART_Receive((UART_Args *)frame[0]);
        break;

    case SVC_I2C_MASTER_TXRX:
        frame[0] = (uint32_t)Kernel_I2C_Master_TransmitReceive((I2C_Args *)frame[0]);
        break;
    case SVC_GET_TICK:
        frame[0] = Kernel_GetTick();
        break;
    case SVC_SPI_TRANSMIT:
        frame[0] = (uint32_t)Kernel_SPI_Transmit((SPI_Args *)frame[0]);
        break;
    case SVC_SPI_RECEIVE:
        frame[0] = (uint32_t)Kernel_SPI_Receive((SPI_Args *)frame[0]);
        break;
    case SVC_SPI_TRANSMITRECV:
        frame[0] = Kernel_SPI_TransmitReceive((SPI_Args *)frame[0]);
        break;
    case SVC_GPIO_NEW:
    {
        const GPIO_NewArgs *a = (const GPIO_NewArgs *)frame[0];
        set_return_r0(frame, (uint32_t)Kernel_GPIO_New(a));
        break;
    }
    case SVC_UART_NEW:
    {
        const UART_NewArgs *a = (const UART_NewArgs *)frame[0];
        set_return_r0(frame, (uint32_t)Kernel_UART_New(a));
        break;
    }
    case SVC_I2C_NEW:
    {
        const I2C_NewArgs *a = (const I2C_NewArgs *)frame[0];
        set_return_r0(frame, (uint32_t)Kernel_I2C_New(a));
        break;
    }
    case SVC_SPI_NEW:
    {
        const SPI_NewArgs *a = (const SPI_NewArgs *)frame[0];
        set_return_r0(frame, (uint32_t)Kernel_SPI_New(a));
        break;
    }
    case SVC_MUTEX_CREATE:
    {
        Thread *maker = (Thread *)frame[0]; // r0 holds the thread pointer
        Mutex *m = Kernal_Create_Mutex(maker);
        frame[0] = (uint32_t)m; // return mutex pointer in r0
        break;
    }
    case SVC_MUTEX_LOCK:
    {
        Mutex *m = (Mutex *)frame[0];
        Kernal_Lock_Mutex(m);
        break;
    }
    case SVC_MUTEX_UNLOCK:
    {
        Mutex *m = (Mutex *)frame[0];
        Kernal_Unlock_Mutex(m);
        break;
    }
    case SVC_MUTEX_READ_FROM_THREAD:
    {
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
        Semaphore_Signal((Semaphore_T *)frame[0]);
        break;
    }
    case SVC_SEMAPHORE_WAIT:
    {
        Semaphore_Wait((Semaphore_T *)frame[0]);
        break;
    }

    case SVC_QUEUE_CREATE:
    {
        const Queue_CreateArgs *a = (const Queue_CreateArgs *)frame[0]; // r0
        Kernel_Queue_Create(a->q, a->buffer, a->msgSize, a->capacity);
        break;
    }

    case SVC_QUEUE_SEND:
    {
        MessageQueue *q = (MessageQueue *)frame[0]; // r0
        const void *msg = (const void *)frame[1];   // r1
        Kernel_Queue_Send(q, msg);
        break;
    }

    case SVC_QUEUE_RECEIVE:
    {
        MessageQueue *q = (MessageQueue *)frame[0]; // r0
        void *msgOut = (void *)frame[1];            // r1
        Kernel_Queue_Receive(q, msgOut);
        break;
    }

    case SVC_QUEUE_TRY_SEND:
    {
        MessageQueue *q = (MessageQueue *)frame[0];
        const void *msg = (const void *)frame[1];
        frame[0] = Kernel_Queue_TrySend(q, msg);
        break;
    }

    case SVC_QUEUE_TRY_RECEIVE:
    {
        MessageQueue *q = (MessageQueue *)frame[0];
        void *msgOut = (void *)frame[1];
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
        return (uint32_t)Kernal_Malloc((size_t)arg0);

    case SVC_FREE:
        Kernal_Free((void *)arg0);
        return 0;

    case SVC_CALLOC:
        return (uint32_t)Kernal_Calloc((size_t)arg0, (size_t)arg1);

    case SVC_REALLOC:
        return (uint32_t)Kernal_Realloc((void *)arg0, (size_t)arg1);

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

// GPIO
API_FUNCTION(GPIO_WritePin)
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
API_FUNCTION(I2C_Master_TransmitReceive)
HAL_StatusTypeDef I2C_Master_TransmitReceive(I2C_HandleTypeDef *hi2c,
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
API_FUNCTION(Queue_Create)
void Queue_Create(MessageQueue *q, void *buffer,
                  uint16_t msgSize, uint16_t capacity)
{
    Queue_CreateArgs args = {
        .q = q,
        .buffer = buffer,
        .msgSize = msgSize,
        .capacity = capacity};

    register Queue_CreateArgs *r0 __asm__("r0") = &args;
    __asm volatile("svc %0" ::"i"(SVC_QUEUE_CREATE), "r"(r0) : "memory");
}
API_FUNCTION(Queue_Send)
void Queue_Send(MessageQueue *q, const void *msg)
{
    register MessageQueue *r0 __asm__("r0") = q;
    register const void *r1 __asm__("r1") = msg;

    __asm volatile("svc %0" ::"i"(SVC_QUEUE_SEND), "r"(r0), "r"(r1) : "memory");
}
API_FUNCTION(Queue_Receive)
void Queue_Receive(MessageQueue *q, void *msgOut)
{
    register MessageQueue *r0 __asm__("r0") = q;
    register void *r1 __asm__("r1") = msgOut;

    __asm volatile("svc %0" ::"i"(SVC_QUEUE_RECEIVE), "r"(r0), "r"(r1) : "memory");
}
API_FUNCTION(Queue_TrySend)
QueueStatus Queue_TrySend(MessageQueue *q, const void *msg)
{
    register MessageQueue *r0 __asm__("r0") = q;
    register const void *r1 __asm__("r1") = msg;

    __asm volatile("svc %0" ::"i"(SVC_QUEUE_TRY_SEND), "r"(r0), "r"(r1) : "memory");

    QueueStatus result;
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}
API_FUNCTION(Queue_TryReceive)
QueueStatus Queue_TryReceive(MessageQueue *q, void *msgOut)
{
    register MessageQueue *r0 __asm__("r0") = q;
    register void *r1 __asm__("r1") = msgOut;

    __asm volatile("svc %0" ::"i"(SVC_QUEUE_TRY_RECEIVE), "r"(r0), "r"(r1) : "memory");

    QueueStatus result;
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}
API_FUNCTION(Timer_Create)
uint8_t Timer_Create(uint32_t ms)
{
    register uint32_t r0 __asm__("r0") = ms;

    __asm volatile("svc %0" ::"i"(SVC_TIMER_CREATE), "r"(r0) : "memory");

    uint8_t id;
    __asm volatile("mov %0, r0" : "=r"(id));
    return id;
}
API_FUNCTION(Timer_IsDone)
bool Timer_IsDone(uint8_t timerId)
{
    register uint32_t r0 __asm__("r0") = timerId;

    __asm volatile("svc %0" ::"i"(SVC_TIMER_IS_DONE), "r"(r0) : "memory");

    bool done;
    __asm volatile("mov %0, r0" : "=r"(done));
    return done;
}

API_FUNCTION(Timer_Reset)
bool Timer_Reset(uint8_t timerId, uint32_t ms)
{
    register uint32_t r0 __asm__("r0") = timerId;
    register uint32_t r1 __asm__("r1") = ms;

    __asm volatile("svc %0" ::"i"(SVC_TIMER_RESET), "r"(r0), "r"(r1) : "memory");

    bool result;
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}

API_FUNCTION(Timer_Cancel)
bool Timer_Cancel(uint8_t timerId)
{
    register uint32_t r0 __asm__("r0") = timerId;

    __asm volatile("svc %0" ::"i"(SVC_TIMER_CANCEL), "r"(r0) : "memory");

    bool result;
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}

API_FUNCTION(Timer_Remaining)
uint32_t Timer_Remaining(uint8_t timerId)
{
    register uint32_t r0 __asm__("r0") = timerId;

    __asm volatile("svc %0" ::"i"(SVC_TIMER_REMAINING), "r"(r0) : "memory");

    uint32_t remaining;
    __asm volatile("mov %0, r0" : "=r"(remaining));
    return remaining;
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