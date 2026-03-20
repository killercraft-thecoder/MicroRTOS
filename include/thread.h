/**
 * @file thread.h
 * @brief Thread management API and kernel-mediated I/O wrappers.
 *
 * @note I/O functions (GPIO, UART, I2C) are placed here — rather than in a
 *       separate io.h — because in this RTOS design, all hardware access is
 *       mediated by the kernel via SVC calls. These wrappers are part of the
 *       *thread-facing* API surface: they are the only safe way for an
 *       unprivileged thread to request I/O. Grouping them here keeps all
 *       user-callable, SVC-backed services in one place.
 */

#ifndef MICRO_RTOS_THREAD_H
#define MICRO_RTOS_THREAD_H

#include <stdint.h>
#include <stdbool.h>
#include <CMSIS/stm32f4xx_hal_gpio.h>
#include <CMSIS/stm32f4xx_hal_uart.h>
#include <CMSIS/stm32f4xx_hal_spi.h>
#include <CMSIS/core_cm4.h>
#include "mustinclude.h"
#include "tlsf.h"

typedef struct
{
    volatile int32_t value;
    volatile uint8_t locked; // 0 = free, 1 = locked
} ATOMIC_NUMBER;

typedef struct
{
    volatile bool value;
    volatile uint8_t locked;
} ATOMIC_BOOL;

typedef struct
{
    volatile uint32_t bits;
    volatile uint8_t locked;
} ATOMIC_FLAGS;

static inline void Atomic_Lock(ATOMIC_NUMBER *a)
{
    while (__atomic_test_and_set(&a->locked, __ATOMIC_ACQUIRE))
    {
        // busy wait
    }
}

static inline void Atomic_Unlock(ATOMIC_NUMBER *a)
{
    __atomic_clear(&a->locked, __ATOMIC_RELEASE);
}

static inline void Atomic_Set(ATOMIC_NUMBER *a, int32_t val)
{
    Atomic_Lock(a);
    a->value = val;
    Atomic_Unlock(a);
}

static inline int32_t Atomic_Get(ATOMIC_NUMBER *a)
{
    Atomic_Lock(a);
    int32_t val = a->value;
    Atomic_Unlock(a);
    return val;
}

static inline void AtomicBool_Set(ATOMIC_BOOL *a, bool val)
{
    while (__atomic_test_and_set(&a->locked, __ATOMIC_ACQUIRE))
        ;
    a->value = val;
    __atomic_clear(&a->locked, __ATOMIC_RELEASE);
}

static inline bool AtomicBool_Get(ATOMIC_BOOL *a)
{
    while (__atomic_test_and_set(&a->locked, __ATOMIC_ACQUIRE))
        ;
    bool val = a->value;
    __atomic_clear(&a->locked, __ATOMIC_RELEASE);
    return val;
}

static inline void AtomicFlags_Set(ATOMIC_FLAGS *f, uint32_t mask)
{
    while (__atomic_test_and_set(&f->locked, __ATOMIC_ACQUIRE))
        ;
    f->bits |= mask;
    __atomic_clear(&f->locked, __ATOMIC_RELEASE);
}

static inline void AtomicFlags_Clear(ATOMIC_FLAGS *f, uint32_t mask)
{
    while (__atomic_test_and_set(&f->locked, __ATOMIC_ACQUIRE))
        ;
    f->bits &= ~mask;
    __atomic_clear(&f->locked, __ATOMIC_RELEASE);
}

static inline bool AtomicFlags_Test(ATOMIC_FLAGS *f, uint32_t mask)
{
    while (__atomic_test_and_set(&f->locked, __ATOMIC_ACQUIRE))
        ;
    bool result = (f->bits & mask) != 0;
    __atomic_clear(&f->locked, __ATOMIC_RELEASE);
    return result;
}

#define __atomic_read(src, dest) \
    do                           \
    {                            \
        __disable_irq();         \
        (dest) = (src);          \
        __enable_irq();          \
    } while (0)

#define __atomic_write(var, value) \
    do                             \
    {                              \
        __disable_irq();           \
        (var) = (value);           \
        __finish_writes();         \
        __enable_irq();            \
    } while (0)

// Ensure all memory writes complete before continuing
#define __finish_writes() __DSB()

// Ensure memory accesses are observed in order
#define __flush_memory() __DMB()

// Flush CPU pipeline (use after changing system control registers)
#define __flush_pipeline() __ISB()

// Full sync: flush memory + finish writes + pipeline
#define __sync_all() \
    do               \
    {                \
        __DMB();     \
        __DSB();     \
        __ISB();     \
    } while (0)
#if defined(__GNUC__) || defined(__clang__)
#define ALIGN_STACK __attribute__(aligned(8))
#elif defined(_MSC_VER)
#define ALIGN_STACK __declspec(align(8))
#else
#error "ALIGN_STACK Not Supported on this compiler"
#endif

// Time & durations
typedef uint32_t time_t;    // absolute time in ms since boot (1 tick = 1 ms)
typedef int32_t duration_t; // signed interval in ms (end - start)

// Scheduling & priorities
typedef int priority_t;  // thread priority level
typedef uint32_t tick_t; // tick count

// Flags & masks
typedef uint32_t flag32_t; // 32-bit flags
typedef uint64_t flag64_t; // 64-bit flags
typedef uint16_t flag16_t; // 16-bit flags

// Error/status codes
typedef int status_t; // return codes from threads. and maybe OS functions in the future.

#define MAX_TIMERS_PER_THREAD 4

#define KERNAL_FUNCTION __attribute__((section(".text.kernel")))
#if defined(__GNUC__) || defined(__clang__)
#define API_FUNCTION(fn)        \
    static void *__api_ptr_##fn \
        __attribute__((used, section(".api_table"))) = fn;
#elif defined(_MSC_VER)
#pragma section(".api_table", read)
#define API_FUNCTION(fn) \
    __declspec(allocate(".api_table")) static void *__api_ptr_##fn = fn
#else
#error "API_FUNCTION macro not supported for this compiler"
#endif

typedef enum : uint8_t
{
    THREAD_READY,               // In ready queue, waiting to run
    THREAD_RUNNING,             // Currently executing
    THREAD_BLOCKED,             // Waiting on event/semaphore/mutex
    THREAD_SLEEPING,            // Delayed until a wakeup tick
    THREAD_SUSPENDED,           // Explicitly stopped, not scheduled
    THREAD_TERMINATED,          // Finished, awaiting cleanup
    THREAD_WAITING_FOR_SERVICE, // Waiting For Service. (not used)
    THREAD_HALTED,              // Thread stopped due to fault or security ban
    THREAD_BLOCKED_SEMAPHORE,   // Thread Blocked Waiting For Semaphore
    THREAD_BLOCKED_Queue_Send,
    THREAD_BLOCKED_QUEUE_RECV,
} ThreadState;

enum : uint8_t
{
    // Thread/Process services
    SVC_THREAD_SLEEP = 1,
    SVC_YIELD = 2,
    SVC_CREATE_THREAD = 3,
    SVC_ADD_PROCESS = 4,
    SVC_EXIT = 5,
    SVC_REMOVE_PROCESS = 6, // This is gonna drive me up the walls. But remeber never change any existing svcs

    // I/O services
    SVC_GPIO_WRITE = 10,
    SVC_GPIO_READ = 11,
    SVC_UART_TRANSMIT = 12,
    SVC_UART_RECEIVE = 13,
    SVC_I2C_MASTER_TXRX = 14,
    SVC_SPI_TRANSMIT = 15,
    SVC_SPI_RECEIVE = 16,
    SVC_SPI_TRANSMITRECV = 17,
    SVC_GPIO_NEW = 18,
    SVC_UART_NEW = 19,
    SVC_I2C_NEW = 20,
    SVC_SPI_NEW = 21,

    // Time Services
    SVC_GET_TICK = 40,
    SVC_TIMER_CREATE = 41,
    SVC_TIMER_ISDONE = 42,
    SVC_TIMER_RESET = 43,
    SVC_TIMER_CANCEL = 44,
    SVC_TIMER_REMAINING = 45,

    // Mutex & Semaphores
    SVC_MUTEX_CREATE = 50,
    SVC_MUTEX_LOCK = 51,
    SVC_MUTEX_UNLOCK = 52,
    SVC_MUTEX_DESTROY = 53,
    SVC_MUTEX_READ_FROM_THREAD = 54,
    SVC_SEMAPHORE_GET = 55,
    SVC_SEMAPHORE_SIGNAL = 56,
    SVC_SEMAPHORE_WAIT = 57,

    // Mesage Queues
    SVC_Queue_Create = 60, // Creates a queue with a user-provided buffer
    SVC_Queue_Send = 61,   // Sends a message (blocks if full)
    SVC_QUEUE_RECIVE = 62, // Recives a message (blocks if empty)
    SVC_QUEUE_TRY_SEND = 63,
    SVC_QUEUE_TRY_RECEIVE = 64,

    // Memory
    SVC_MALLOC = 70,
    SVC_FREE = 71,
    SVC_CALLOC = 72,
    SVC_REALLOC = 73,

    // Filesystem Services
    SVC_FS_OPEN = 80,
    SVC_FS_CLOSE = 81,
    SVC_FS_READ = 82,
    SVC_FS_WRITE = 83,
    SVC_FS_LIST = 84,
    SVC_VFS_REGISTER_DRIVER = 85,

};

typedef enum : uint32_t
{
    UNLOCKED = 0,
    LOCKED = 1,
    FAIL = 2
} Get_Mutex_Result;

typedef struct
{
    bool locked;
    Thread *owner;
} Mutex;

typedef struct
{
    uint8_t value; // if >0 then can be waited on and immditly resume , if equal to 0 then block task.
} Semaphore;

typedef struct
{
    uint8_t index; // index in the kernals list of semaphores this is pointing to
} Semaphore_T;

typedef struct
{
    MessageQueue *q;   // where to initialize
    void *buffer;      // backing storage
    uint16_t msgSize;  // bytes per message
    uint16_t capacity; // number of messages
} Queue_CreateArgs;

typedef struct
{
    uint32_t *stack;
    char *name[8];
} _ThreadPartialArgs;

/**
 * @typedef MessageQueue
 * @brief Structure representing a fixed‑size message queue.
 *
 * The queue stores messages of uniform size in a circular buffer.
 * All fields are managed by the kernel; user code should treat this
 * as an opaque type
 */
typedef struct
{
    uint8_t *buffer;   // Pointer to raw message storage
    uint16_t msgSize;  // Size of each message in bytes
    uint16_t capacity; // Max number of messages

    uint16_t head;  // Read index
    uint16_t tail;  // Write index
    uint16_t count; // Number of messages currently in queue

    Thread *waitSend; // Linked list of threads blocked on send
    Thread *waitRecv; // Linked list of threads blocked on receive

    uint8_t inUse; // 0 = free slot, 1 = allocated queue
} MessageQueue;

/**
 * @enum QueueStatus
 * @brief Return codes for queue operations.
 */
typedef enum
{
    QUEUE_OK = 0,
    QUEUE_FULL = -1,
    QUEUE_EMPTY = -2
} QueueStatus;

/**
 * @enum SoftTimer
 * @brief information used for software timers
 */
typedef struct
{
    uint32_t ms_left;
    bool active;
    bool finished;
} SoftTimer;

/**
 *  Full CPU context for an ARM Cortex-M core.
 *  This is saved/restored during context switches.
 *  Includes all general-purpose registers, SP and LR
 */
typedef struct
{
    uint32_t R0;  // R0
    uint32_t R1;  // R1
    uint32_t R2;  // R2
    uint32_t R3;  // R3
    uint32_t R4;  // R4
    uint32_t R5;  // R5
    uint32_t R6;  // R6
    uint32_t R7;  // R7
    uint32_t R8;  // R8
    uint32_t R9;  // R9
    uint32_t R10; // R10
    uint32_t R11; // R11
    uint32_t R12; // R12
    uint32_t SP;  // Reserved
    uint32_t LR;  // Link Register
} PROCESSOR_TCB;

/**
 *  Thread Control Block (TCB) for the scheduler.
 *  Holds CPU context, scheduling info, and thread metadata.
 */
typedef struct
{
    PROCESSOR_TCB context; // Saved CPU registers/context
    uint32_t *psp;         // Process Stack Pointer (top of saved frame)
    uint32_t *stackBase;   // Base of stack memory
    uint32_t stackSize;    // Stack size in bytes
    priority_t priority;   // Static priority
    uint8_t control;       // Saved CONTROL value (nPRIV, SPSEL, FPCA)
    uint8_t primask;
    uint8_t basepri;
    uint8_t faultmask;
    uint8_t usesFPU;          // If nonzero, manage FP context (not used)
    ThreadState state;        // Current thread state
    void (*entry)(void *arg); // Thread entry function
    void *arg;                // Argument to entry function
    uint32_t periodTicks;     // Period for periodic tasks
    uint32_t nextReleaseTick; // Next release time in ticks
    status_t exit_code;       // Exit Code
    Mutex *ownedMutexes[4];   // Mutexes List
    uint8_t ownedCount;       // How many Mutexes Held
    char name[8];             // Thread Name
    int semaphoreIndex;       // Index of Semaphore this thread is Waiting On
    uint8_t queueIndex;       // index into queueList[]
    SoftTimer timers[MAX_TIMERS_PER_THREAD];
} Thread;

typedef Thread thread_t;

// -----------------------------------------------------------------------------
// Scheduler API
// -----------------------------------------------------------------------------

/**
 * @brief Initialize the scheduler state and start it.
 *
 * Resets all scheduler bookkeeping in g_kernel, sets the current thread index
 * to zero, clears the current thread pointer, and then calls Start_Scheduler()
 * to begin execution.
 *
 * @note This function does not return; once the scheduler starts, control
 *       passes to the first scheduled thread.
 */
void Init_Scheduler(void);

/**
 * @brief Start the scheduler main loop.
 *
 * Picks the first thread from the ready list, marks it as running, configures
 * the SysTick timer for 1 ms ticks, and triggers a PendSV to perform the first
 * context switch. Then enters the idle loop, sleeping until interrupts occur.
 *
 * @note This function does not return under normal operation. Also this is auto called by `Init_Skeduler`
 */
void Start_Scheduler(void);

/**
 * @brief Create and register a new thread with the scheduler.
 *
 * Initializes the Thread control block, builds its initial stack frame so it
 * will start at the given entry point with the provided argument, and adds it
 * to the scheduler's ready list.
 *
 * @param t           Pointer to the Thread control block to initialize.
 * @param entry       Function pointer to the thread's entry function.
 * @param arg         Argument to pass to the entry function.
 * @param stack       Pointer to the base of the thread's stack memory.
 * @param stackBytes  Size of the stack in bytes.
 * @param priority    Thread priority (0 = lowest, higher = more urgent).
 */
void Create_Thread(thread_t *t, void (*entry)(void *), void *arg,
                   uint32_t *stack, size_t stackBytes, status_t priority, char *name[5]);

/**
 * @brief Voluntarily yield the CPU to another ready thread.
 *
 * Causes the scheduler to pend a context switch at the next opportunity,
 * allowing other threads of equal priority to run.
 */
void Yield(void);

/**
 * @brief Scheduler tick handler, called from SysTick_Handler().
 *
 * Increments the system tick counter, updates time slice counters for the
 * current thread, wakes periodic or sleeping threads whose release time has
 * arrived, and pends a context switch if the current thread's quantum expired.
 */
void Scheduler_Tick(void);

/**
 * @brief Select the next thread to run.
 *
 * Implements the scheduling policy to
 * choose the next ready thread from the scheduler's ready list.
 *
 * @return Pointer to the Thread control block of the next thread to run,
 *         or NULL if no threads are ready.
 */
thread_t *Scheduler_GetNextThread(void);

/**
 * @brief Exit Current Thread
 */
void Thread_Exit(status_t code);

/**
 * @brief Build the initial stack frame for a new thread.
 *
 * Sets up the stack so that when the scheduler restores it for the first time,
 * the CPU will return to Thread mode and begin executing entry(arg).
 *
 * @param t     Pointer to the Thread control block.
 * @param entry Function pointer to the thread's entry function.
 * @param arg   Argument to pass to the entry function.
 *
 * @note This is typically called only from Create_Thread().
 */
static void Init_Thread_Stack(Thread *t, void (*entry)(void *), void *arg);

static inline void Save_Context(Thread *t);
static inline void Restore_Context(Thread *t);

/**
 * @brief Put the current thread to sleep for a given number of milliseconds.
 *
 * Marks the current thread as sleeping and sets its wake-up tick based on
 * the current system tick count. The scheduler will not run this thread
 * again until the specified delay has elapsed.
 *
 * @param ms Number of milliseconds to sleep.
 */
void Thread_Sleep(time_t ms);

typedef struct
{
    SPI_HandleTypeDef *hspi;
    uint8_t *txData;
    uint8_t *rxData;
    uint16_t size;
    uint32_t timeout;
} SPI_Args;

__attribute__((always_inline)) static inline HAL_StatusTypeDef SPI_Transmit(SPI_Args *args)
{
    HAL_StatusTypeDef result;
    __asm volatile(
        "mov r0, %1   \n"
        "svc %1       \n"
        "mov %0, r0   \n"
        : "=r"(result)
        : "I"(SVC_SPI_TRANSMIT), "r"(args)
        : "r0", "r1", "r2", "r3", "r12", "lr", "memory");
    return result;
}

/**
 * @brief Receive data over SPI in an RTOS-safe way.
 * @param pData Pointer to buffer to store received data.
 * @param Size Number of bytes to receive.
 * @param Timeout Timeout duration in milliseconds.
 * @return HAL status.
 */
__attribute__((always_inline)) static inline HAL_StatusTypeDef SPI_Receive(SPI_Args *args)
{
    HAL_StatusTypeDef result;
    __asm volatile(
        "mov r0, %1   \n"
        "svc %1       \n"
        "mov %0, r0   \n"
        : "=r"(result)
        : "I"(SVC_SPI_RECEIVE), "r"(args)
        : "r0", "r1", "r2", "r3", "r12", "lr", "memory");
    return result;
}

/**
 * @brief Transmit and receive data over SPI (full-duplex) in an RTOS-safe way.
 * @param pTxData Pointer to transmit buffer.
 * @param pRxData Pointer to receive buffer.
 * @param Size Number of bytes to transmit/receive.
 * @param Timeout Timeout duration in milliseconds.
 * @return HAL status.
 */
__attribute__((always_inline)) static inline HAL_StatusTypeDef SPI_TransmitReceive(SPI_Args *args)
{
    HAL_StatusTypeDef result;
    __asm volatile(
        "mov r0, %1   \n"
        "svc %1       \n"
        "mov %0, r0   \n"
        : "=r"(result)
        : "I"(SVC_SPI_TRANSMITRECV), "r"(args)
        : "r0", "r1", "r2", "r3", "r12", "lr", "memory");
    return result;
}

/**
 * @brief Write a logic state to a GPIO pin.
 */
void GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);

/**
 * @brief Read the logic state of a GPIO pin.
 */
GPIO_PinState GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief UART argument structure.
 */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *pData;
    uint16_t Size;
    uint32_t Timeout;
} UART_Args;

/**
 * @brief Transmit data over UART.
 */
HAL_StatusTypeDef UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
                                uint16_t Size, uint32_t Timeout);

/**
 * @brief Receive data over UART.
 */
HAL_StatusTypeDef UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData,
                               uint16_t Size, uint32_t Timeout);

/**
 * @brief I2C argument structure.
 */
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

/**
 * @brief Perform an I2C master transmit + receive transaction.
 */
HAL_StatusTypeDef I2C_Master_TransmitReceive(I2C_HandleTypeDef *hi2c,
                                             uint16_t DevAddress,
                                             uint8_t *pTxData, uint16_t TxSize,
                                             uint8_t *pRxData, uint16_t RxSize,
                                             uint32_t Timeout);

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t mode;
    uint32_t pull;
    uint32_t speed;
} GPIO_NewArgs;

typedef struct
{
    USART_TypeDef *instance;
    uint32_t baudrate;
    uint32_t wordLength;
    uint32_t stopBits;
    uint32_t parity;
    uint32_t mode;
} UART_NewArgs;

typedef struct
{
    I2C_TypeDef *instance;
    uint32_t timing;
    uint32_t addressingMode;
    uint32_t ownAddress;
} I2C_NewArgs;

typedef struct
{
    SPI_TypeDef *instance;
    uint32_t mode;
    uint32_t direction;
    uint32_t datasize;
    uint32_t clkpolarity;
    uint32_t clkphase;
    uint32_t nss;
    uint32_t baudratePrescaler;
    uint32_t firstBit;
} SPI_NewArgs;

typedef struct
{
    MessageQueue *q;
    void *buffer;
    uint16_t msgSize;
    uint16_t capacity;
} Queue_CreateArgs;

/**
 * @brief Create and initialize a new GPIO pin.
 * @param port GPIO port base (GPIOA, GPIOB, ...).
 * @param pin  Pin mask (GPIO_PIN_x).
 * @param mode GPIO mode (e.g. GPIO_MODE_OUTPUT_PP).
 * @param pull Pull-up/down config (GPIO_NOPULL, GPIO_PULLUP, ...).
 * @param speed Speed config (GPIO_SPEED_FREQ_LOW, ...).
 */
void GPIO_NEW(GPIO_TypeDef *port, uint16_t pin,
              uint32_t mode, uint32_t pull, uint32_t speed);

/**
 * @brief Create and initialize a new UART handle.
 * @param instance UART instance (e.g. USART1, USART2).
 * @param baudrate Baud rate (e.g. 115200).
 * @param wordLength Word length (UART_WORDLENGTH_8B, ...).
 * @param stopBits Stop bits (UART_STOPBITS_1, ...).
 * @param parity Parity (UART_PARITY_NONE, ...).
 * @param mode Mode (UART_MODE_TX_RX, ...).
 * @return Pointer to initialized UART handle.
 */
UART_HandleTypeDef *UART_NEW(USART_TypeDef *instance,
                             uint32_t baudrate,
                             uint32_t wordLength,
                             uint32_t stopBits,
                             uint32_t parity,
                             uint32_t mode);

/**
 * @brief Create and initialize a new I2C handle.
 * @param instance I2C instance (I2C1, I2C2, ...).
 * @param timing Timing register value (from CubeMX or datasheet).
 * @param addressingMode Addressing mode (I2C_ADDRESSINGMODE_7BIT, ...).
 * @param ownAddress Own address (if slave mode).
 * @return Pointer to initialized I2C handle.
 */
I2C_HandleTypeDef *I2C_NEW(I2C_TypeDef *instance,
                           uint32_t timing,
                           uint32_t addressingMode,
                           uint32_t ownAddress);

/**
 * @brief Create and initialize a new SPI handle.
 * @param instance SPI instance (SPI1, SPI2, ...).
 * @param mode SPI mode (SPI_MODE_MASTER or SPI_MODE_SLAVE).
 * @param direction SPI direction (SPI_DIRECTION_2LINES, ...).
 * @param datasize Data size (SPI_DATASIZE_8BIT, ...).
 * @param clkpolarity Clock polarity (SPI_POLARITY_LOW/HIGH).
 * @param clkphase Clock phase (SPI_PHASE_1EDGE/2EDGE).
 * @param nss NSS management (SPI_NSS_SOFT/HARD_INPUT/HARD_OUTPUT).
 * @param baudratePrescaler Prescaler (SPI_BAUDRATEPRESCALER_x).
 * @param firstBit First bit (SPI_FIRSTBIT_MSB/LSB).
 * @return Pointer to initialized SPI handle.
 */
SPI_HandleTypeDef *SPI_NEW(SPI_TypeDef *instance,
                           uint32_t mode,
                           uint32_t direction,
                           uint32_t datasize,
                           uint32_t clkpolarity,
                           uint32_t clkphase,
                           uint32_t nss,
                           uint32_t baudratePrescaler,
                           uint32_t firstBit);

/**
 * @brief Get the current system tick count.
 *
 * This function returns the current kernel tick counter value.
 * The tick counter is incremented by the system timer interrupt
 * (SysTick) every millisecond.
 *
 * @note This function can be called from any thread. In unprivileged
 *       mode, it will invoke a system call (SVC) to retrieve the value
 *       from the kernel.
 *
 * @return uint32_t The current tick count in kernel ticks.
 *                  Units depend on the system tick frequency.
 *
 * @see Thread_Sleep
 * @see Kernel_GetTick
 */
time_t OS_GetTick(void);

/**
 * @brief Create a unlocked mutex onto the provideded task
 * @param maker A Pointer to The thread that should Own it
 */
Mutex *Mutex_Create(Thread *maker);

/**
 * @brief Lock a Mutex
 * @param m pointer to the Mutex
 */
void Mutex_Lock(Mutex *m);

/**
 * @brief Unlock a Mutex
 * @param m pointer to the Mutex
 */
void Mutex_Unlock(Mutex *m);

/**
 * @brief Create and initialize a message queue.
 *
 * The user must supply the queue object and a backing buffer.
 * The buffer must be at least (msgSize * capacity) bytes.
 *
 * @param q         Pointer to a MessageQueue object.
 * @param buffer    Raw storage for messages.
 * @param msgSize   Size of each message in bytes.
 * @param capacity  Maximum number of messages the queue can hold.
 */
void Queue_Create(MessageQueue *q, void *buffer,
                  uint16_t msgSize, uint16_t capacity);

/**
 * @brief Send a message to a queue.
 *
 * If the queue is full, the calling thread will block until space is available.
 *
 * @param q     Pointer to the queue.
 * @param msg   Pointer to the message to send.
 */
void Queue_Send(MessageQueue *q, const void *msg);

/**
 * @brief Receive a message from a queue.
 *
 * If the queue is empty, the calling thread will block until a message arrives.
 *
 * @param q        Pointer to the queue.
 * @param msgOut   Pointer to a buffer where the message will be copied.
 */
void Queue_Receive(MessageQueue *q, void *msgOut);

/**
 * @brief Attempt to send a message to the queue (non‑blocking).
 *
 * This is a user‑mode wrapper that triggers an SVC call.
 *
 * @param q     Pointer to the queue.
 * @param msg   Pointer to the message data to send.
 *
 * @return QUEUE_OK on success, or QUEUE_FULL if the queue is full.
 */
QueueStatus Queue_TrySend(MessageQueue *q, const void *msg);

/**
 * @brief Attempt to receive a message from the queue (non‑blocking).
 *
 * This is a user‑mode wrapper that triggers an SVC call.
 *
 * @param q         Pointer to the queue.
 * @param msgOut    Pointer to a buffer where the message will be copied.
 *
 * @return QUEUE_OK on success, or QUEUE_EMPTY if the queue is empty.
 */
QueueStatus Queue_TryReceive(MessageQueue *q, void *msgOut);

/**
 * @brief Create a new software timer for the calling thread.
 *
 * The timer begins counting down immediately from the specified duration.
 * Timers are per-thread; each thread may have up to MAX_TIMERS_PER_THREAD timers.
 *
 * @param ms Duration of the timer in milliseconds.
 *
 * @return uint8_t
 *         Timer ID (0–3) on success.
 *         0xFF if no timer slots are available.
 *
 * @note This function does not block. The timer runs in the background.
 * @note The timer must be polled using Timer_IsDone().
 */
uint8_t Timer_Create(uint32_t ms);

/**
 * @brief Check whether a previously created timer has finished.
 *
 * @param timerId The ID returned by Timer_Create().
 *
 * @return true  if the timer has expired.
 * @return false if the timer is still running or the ID is invalid.
 *
 * @note This function does not block.
 * @note Once a timer is finished, it remains finished until reused.
 */
bool Timer_IsDone(uint8_t timerId);

/**
 * @brief Reset an existing timer to a new duration.
 *
 * The timer begins counting down immediately from the new value.
 *
 * @param timerId The ID returned by Timer_Create().
 * @param ms      New duration in milliseconds.
 *
 * @return true  if the timer was valid and reset.
 * @return false if the timerId is invalid or the timer is inactive.
 */
bool Timer_Reset(uint8_t timerId, uint32_t ms);

/**
 * @brief Cancel a running or finished timer.
 *
 * The timer slot becomes free and may be reused by Timer_Create().
 *
 * @param timerId The ID returned by Timer_Create().
 *
 * @return true  if the timer was valid and cancelled.
 * @return false if the timerId is invalid.
 */
bool Timer_Cancel(uint8_t timerId);

/**
 * @brief Get the remaining time of a timer.
 *
 * @param timerId The ID returned by Timer_Create().
 *
 * @return uint32_t Remaining milliseconds.
 *         Returns 0 if the timer is finished, inactive, or invalid.
 */
uint32_t Timer_Remaining(uint8_t timerId);

/**
 * @brief Allocates a block of memory from the kernel heap.
 *
 * This is a system call wrapper around the kernel's memory allocator.
 * Currently it forwards directly to libc malloc(), but the backend
 * can be replaced later without changing user code.
 *
 * @param size Number of bytes to allocate.
 * @return Pointer to allocated memory, or NULL on failure.
 */
void *Malloc(size_t size);

/**
 * @brief Frees a block of memory previously allocated with kmalloc/kcalloc/krealloc.
 *
 * This is a system call wrapper around the kernel's memory free routine.
 * Currently it forwards directly to libc free().
 *
 * @param ptr Pointer to memory to free. NULL is ignored.
 */
void Free(void *ptr);

/**
 * @brief Allocates zero‑initialized memory from the kernel heap.
 *
 * This is a system call wrapper around the kernel's calloc implementation.
 * Currently it forwards directly to libc calloc().
 *
 * @param n Number of elements.
 * @param size Size of each element in bytes.
 * @return Pointer to allocated memory, or NULL on failure.
 */
void *Calloc(size_t n, size_t size);

/**
 * @brief Resizes a previously allocated memory block.
 *
 * This is a system call wrapper around the kernel's realloc implementation.
 * Currently it forwards directly to libc realloc().
 *
 * @param ptr Pointer to previously allocated memory (may be NULL).
 * @param newSize New size in bytes.
 * @return Pointer to resized memory block, or NULL on failure.
 */
void *Realloc(void *ptr, size_t newSize);

// -----------------------------------------------------------------------------
// User-Facing FileSystem API
// -----------------------------------------------------------------------------

API_FUNCTION(FS_Open)
/**
 * @brief Open a file or resource at the given path.
 * @param path Path to the file.
 * @param flags Access mode flags.
 * @return File descriptor, or negative on error.
 */
int FS_Open(const char *path, int flags);

API_FUNCTION(FS_Close)
/**
 * @brief Close an open file descriptor.
 * @param fd File descriptor to close.
 * @return 0 on success, negative on error.
 */
int FS_Close(int fd);

API_FUNCTION(FS_Read)
/**
 * @brief Read data from an open file.
 * @param fd File descriptor.
 * @param buffer Destination buffer.
 * @param size Maximum number of bytes to read.
 * @return Number of bytes read, or negative on error.
 */
int FS_Read(int fd, void *buffer, int size);

API_FUNCTION(FS_Write)
/**
 * @brief Write data to an open file.
 * @param fd File descriptor.
 * @param buffer Source buffer.
 * @param size Number of bytes to write.
 * @return Number of bytes written, or negative on error.
 */
int FS_Write(int fd, const void *buffer, int size);

API_FUNCTION(FS_List)
/**
 * @brief List directory contents into a buffer.
 * @param path Directory path.
 * @param outBuffer Output buffer for listing.
 * @param maxLen Maximum buffer length.
 * @return Number of bytes written, or negative on error.
 */
int FS_List(const char *path, char *outBuffer, int maxLen);

API_FUNCTION(VFS_RegisterDriver)
/**
 * @brief Register a filesystem driver with the VFS.
 * @param driver Pointer to a FileSystemDriver structure.
 * @return 0 on success, negative on error.
 */
int VFS_RegisterDriver(FileSystemDriver *driver);

// How Many Milliseconds since boot
static inline time_t OS_runtimeMS(void) { return OS_GetTick(); }

// Here so that HAL can proplery get ticks , uses the kernal function since HAL has privalges.

inline uint32_t HAL_GetTick()
{
    return Kernel_GetTick();
}

extern "C"
{
    /** @internal @short Do Not Use unless in kernal. */
    static inline uint32_t Kernel_GetTick(void);
    /** @internal @short Do Not Use unless in kernal. */
    inline void Kernal_Create_Thread(Thread *t, void (*entry)(void *), void *arg,
                                     uint32_t *stack, uint32_t stackBytes, status_t priority);
    /** @internal @short Do Not Use unless in kernal */
    void Kernal_Wipe_Thread(Thread *t);
}

#endif // THREAD_H

#ifndef SPECIAL_STUFF
#define SPECIAL_STUFF

// PACKED macro
#if defined(__GNUC__) || defined(__clang__)
#define PACKED __attribute__((packed))
#elif defined(__ICCARM__) // IAR
#define PACKED __packed
#elif defined(__ARMCC_VERSION) // ARM Compiler 5/6
#define PACKED __attribute__((packed))
#else
#define PACKED
#endif

// FASTFUNC macro
#if defined(STM32F479xx)
#if defined(__GNUC__) || defined(__clang__)
#define FASTFUNC __attribute__((section(".ccmram")))
#elif defined(__ICCARM__)
#define FASTFUNC @ ".ccmram"
#elif defined(__ARMCC_VERSION)
#define FASTFUNC __attribute__((section(".ccmram")))
#else
#define FASTFUNC
#endif
#else
#if defined(__GNUC__) || defined(__clang__)
#define FASTFUNC __attribute__((section(".ramfunc")))
#elif defined(__ICCARM__)
#define FASTFUNC @ ".ramfunc"
#elif defined(__ARMCC_VERSION)
#define FASTFUNC __attribute__((section(".ramfunc")))
#else
#define FASTFUNC
#endif
#endif

#endif // SPECIAL_STUFF