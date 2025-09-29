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

#ifndef THREAD_H
#define THREAD_H

#include <stdint.h>
#include "../include/CMSIS/stm32f4xx_hal_gpio.h"
#include "../include/CMSIS/stm32f4xx_hal_uart.h"
#include "../include/CMSIS/stm32f4xx_hal_spi.h"
#include "../include/CMSIS/core_cm4.h"
#include "../include/mustinclude.h" // Include Reqiured Files (Files unused , nut if not included compiler will complain about undefined symbols)

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

// Time & durations
typedef uint32_t time_t;    // absolute time in ms since boot (1 tick = 1 ms)
typedef int32_t duration_t; // signed interval in ms (end - start)

// Scheduling & priorities
typedef int priority_t; // thread priority level
typedef uint32_t tick_t;    // tick count

// Flags & masks
typedef uint32_t flag32_t; // 32-bit flags
typedef uint64_t flag64_t; // 64-bit flags
typedef uint16_t flag16_t; // 16-bit flags

// Error/status codes
typedef int status_t; // return codes from threads/OS functions in the future.

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
} ThreadState;

enum
{
    // Thread/Process services
    SVC_THREAD_SLEEP = 1,
    SVC_YIELD = 2,
    SVC_CREATE_THREAD = 3,
    SVC_ADD_PROCESS = 4,
    SVC_EXIT = 5,

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

};

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
    uint32_t SP;  // Process Stack Pointer
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
    uint8_t usesFPU;          // If nonzero, manage FP context
    ThreadState state;        // Current thread state
    void (*entry)(void *arg); // Thread entry function
    void *arg;                // Argument to entry function
    uint32_t periodTicks;     // Period for periodic tasks
    uint32_t nextReleaseTick; // Next release time in ticks
    status_t exit_code;       // Exit Code
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
                   uint32_t *stack, size_t stackBytes, status_t priority);

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


extern "C" {
    
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

}

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
        "svc %1       \n"
        "mov %0, r0   \n"
        : "=r"(result)
        : "I"(SVC_SPI_TRANSMIT), "r"(args)
        : "r0");
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
        "svc %1       \n"
        "mov %0, r0   \n"
        : "=r"(result)
        : "I"(SVC_SPI_RECEIVE), "r"(args)
        : "r0");
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
        "svc %1       \n"
        "mov %0, r0   \n"
        : "=r"(result)
        : "I"(SVC_SPI_TRANSMITRECV), "r"(args)
        : "r0");
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


typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t mode;
    uint32_t pull;
    uint32_t speed;
} GPIO_NewArgs;

typedef struct {
    USART_TypeDef *instance;
    uint32_t baudrate;
    uint32_t wordLength;
    uint32_t stopBits;
    uint32_t parity;
    uint32_t mode;
} UART_NewArgs;

typedef struct {
    I2C_TypeDef *instance;
    uint32_t timing;
    uint32_t addressingMode;
    uint32_t ownAddress;
} I2C_NewArgs;

typedef struct {
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

// How Many Milliseconds since boot
static inline time_t OS_runtimeMS(void) { return OS_GetTick(); }

// Here so that HAL can proplery get ticks.

inline uint32_t HAL_GetTick() {
    return OS_GetTick();
}

#endif // THREAD_H