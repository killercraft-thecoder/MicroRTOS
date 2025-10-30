#ifndef PROCESS_H
#define PROCESS_H

#include <stdint.h>
#include "thread.h"

#define __FPU_PRESENT 1U

#if defined(__GNUC__) || defined(__clang__)
#define ALIGNED(X) __attribute__((aligned(X)))
#elif defined(_MSC_VER)
#define ALIGNED(X) __declspec(align(X))
#else
#warning "ALIGNED(X) is not supported on this compiler"
#define ALIGNED(X) /* alignment not supported */
#endif

typedef struct
{
    uint32_t pid;        // Process ID
    Thread *mainThread;  // Main thread of the process
    uint32_t memorySize; // Size of allocated memory
} Process;

// ---- Kernel limits ----
/**
 * @brief Maximum number of processes the kernel can track at once.
 * @note Adjust based on available RAM and application needs.
 */
#define MAX_PROCESSES 8

/**
 * @brief Maximum number of threads the kernel can track at once.
 * @note Includes all threads across all processes.
 */
#define MAX_THREADS 8

// Forward declaration to avoid circular include
struct Thread;

// -----------------------------------------------------------------------------
// Contiguous OS data block (no linker script needed)
// -----------------------------------------------------------------------------
typedef struct
{
    // -------------------
    //  System Tick Managment
    // -------------------
    volatile tick_t systemTicks;
    // --------------------
    // Process management
    // --------------------
    volatile Process processTable[MAX_PROCESSES];
    volatile uint8_t processCount;

    // --------------------
    // Scheduler state
    // --------------------
    Thread *threadList[MAX_THREADS]; // All threads known to the scheduler
    uint8_t threadCount;             // Number of active threads
    uint8_t currentIndex;            // Index of currently running thread
    Thread *currentThread;           // Pointer to currently running thread

} KernelData;

// Declare the global kernel data (no storage here)
extern KernelData g_kernel;

// Check for invalid combinations
#if defined(ENABLE_GLOBAL_DATA) && defined(ENABLE_SMALL_GLOBAL_DATA)
#error "You cannot define both ENABLE_GLOBAL_DATA and ENABLE_SMALL_GLOBAL_DATA"
#endif

#ifdef ENABLE_GLOBAL_DATA

typedef struct
{
    // 32-bit general-purpose flags — bit meanings are up to user code
    volatile uint32_t flags;

    // 32-bit timestamp or counter (can be OS_GetTick, downtime,etc)
    volatile time_t timestamp;

    // 64-bit general-purpose data slots
    volatile uint64_t data64[4]; // 4 slots for arbitrary 64-bit values

    // 32-bit general-purpose data slots
    volatile uint32_t data32[8]; // 8 slots for arbitrary 32-bit values

    // 16-bit general-purpose data slots
    volatile uint16_t data16[8]; // 8 slots for arbitrary 16-bit values

    // 8-bit general-purpose data slots
    volatile uint8_t data8[16]; // 16 slots for arbitrary 8-bit values

} OS_GlobalShared;

// Declare the global instance
extern OS_GlobalShared g_shared;

#elif defined(ENABLE_SMALL_GLOBAL_DATA)

typedef struct
{
    /**  32-bit general-purpose flags — bit meanings are up to user code */
    volatile uint16_t flags;
    /** 32-bit counter */
    volatile uint32_t counter;
    /** 32-bit general-purpose data slots */
    volatile uint32_t data32[4];
} OS_GlobalShared;

// Declare the global instance
extern OS_GlobalShared g_shared;

#endif

void Add_Process(Process process);
void Remove_Process(Process process);

// -------------------
//  Internal Use Only
// -------------------
void Kernal_Add_Process(Process *process);
void Kernal_Remove_Process(Process *process);

#endif // PROCESS_H