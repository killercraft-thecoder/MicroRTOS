#ifndef PROCESS_H
#define PROCESS_H

#include <stdint.h>
#include "thread.h"
#include "tlsf.h"

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

/** 
 * @brief Maximum number of semaphores the kernal can track at once
 * @note semaphores are shared globally
*/
#define MAX_SEMAPHORES 24

// Forward declaration to avoid circular include
struct Thread;
struct Semaphore;
struct MessageQueue;


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
    Process processTable[MAX_PROCESSES];
    uint8_t processCount;

    // --------------------
    // Scheduler state
    // --------------------
    Thread *threadList[MAX_THREADS]; // All threads known to the scheduler
    uint8_t threadCount;             // Number of active threads
    uint8_t currentIndex;            // Index of currently running thread
    Thread *currentThread;           // Pointer to currently running thread

    // Semaphores
    Semaphore semaphoreList[MAX_SEMAPHORES]; // list of semaphores
    int seamphoreBitMask; // bit mask to find free semaphores

    // Message Queues
    MessageQueue queueList[MAX_QUEUES];
    uint8_t queueCount;

    // -------------------
    // TLSF Allocator
    // -------------------
    TlsfControl tlsf;




} KernelData;

// Declare the global kernel data (no storage here)
extern KernelData g_kernel;

void Add_Process(Process process);
void Remove_Process(Process process);

// -------------------
//  Internal Use Only
// -------------------
void Kernal_Add_Process(Process *process);
void Kernal_Remove_Process(Process *process);

#endif // PROCESS_H