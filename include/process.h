#ifndef PROCESS_H
#define PROCESS_H

#include <stdint.h>
#include "thread.h"

#define __FPU_PRESENT 1U

typedef struct
{
    uint32_t pid;        // Process ID
    Thread *mainThread;  // Main thread of the process
    void *memoryRegion;  // Pointer to process memory (if needed)
    uint32_t memorySize; // Size of allocated memory
} Process;

// ---- Kernel limits ----
/**
 * @brief Maximum number of processes the kernel can track at once.
 * @note Adjust based on available RAM and application needs.
 */
#define MAX_PROCESSES  8

/**
 * @brief Maximum number of threads the kernel can track at once.
 * @note Includes all threads across all processes.
 */
#define MAX_THREADS    8

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
    uint32_t systemTicks;
    // --------------------
    // Process management
    // --------------------
    Process processTable[MAX_PROCESSES];
    uint8_t processCount;

    // Map process slot -> assigned MPU region index (0xFF means unassigned)
    uint8_t procRegionMap[MAX_PROCESSES];

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

void Add_Process(Process process);
void Remove_Process(Process process);

// -------------------
//  Internal Use Only
// -------------------
void Kernal_Add_Process(Process *process);
void Kernal_Remove_Process(Process *process);

#endif // PROCESS_H