#include <process.h>
#include <thread.h>
#include <mpu.h>

#include <stddef.h>
#include <string.h>
#include <CMSIS/stm32f4xx_hal_def.h>

// CMSIS core
#include <CMSIS/core_cm4.h>
#include <CMSIS/mpu_armv7.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

#ifndef MAX_PROCESSES
#define MAX_PROCESSES 8
#endif

// Reserve MPU region 0 for OS data, process regions start from here
#define OS_MPU_REGION_INDEX 0u
#define FIRST_PROCESS_REGION 1u

// Minimum MPU alignment is 32 bytes; align the whole OS data block.
static ALIGNED(32) KernelData g_kernel;
#ifdef ENABLE_GLOBAL_DATA
OS_GlobalShared g_shared = {0}; // Zero-initialized
#elif defined(ENABLE_SMALL_GLOBAL_DATA)
OS_GlobalShared g_shared = {0}; // Zero-initialized
#endif

// Cached hardware MPU region count and one-time OS MPU config flag
static uint32_t g_mpuRegionCount = 0;
static uint8_t g_osMpuConfigured = 0;

// Disable a specific MPU region index
static inline void DisableRegion(uint8_t regionNumber)
{
    MPU->RNR = regionNumber & 0xFFu;
    MPU->RASR = 0u; // clear ENABLE
}

// -----------------------------------------------------------------------------
// API implementation
// -----------------------------------------------------------------------------

static inline void Add_Process(Process *p)
{
    register Process *r0 __asm__("r0") = p;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_ADD_PROCESS) : "memory");
}

static inline void Remove_Process(Process *p)
{
    register Process *r0 __asm__("r0") = p;
    __asm volatile("svc %[imm]" ::"r"(r0), [imm] "I"(SVC_REMOVE_PROCESS) : "memory");
}
KERNAL_FUNCTION
void Kernal_Add_Process(Process *process)
{

    // Bounds check
    if (g_kernel.processCount >= MAX_PROCESSES)
    {
        return;
    }

    // Auto-assign PID if zero
    if (process->pid == 0)
    {
        static uint32_t nextPid = 1;
        process->pid = nextPid++;
    }

    // Determine slot and initialize mapping
    uint8_t slot = g_kernel.processCount;

    // Store process in OS table (by value copy)
    g_kernel.processTable[slot] = *process;
    g_kernel.processCount++;

    // If the process has a main thread, register it with the scheduler
    if (process->mainThread != NULL)
    {
        Kernal_Create_Thread(
            process->mainThread,
            process->mainThread->entry,
            process->mainThread->arg,
            process->mainThread->stackBase,
            process->mainThread->stackSize,
            process->mainThread->priority);
    }
}
KERNAL_FUNCTION
void Kernal_Remove_Process(Process *process)
{
    // Find the process by PID
    for (uint8_t i = 0; i < g_kernel.processCount; ++i)
    {
        if (g_kernel.processTable[i].pid == process->pid)
        {

            // TODO: Remove process threads from scheduler if it later track's ownership

            // Compact the table and region map
            for (uint8_t j = i; j < (g_kernel.processCount - 1); ++j)
            {
                g_kernel.processTable[j] = g_kernel.processTable[j + 1];
            }
            g_kernel.processCount--;
            break;
        }
    }
}