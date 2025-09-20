#include "mpu.h"
#include "core_cm4.h"  // or core_cm3.h / core_cm7.h depending on MCU
#include "cmsis_gcc.h" // or cmsis_armclang.h / cmsis_iccarm.h
#include "include/process.h"
#include "thread.h"

// ---- Internal helpers ----

// Round up to next power of two >= 32, return SIZE field encoding for RASR
static inline uint32_t mpu_encode_size_field(uint32_t sizeBytes)
{
    if (sizeBytes < 32U)
        sizeBytes = 32U;

    // Round up to next power of two
    sizeBytes--;
    sizeBytes |= sizeBytes >> 1;
    sizeBytes |= sizeBytes >> 2;
    sizeBytes |= sizeBytes >> 4;
    sizeBytes |= sizeBytes >> 8;
    sizeBytes |= sizeBytes >> 16;
    sizeBytes++;

    // log2(sizeBytes) - 1
    uint32_t p = 0;
    uint32_t tmp = sizeBytes;
    while (tmp > 1U)
    {
        tmp >>= 1U;
        p++;
    }
    uint32_t SIZE = (p - 1U) & 0x1FU;
    return (SIZE << MPU_RASR_SIZE_Pos) & MPU_RASR_SIZE_Msk;
}

/**
 * @brief Convert a byte size to the MPU RASR SIZE field encoding.
 *
 * @param size_bytes Size of the region in bytes (must be a power of two, >= 32).
 * @return Encoded SIZE field value for MPU_RASR, or 0xFF if invalid.
 */
static inline uint32_t mpu_size_encoding(uint32_t size_bytes)
{
    // Minimum region size is 32 bytes on ARMv7-M
    if (size_bytes < 32U)
    {
        return 0xFFU; // invalid
    }

    // Must be a power of two
    if ((size_bytes & (size_bytes - 1U)) != 0U)
    {
        return 0xFFU; // invalid
    }

    // log2(size_bytes) - 1
    uint32_t log2_val = 0U;
    uint32_t tmp = size_bytes;
    while (tmp > 1U)
    {
        tmp >>= 1U;
        log2_val++;
    }

    return (log2_val - 1U) & 0x1FU; // SIZE field is 5 bits
}

// Align base address down to region size
static inline uint32_t mpu_align_base(uint32_t baseAddr, uint32_t sizeBytes)
{
    if (sizeBytes < 32U)
        sizeBytes = 32U;

    // Round up to next power of two
    uint32_t sz = sizeBytes - 1U;
    sz |= sz >> 1;
    sz |= sz >> 2;
    sz |= sz >> 4;
    sz |= sz >> 8;
    sz |= sz >> 16;
    sz += 1U;

    return baseAddr & ~(sz - 1U);
}

// ---- Public API ----

uint32_t MPU_GetRegionCount(void)
{
    return ((MPU->TYPE >> 8) & 0xFFU);
}

void MPU_Init(void)
{
    __DMB();
    __DSB();

    // Disable MPU for configuration
    MPU->CTRL = 0;

    // -------------------------------
    // Region 0: Kernel memory
    // -------------------------------
    MPU->RNR = 0;
    MPU->RBAR = ((uint32_t)&g_kernel & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | 0;
    MPU->RASR = (0x1U << MPU_RASR_AP_Pos) | // Priv RW, Unpriv NA
                (0U << MPU_RASR_XN_Pos) |   // Execution allowed
                (MPU_REGION_SIZE_32KB << MPU_RASR_SIZE_Pos) |
                (1U << MPU_RASR_ENABLE_Pos);

    // -------------------------------
    // Enable MemManage, BusFault, UsageFault
    // -------------------------------
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;

    // Enable MPU with default privileged map
    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;

    __DSB();
    __ISB();
}

extern "C"
{
    // Memory Management Fault
    void MemManage_Handler(void)
    {
        Thread *t = g_kernel.currentThread;
        t->state = ThreadState::THREAD_HALTED;
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }

    // Bus Fault
    void BusFault_Handler(void)
    {
        Thread *t = g_kernel.currentThread;
        t->state = ThreadState::THREAD_HALTED;
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }

    // Usage Fault
    void UsageFault_Handler(void)
    {
        Thread *t = g_kernel.currentThread;
        t->state = ThreadState::THREAD_HALTED;
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }

    // Hard Fault
    void HardFault_Handler(void)
    {
        __asm volatile(
            "TST lr, #4            \n" // Test EXC_RETURN bit 2
            "ITE EQ                \n"
            "MRSEQ r0, MSP         \n" // Main Stack Pointer
            "MRSNE r0, PSP         \n" // Process Stack Pointer
            "B hardfault_c_handler \n");
    }

    void hardfault_c_handler(uint32_t * /*stacked_regs*/)
    {
        uint32_t cfsr = SCB->CFSR;
        uint32_t hfsr = SCB->HFSR;

        bool is_usage_fault = (cfsr & 0xFFFF0000) != 0;
        bool is_bus_fault = (cfsr & 0x0000FF00) != 0;
        bool is_memmanage_fault = (cfsr & 0x000000FF) != 0;
        bool is_forced_hardfault = (hfsr & (1 << 30)) != 0;

        // If it's a recoverable fault, isolate the thread
        if (is_usage_fault || is_bus_fault || is_memmanage_fault || is_forced_hardfault)
        {
            if (g_kernal && g_kernal->currentThread)
            {
                g_kernal->currentThread->state = THREAD_HALTED;
                Kernal_Yield();
                return;
            }
        }

        // If it's not recoverable or no thread context, reset
        NVIC_SystemReset();
    }

    void DebugMon_Handler(void)
    {
        return; // TODO actually implement this handler.
    }
}

void MPU_ConfigureRegion(uint8_t regionNumber, const MPURegion *region)
{
    if (!region)
        return;
    uint32_t regions = MPU_GetRegionCount();
    if (regionNumber >= regions)
        return;

    uint32_t encodedSize = mpu_encode_size_field(region->size);
    uint32_t alignedBase = mpu_align_base(region->baseAddress, region->size);

    __DMB();
    __DSB();

    MPU->RNR = (uint32_t)regionNumber & 0xFFU;
    MPU->RBAR = alignedBase & MPU_RBAR_ADDR_Msk;

    uint32_t rasr = (region->attributes & ~(MPU_RASR_SIZE_Msk | MPU_RASR_ENABLE_Msk)) |
                    encodedSize |
                    MPU_RASR_ENABLE_Msk;

    MPU->RASR = rasr;

    __DSB();
    __ISB();
}

void MPU_Enable(void)
{
    __DMB();
    __DSB();
    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
    __DSB();
    __ISB();
}

void MPU_Disable(void)
{
    __DMB();
    __DSB();
    MPU->CTRL = 0U;
    __DSB();
    __ISB();
}