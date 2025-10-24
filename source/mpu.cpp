#include "../include/CMSIS/mpu_armv7.h"
#include "../include/CMSIS/core_cm4.h"  // or core_cm3.h / core_cm7.h depending on MCU
#include "../include/CMSIS/cmsis_gcc.h" // or cmsis_armclang.h / cmsis_iccarm.h
#include "../include/process.h"
#include "../include/thread.h"

extern "C" {
    void Kernal_Yield();
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

// Build a common attribute set for normal SRAM: cacheable, bufferable, shareable
static inline uint32_t mpu_attr_normal_mem_rw_noexec_priv_full_unpriv_full(void)
{
    return (0x3u << MPU_RASR_AP_Pos) | // AP=3: Priv RW/Unpriv RW
           (1u << MPU_RASR_XN_Pos) |   // XN=1: no execute
           (0u << MPU_RASR_TEX_Pos) |
           (1u << MPU_RASR_C_Pos) | // Cacheable
           (1u << MPU_RASR_B_Pos) | // Bufferable
           (1u << MPU_RASR_S_Pos);  // Shareable
}

// OS data must be privileged-only (kernel/ISRs), not accessible from unprivileged threads
static inline uint32_t mpu_attr_os_data_priv_rw_unpriv_none_noexec(void)
{
    return (0x1u << MPU_RASR_AP_Pos) | // AP=1: Priv RW / Unpriv No Access
           (1u << MPU_RASR_XN_Pos) |   // XN=1: no execute
           (0u << MPU_RASR_TEX_Pos) |
           (1u << MPU_RASR_C_Pos) |
           (1u << MPU_RASR_B_Pos) |
           (1u << MPU_RASR_S_Pos);
}

// OS code in Flash: execute-only for unprivileged
static inline uint32_t mpu_attr_os_flash_priv_rw_unpriv_none_exec(void)
{
    return (0x1u << MPU_RASR_AP_Pos) | // AP=1: Priv RW / Unpriv No Access (data)
           (0u   << MPU_RASR_XN_Pos) | // XN=0: executable
           (0u   << MPU_RASR_TEX_Pos) |
           (1u   << MPU_RASR_C_Pos)  | // Cacheable
           (1u   << MPU_RASR_B_Pos)  | // Bufferable
           (0u   << MPU_RASR_S_Pos);   // Non-shareable (Flash is not shareable)
}

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

inline __attribute__((weak)) int ARM_MPU_GetRegionCount() {
    return 8;
}

// ---- Public API ----

inline uint32_t MPU_GetRegionCount(void)
{
    return ARM_MPU_GetRegionCount();
}

extern uint32_t __kernel_text_start__, __kernel_text_end__;

KERNAL_FUNCTION
void MPU_AddKernelFlashExecuteOnly(void)
{
    // Compute aligned base/size per MPU rules
    uint32_t start      = (uint32_t)&__kernel_text_start__;
    uint32_t end        = (uint32_t)&__kernel_text_end__;
    uint32_t span       = end - start;
    uint32_t size_pow2  = round_pow2_up(span);
    uint32_t base       = start & ~(size_pow2 - 1);
    uint32_t size_field = mpu_size_field(size_pow2);

    // Choose a region index that won’t be overridden; higher numbers win on overlap
    const uint32_t region = 7;

    // Program the region: executable, unprivileged data access denied
    ARM_MPU_SetRegionEx(
        region,
        base,
        ARM_MPU_RASR(
            0,          // executeNever = 0 -> executable
            mpu_attr_os_flash_priv_rw_unpriv_none_exec(),      // AP: Priv RW, Unpriv No Access (data reads/writes fault in unpriv)
            0,          // TEX
            0,          // Shareable (Flash typically non-shareable)
            1,          // Cacheable
            1,          // Bufferable
            size_field, // size encoding
            1           // enable
        )
    );

    // Optional: subregion disable (SRD) to trim spill if size_pow2 > span and >= 256B
    // MPU->RNR  = region;
    // MPU->RASR |= (srd_mask << MPU_RASR_SRD_Pos);

    __DSB(); __ISB();
}


KERNAL_FUNCTION
void MPU_Init(void)
{
    // Ensure memory operations complete before MPU config
    __DMB();
    __DSB();

    // Disable MPU before configuration
    ARM_MPU_Disable();

    // -------------------------------
    // Region 0: Kernel memory
    // -------------------------------
    ARM_MPU_SetRegionEx(
        0, // Region number
        (uint32_t)&g_kernel, // Base address
        ARM_MPU_RASR(
            0, // Disable execution (XN = 0)
            mpu_attr_os_data_priv_rw_unpriv_none_noexec(), // Access permission: Privileged RW, Unprivileged NA
            0, // Type extension field (TEX)
            0, // Shareable
            0, // Cacheable
            0, // Bufferable
            mpu_size_encoding(sizeof(g_kernel)), // Region size
            1  // Enable region
        )
    );

    MPU_AddKernelFlashExecuteOnly();

    // -------------------------------
    // Enable fault handlers
    // -------------------------------
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_USGFAULTENA_Msk;

    // Enable MPU with default privileged memory map
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    // Ensure MPU settings take effect
    __DSB();
    __ISB();
}


extern "C"
{
    // Memory Management Fault
    KERNAL_FUNCTION
    void MemManage_Handler(void)
    {
        Thread *t = g_kernel.currentThread;
        t->state = ThreadState::THREAD_HALTED;
        Kernal_Wipe_Thread(t);
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }

    // Bus Fault
    KERNAL_FUNCTION
    void BusFault_Handler(void)
    {
        Thread *t = g_kernel.currentThread;
        t->state = ThreadState::THREAD_HALTED;
        Kernal_Wipe_Thread(t);
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }

    // Usage Fault
    KERNAL_FUNCTION
    void UsageFault_Handler(void)
    {
        Thread *t = g_kernel.currentThread;
        t->state = ThreadState::THREAD_HALTED;
        Kernal_Wipe_Thread(t);
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }

    // Hard Fault
    KERNAL_FUNCTION
    void HardFault_Handler(void)
    {
        __asm volatile(
            "TST lr, #4            \n" // Test EXC_RETURN bit 2
            "ITE EQ                \n"
            "MRSEQ r0, MSP         \n" // Main Stack Pointer
            "MRSNE r0, PSP         \n" // Process Stack Pointer
            "B hardfault_c_handler \n");
    }
    KERNAL_FUNCTION
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
            if (g_kernel && g_kernel->currentThread)
            {
                g_kernel->currentThread->state = THREAD_HALTED;
                Kernal_Wipe_Thread(g_kernel->currentThread);
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
KERNAL_FUNCTION
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

inline void MPU_Enable(void)
{
    __DMB();
    __DSB();
    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
    __DSB();
    __ISB();
}

inline void MPU_Disable(void)
{
    __DMB();
    __DSB();
    MPU->CTRL = 0U;
    __DSB();
    __ISB();
}