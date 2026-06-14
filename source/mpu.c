#include "CMSIS/mpu_armv7.h"
#include "CMSIS/core_cm4.h"
#include "process.h"
#include "thread.h"
#include "debug_printf.h"
#include "vfs.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// -----------------------------------------------------------------------------
// Fault trace buffer for post-mortem diagnostics
// -----------------------------------------------------------------------------
#define FAULT_TRACE_DEPTH 8

typedef struct
{
    uint32_t timestamp; // reserved: system tick if available
    uint32_t pc;
    uint32_t regs[8]; // r0,r1,r2,r3,r12,lr,pc,xpsr
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t bfar;
    uint32_t mmfar;
} FaultRecord;

static FaultRecord g_fault_trace[FAULT_TRACE_DEPTH];
static volatile uint32_t g_fault_trace_idx = 0;
static volatile uint32_t g_fault_file_counter = 0;

/* Forward declaration of dump helper (also exported in mpu.h) */
int Kernel_Dump_FaultTrace(char *outBuffer, int maxLen);

static void record_fault(uint32_t *stacked_regs, uint32_t cfsr, uint32_t hfsr)
{
    if (!stacked_regs)
        return;

    uint32_t idx;
    __disable_irq();
    idx = g_fault_trace_idx++ % FAULT_TRACE_DEPTH;
    __enable_irq();

    FaultRecord *r = &g_fault_trace[idx];
    r->timestamp = g_kernel.systemTicks;
    r->pc = stacked_regs[6];
    r->regs[0] = stacked_regs[0];
    r->regs[1] = stacked_regs[1];
    r->regs[2] = stacked_regs[2];
    r->regs[3] = stacked_regs[3];
    r->regs[4] = stacked_regs[4];
    r->regs[5] = stacked_regs[5];
    r->regs[6] = stacked_regs[6];
    r->regs[7] = stacked_regs[7];
    r->cfsr = cfsr;
    r->hfsr = hfsr;
    r->bfar = SCB->BFAR;
    r->mmfar = SCB->MMFAR;

#ifdef AUTOSAVE_FAULTS
    /* Attempt to autosave a textual dump of the fault via VFS. */
    char tmp_buf[512];
    int n = Kernel_Dump_FaultTrace(tmp_buf, sizeof(tmp_buf));
    if (n > 0)
    {
        /* Build filename: <cause>_<N>_<owner> */
        const char *cause = "Fault";
        if ((r->cfsr & 0x000000FFU) != 0)
            cause = "MemManage";
        else if ((r->cfsr & 0x0000FF00U) != 0)
            cause = "BusFault";
        else if ((r->cfsr & 0xFFFF0000U) != 0)
            cause = "UsageFault";
        else if ((r->hfsr & (1u << 30)) != 0)
            cause = "HardFault";

        char owner[32] = "unknown";
        if ((uint32_t)r->pc >= (uint32_t)&__kernel_text_start__ && (uint32_t)r->pc < (uint32_t)&__kernel_text_end__)
        {
            snprintf(owner, sizeof(owner), "kernel");
        }
        else if (g_kernel.currentThread)
        {
            /* Prefix thread name with capital 'T' */
            char tn[16] = {0};
            for (int i = 0; i < 8; i++)
                tn[i] = (char)g_kernel.currentThread->name[i];
            snprintf(owner, sizeof(owner), "T%.*s", 8, tn);
        }

        char fname[128];
        uint32_t id = __atomic_fetch_add(&g_fault_file_counter, 1, __ATOMIC_RELAXED);
        snprintf(fname, sizeof(fname), "/dbg/faultrecords/%s_%u_%s", cause, (unsigned)id, owner);

        int fd = VFS_Open(fname, 1);
        if (fd >= 0)
        {
            VFS_Write(fd, tmp_buf, n);
            VFS_Close(fd);
        }
    }
#endif
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
           (0u << MPU_RASR_XN_Pos) |   // XN=0: executable
           (0u << MPU_RASR_TEX_Pos) |
           (1u << MPU_RASR_C_Pos) | // Cacheable
           (1u << MPU_RASR_B_Pos) | // Bufferable
           (0u << MPU_RASR_S_Pos);  // Non-shareable (Flash is not shareable)
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

inline __attribute__((weak)) int ARM_MPU_GetRegionCount()
{
    return 8;
}

// ---- Public API ----

inline uint32_t MPU_GetRegionCount(void)
{
    return ARM_MPU_GetRegionCount();
}

extern uint32_t __kernel_text_start__, __kernel_text_end__;
extern uint32_t __heap_start__;
extern uint32_t __heap_end__;
extern uint32_t __stack_start__;
extern uint32_t __stack_end__;

KERNAL_FUNCTION
void MPU_AddKernelFlashExecuteOnly(void)
{
    // Compute aligned base/size per MPU rules
    uint32_t start = (uint32_t)&__kernel_text_start__;
    uint32_t end = (uint32_t)&__kernel_text_end__;
    uint32_t span = end - start;
    uint32_t size_pow2 = round_pow2_up(span);
    uint32_t base = start & ~(size_pow2 - 1);
    uint32_t size_field = mpu_size_field(size_pow2);

    // Choose a region index that won’t be overridden; higher numbers win on overlap
    const uint32_t region = 7;

    // Program the region: executable, unprivileged data access denied
    ARM_MPU_SetRegionEx(
        region,
        base,
        ARM_MPU_RASR(
            0,                                            // executeNever = 0 -> executable
            mpu_attr_os_flash_priv_rw_unpriv_none_exec(), // AP: Priv RW, Unpriv No Access (data reads/writes fault in unpriv)
            0,                                            // TEX
            0,                                            // Shareable (Flash typically non-shareable)
            1,                                            // Cacheable
            1,                                            // Bufferable
            size_field,                                   // size encoding
            1                                             // enable
            ));

    // Optional: subregion disable (SRD) to trim spill if size_pow2 > span and >= 256B
    // MPU->RNR  = region;
    // MPU->RASR |= (srd_mask << MPU_RASR_SRD_Pos);

    __DSB();
    __ISB();
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
        0,                   // Region number
        (uint32_t)&g_kernel, // Base address
        ARM_MPU_RASR(
            0,                                             // Disable execution (XN = 0)
            mpu_attr_os_data_priv_rw_unpriv_none_noexec(), // Access permission: Privileged RW, Unprivileged NA
            0,                                             // Type extension field (TEX)
            0,                                             // Shareable
            0,                                             // Cacheable
            0,                                             // Bufferable
            mpu_size_encoding(sizeof(g_kernel)),           // Region size
            1                                              // Enable region
            ));

    MPU_AddKernelFlashExecuteOnly();

    // -------------------------------
    // Region 1: User heap (RW, XN)
    // -------------------------------
    uint32_t heap_base = (uint32_t)&__heap_start__;
    uint32_t heap_size = (uint32_t)&__heap_end__ - (uint32_t)&__heap_start__;

    // heap_size must be rounded to a valid MPU size; assume mpu_size_encoding() handles that
    ARM_MPU_SetRegionEx(
        1,         // Region number
        heap_base, // Base address
        ARM_MPU_RASR(
            1,                                    // XN = 1 (no execute)
            mpu_attr_os_data_priv_rw_unpriv_rw(), // Priv RW, Unpriv RW
            0,                                    // TEX
            0,                                    // Shareable
            0,                                    // Cacheable
            0,                                    // Bufferable
            mpu_size_encoding(heap_size),         // Region size
            1                                     // Enable
            ));

            // -------------------------------
            // Region 2: Kernel stack (RW, XN, priv only)
            // -------------------------------
            uint32_t stack_base = (uint32_t)&__stack_start__;
            uint32_t stack_size = (uint32_t)&__stack_end__ - (uint32_t)&__stack_start__;
            
            ARM_MPU_SetRegionEx(
                2,                  // pick a free region index
                stack_base,
                ARM_MPU_RASR(
                    1,                                         // XN = 1 (no execute)
                    mpu_attr_os_data_priv_rw_unpriv_none_noexec(), // Priv RW, Unpriv NA, XN
                    0,
                    0,
                    0,
                    0,
                    mpu_size_encoding(stack_size),
                    1
                ));

    // -------------------------------
    // Enable fault handlers
    // -------------------------------
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_USGFAULTENA_Msk;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_MON_EN_Msk; // Enable DebugMon_Handler

    // Enable MPU with default privileged memory map
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    // Ensure MPU settings take effect
    __DSB();
    __ISB();
}

// Memory Management Fault
KERNAL_FUNCTION
void MemManage_Handler(void)
{
    /* Determine faulting PC from stacked frame to decide whether the fault
       originated in kernel code/data or in the current user thread. */
    uint32_t *stacked_ptr;
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq %0, msp\n"
        "mrsne %0, psp\n"
        : "=r"(stacked_ptr)
        :
        :);

    uint32_t fault_pc = stacked_ptr[6];

    bool in_kernel = false;
    if ((uint32_t)&__kernel_text_start__ <= fault_pc && fault_pc < (uint32_t)&__kernel_text_end__)
        in_kernel = true;
    if ((uint32_t)&g_kernel <= fault_pc && fault_pc < ((uint32_t)&g_kernel + sizeof(g_kernel)))
        in_kernel = true;
    if ((uint32_t)&__stack_start__ <= fault_pc && fault_pc < (uint32_t)&__stack_end__)
        in_kernel = true;

    if (in_kernel)
    {
        uint32_t cfsr = SCB->CFSR;
        uint32_t hfsr = SCB->HFSR;
        record_fault(stacked_ptr, cfsr, hfsr);
        debug_printf("MemManage fault in kernel at PC=0x%08lX, CFSR=0x%08lX HFSR=0x%08lX\n", (unsigned long)fault_pc, (unsigned long)cfsr, (unsigned long)hfsr);
        if (cfsr & (1u << 7))
            debug_printf("  MMFAR=0x%08lX\n", (unsigned long)SCB->MMFAR);
        NVIC_SystemReset();
        return;
    }

    Thread *t = g_kernel.currentThread;
    if (t)
    {
        uint32_t cfsr = SCB->CFSR;
        uint32_t hfsr = SCB->HFSR;
        record_fault(stacked_ptr, cfsr, hfsr);
        debug_printf("MemManage in thread at PC=0x%08lX, CFSR=0x%08lX\n", (unsigned long)fault_pc, (unsigned long)cfsr);
        if (cfsr & (1u << 7))
            debug_printf("  MMFAR=0x%08lX\n", (unsigned long)SCB->MMFAR);
        t->state = THREAD_HALTED;
        Kernal_Wipe_Thread(t);
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

// Bus Fault
KERNAL_FUNCTION
void BusFault_Handler(void)
{
    uint32_t *stacked_ptr;
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq %0, msp\n"
        "mrsne %0, psp\n"
        : "=r"(stacked_ptr)
        :
        :);

    uint32_t fault_pc = stacked_ptr[6];

    bool in_kernel = false;
    if ((uint32_t)&__kernel_text_start__ <= fault_pc && fault_pc < (uint32_t)&__kernel_text_end__)
        in_kernel = true;
    if ((uint32_t)&g_kernel <= fault_pc && fault_pc < ((uint32_t)&g_kernel + sizeof(g_kernel)))
        in_kernel = true;
    if ((uint32_t)&__stack_start__ <= fault_pc && fault_pc < (uint32_t)&__stack_end__)
        in_kernel = true;

    if (in_kernel)
    {
        uint32_t cfsr = SCB->CFSR;
        uint32_t hfsr = SCB->HFSR;
        record_fault(stacked_ptr, cfsr, hfsr);
        debug_printf("BusFault in kernel at PC=0x%08lX, CFSR=0x%08lX HFSR=0x%08lX\n", (unsigned long)fault_pc, (unsigned long)cfsr, (unsigned long)hfsr);
        if (cfsr & (1u << 15))
            debug_printf("  BFAR=0x%08lX\n", (unsigned long)SCB->BFAR);
        NVIC_SystemReset();
        return;
    }

    Thread *t = g_kernel.currentThread;
    if (t)
    {
        uint32_t cfsr = SCB->CFSR;
        uint32_t hfsr = SCB->HFSR;
        record_fault(stacked_ptr, cfsr, hfsr);
        debug_printf("BusFault in thread at PC=0x%08lX, CFSR=0x%08lX\n", (unsigned long)fault_pc, (unsigned long)cfsr);
        if (cfsr & (1u << 15))
            debug_printf("  BFAR=0x%08lX\n", (unsigned long)SCB->BFAR);
        t->state = THREAD_HALTED;
        Kernal_Wipe_Thread(t);
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

// Usage Fault
KERNAL_FUNCTION
void UsageFault_Handler(void)
{
    uint32_t *stacked_ptr;
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq %0, msp\n"
        "mrsne %0, psp\n"
        : "=r"(stacked_ptr)
        :
        :);

    uint32_t fault_pc = stacked_ptr[6];

    bool in_kernel = false;
    if ((uint32_t)&__kernel_text_start__ <= fault_pc && fault_pc < (uint32_t)&__kernel_text_end__)
        in_kernel = true;
    if ((uint32_t)&g_kernel <= fault_pc && fault_pc < ((uint32_t)&g_kernel + sizeof(g_kernel)))
        in_kernel = true;
    if ((uint32_t)&__stack_start__ <= fault_pc && fault_pc < (uint32_t)&__stack_end__)
        in_kernel = true;

    if (in_kernel)
    {
        uint32_t cfsr = SCB->CFSR;
        uint32_t hfsr = SCB->HFSR;
        record_fault(stacked_ptr, cfsr, hfsr);
        debug_printf("UsageFault in kernel at PC=0x%08lX, CFSR=0x%08lX HFSR=0x%08lX\n", (unsigned long)fault_pc, (unsigned long)cfsr, (unsigned long)hfsr);
        NVIC_SystemReset();
        return;
    }

    Thread *t = g_kernel.currentThread;
    if (t)
    {
        uint32_t cfsr = SCB->CFSR;
        uint32_t hfsr = SCB->HFSR;
        record_fault(stacked_ptr, cfsr, hfsr);
        debug_printf("UsageFault in thread at PC=0x%08lX, CFSR=0x%08lX\n", (unsigned long)fault_pc, (unsigned long)cfsr);
        t->state = THREAD_HALTED;
        Kernal_Wipe_Thread(t);
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
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

    // stacked_regs passed in r0 by the assembly stub
    uint32_t *stacked_regs = (uint32_t *)__get_PSP();
    // Determine which stack was used for exception by testing LR in EXC_RETURN
    uint32_t lr_val;
    __asm volatile("mov %0, lr" : "=r"(lr_val));
    if ((lr_val & 0x4) == 0)
    {
        // MSP used
        stacked_regs = (uint32_t *)__get_MSP();
    }

    uint32_t fault_pc = 0;
    if (stacked_regs)
        fault_pc = stacked_regs[6];

    bool in_kernel = false;
    if ((uint32_t)&__kernel_text_start__ <= fault_pc && fault_pc < (uint32_t)&__kernel_text_end__)
        in_kernel = true;
    if ((uint32_t)&g_kernel <= fault_pc && fault_pc < ((uint32_t)&g_kernel + sizeof(g_kernel)))
        in_kernel = true;
    if ((uint32_t)&__stack_start__ <= fault_pc && fault_pc < (uint32_t)&__stack_end__)
        in_kernel = true;

    if (in_kernel)
    {
        record_fault(stacked_regs, cfsr, hfsr);
        debug_printf("HardFault in kernel at PC=0x%08lX, CFSR=0x%08lX HFSR=0x%08lX\n", (unsigned long)fault_pc, (unsigned long)cfsr, (unsigned long)hfsr);
        if (cfsr & (1u << 15))
            debug_printf("  BFAR=0x%08lX\n", (unsigned long)SCB->BFAR);
        if (cfsr & (1u << 7))
            debug_printf("  MMFAR=0x%08lX\n", (unsigned long)SCB->MMFAR);
        NVIC_SystemReset();
        return;
    }

    // If it's a recoverable fault and we have a current thread, isolate it
    if (is_usage_fault || is_bus_fault || is_memmanage_fault || is_forced_hardfault)
    {
        if (g_kernel.currentThread)
        {
            record_fault(stacked_regs, cfsr, hfsr);
            debug_printf("HardFault in thread at PC=0x%08lX, CFSR=0x%08lX HFSR=0x%08lX\n", (unsigned long)fault_pc, (unsigned long)cfsr, (unsigned long)hfsr);
            if (cfsr & (1u << 15))
                debug_printf("  BFAR=0x%08lX\n", (unsigned long)SCB->BFAR);
            if (cfsr & (1u << 7))
                debug_printf("  MMFAR=0x%08lX\n", (unsigned long)SCB->MMFAR);
            g_kernel.currentThread->state = THREAD_HALTED;
            Kernal_Wipe_Thread(g_kernel.currentThread);
            Kernal_Yield();
            return;
        }
    }

    // Otherwise reset
    NVIC_SystemReset();
}

KERNAL_FUNCTION
void DebugMon_Handler(void)
{
    uint32_t dfsr = SCB->DFSR; // read cause
    SCB->DFSR = dfsr;          // clear flags (write 1s)

    debug_printf("DebugMon fault: DFSR=0x%08lX\n", dfsr);

    if (dfsr & SCB_DFSR_BKPT_Msk)
    {
        // Breakpoint instruction triggered
        debug_printf(" Breakpoint had been triggered.\n");
    }
    if (dfsr & SCB_DFSR_VCATCH_Msk)
    {
        // Vector catch triggered
        debug_printf(" Vector Catch had triggered.\n");
    }
    if (dfsr & SCB_DFSR_EXTERNAL_Msk)
    {
        // External debug request
        debug_printf(" External Debug Request had triggered.\n");
    }
}

KERNAL_FUNCTION
void FPU_IRQHandler(void)
{
    // Read FPSCR to see what caused the fault
    uint32_t fpscr = __get_FPSCR();

    // Mask of all cumulative exception flags we care about
    const uint32_t fpu_flags_mask = 0x9F; // bits 0–4 and 7

    // Extract only the exception flags
    uint32_t flags = fpscr & fpu_flags_mask;

    if (flags != 0)
    {
        // Debug log which flags were set
        debug_printf("FPU fault: FPSCR=0x%08lX\n", fpscr);

        if (flags & (1U << 0))
            debug_printf("  Invalid Operation (IOC)\n");
        if (flags & (1U << 1))
            debug_printf("  Divide by Zero (DZC)\n");
        if (flags & (1U << 2))
            debug_printf("  Overflow (OFC)\n");
        if (flags & (1U << 3))
            debug_printf("  Underflow (UFC)\n");
        if (flags & (1U << 4))
            debug_printf("  Inexact (IXC)\n");
        if (flags & (1U << 7))
            debug_printf("  Input Denormal (IDC)\n");
    }

    // Clear exception flags (write-1-to-clear)
    __set_FPSCR(fpscr & fpu_flags_mask);

    // Halt the current thread like other fault handlers
    Thread *t = g_kernel.currentThread;
    if (t)
    {
        t->state = ThreadState::THREAD_HALTED;
        Kernal_Wipe_Thread(t);
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk; // trigger PendSV for reschedule
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

KERNAL_FUNCTION
int Kernel_Dump_FaultTrace(char *outBuffer, int maxLen)
{
    if (!outBuffer || maxLen <= 0)
        return -1;

    int written = 0;
    for (uint32_t i = 0; i < FAULT_TRACE_DEPTH; i++)
    {
        uint32_t idx = (g_fault_trace_idx + i) % FAULT_TRACE_DEPTH;
        FaultRecord *r = &g_fault_trace[idx];
        if (r->timestamp == 0 && r->pc == 0 && r->cfsr == 0 && r->hfsr == 0)
            continue; // skip empty entries

        int n = snprintf(outBuffer + written, (size_t)maxLen - (size_t)written,
                         "#%u ts=%lu pc=0x%08lX cfsr=0x%08lX hfsr=0x%08lX bfar=0x%08lX mmfar=0x%08lX\n",
                         (unsigned)i, (unsigned long)r->timestamp,
                         (unsigned long)r->pc, (unsigned long)r->cfsr,
                         (unsigned long)r->hfsr, (unsigned long)r->bfar,
                         (unsigned long)r->mmfar);
        if (n < 0)
            break;
        written += n;
        if (written >= maxLen)
            break;

        /* Append register values in a compact line */
        n = snprintf(outBuffer + written, (size_t)maxLen - (size_t)written,
                     " regs=0x%08lX,0x%08lX,0x%08lX,0x%08lX,0x%08lX,0x%08lX,0x%08lX,0x%08lX\n",
                     (unsigned long)r->regs[0], (unsigned long)r->regs[1], (unsigned long)r->regs[2], (unsigned long)r->regs[3], (unsigned long)r->regs[4], (unsigned long)r->regs[5], (unsigned long)r->regs[6], (unsigned long)r->regs[7]);
        if (n < 0)
            break;
        written += n;
        if (written >= maxLen)
            break;
    }

    /* Ensure null termination if space remains */
    if (written < maxLen)
        outBuffer[written < 0 ? 0 : written] = '\0';

    return written;
}