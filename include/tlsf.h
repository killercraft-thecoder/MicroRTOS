#ifndef MICRO_RTOS_TLSF_H
#define MICRO_RTOS_TLSF_H

#include <stddef.h>
#include <stdint.h>
#include "process.h"
#include "debug_printf.h"

// -----------------------------------------------------------------------------
// TLSF configuration
// -----------------------------------------------------------------------------

#define TLSF_FL_BITS   5      // 32 first-level buckets
#define TLSF_SL_BITS   4      // 16 second-level buckets

#define TLSF_FL_COUNT  (1u << TLSF_FL_BITS)
#define TLSF_SL_COUNT  (1u << TLSF_SL_BITS)

#define TLSF_ALIGN     8u
#define TLSF_ALIGN_UP(x)   (((x) + (TLSF_ALIGN - 1)) & ~(TLSF_ALIGN - 1))

// -----------------------------------------------------------------------------
// Block header + free block structures
// -----------------------------------------------------------------------------

typedef struct TlsfBlockHeader
{
    size_t size;                     // includes header; LSB = free flag
    struct TlsfBlockHeader *prev_phys;
} TlsfBlockHeader;

typedef struct TlsfFreeBlock
{
    TlsfBlockHeader header;
    struct TlsfFreeBlock *prev_free;
    struct TlsfFreeBlock *next_free;
} TlsfFreeBlock;

// -----------------------------------------------------------------------------
// TLSF control structure
// -----------------------------------------------------------------------------

typedef struct
{
    uint32_t fl_bitmap;                                   // first-level bitmap
    uint32_t sl_bitmap[TLSF_FL_COUNT];                    // second-level bitmaps
    TlsfFreeBlock *free_lists[TLSF_FL_COUNT][TLSF_SL_COUNT];

    void   *heap_start;
    size_t  heap_size;

} TlsfControl;

// -----------------------------------------------------------------------------
// External linker symbols for heap
// -----------------------------------------------------------------------------

extern uint8_t __heap_start__;
extern uint8_t __heap_end__;

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

void TLSF_Init(TlsfControl *ctl);

void *TLSF_Malloc (TlsfControl *ctl, size_t size);
void  TLSF_Free   (TlsfControl *ctl, void *ptr);
void *TLSF_Realloc(TlsfControl *ctl, void *ptr, size_t newSize);
void *TLSF_Calloc (TlsfControl *ctl, size_t n, size_t size);

#endif // MICRO_RTOS_TLSF_H

