// tlsf.c - Minimal TLSF-style allocator for MicroRTOS

#include "tlsf.h"
#include <string.h>

// -----------------------------------------------------------------------------
// Internal helpers and macros
// -----------------------------------------------------------------------------

// We use the LSB of size as the "free" flag.
#define BLOCK_FREE_FLAG ((size_t)1u)
#define BLOCK_SIZE_MASK (~BLOCK_FREE_FLAG)

static inline size_t block_get_size(const TlsfBlockHeader *h)
{
    return h->size & BLOCK_SIZE_MASK;
}

static inline int block_is_free(const TlsfBlockHeader *h)
{
    return (h->size & BLOCK_FREE_FLAG) != 0;
}

static inline void block_set_free(TlsfBlockHeader *h)
{
    h->size |= BLOCK_FREE_FLAG;
}

static inline void block_set_used(TlsfBlockHeader *h)
{
    h->size &= BLOCK_SIZE_MASK;
}

static inline TlsfFreeBlock *as_free_block(TlsfBlockHeader *h)
{
    return (TlsfFreeBlock *)h;
}

static inline TlsfBlockHeader *as_block_header(void *ptr)
{
    return (TlsfBlockHeader *)ptr;
}

static inline void *block_to_ptr(TlsfBlockHeader *h)
{
    return (void *)((uint8_t *)h + sizeof(TlsfBlockHeader));
}

static inline TlsfBlockHeader *ptr_to_block(void *ptr)
{
    return (TlsfBlockHeader *)((uint8_t *)ptr - sizeof(TlsfBlockHeader));
}

// Round size up to include header and alignment.
static size_t adjust_request_size(size_t size)
{
    if (size == 0)
        return 0;

    size_t total = size + sizeof(TlsfBlockHeader);
    total = TLSF_ALIGN_UP(total);
    if (total < sizeof(TlsfFreeBlock))
        total = sizeof(TlsfFreeBlock);
    return total;
}

// -----------------------------------------------------------------------------
// Bucket mapping (size -> (fl, sl))
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// mapping_insert: Convert a block size into (fl, sl) bucket indices.
// -----------------------------------------------------------------------------
static void mapping_insert(size_t size, int *fl, int *sl)
{
    // Ensure minimum block size
    if (size < sizeof(TlsfFreeBlock))
        size = sizeof(TlsfFreeBlock);

    // First-level index = log2(size)
    int f = 31 - __builtin_clz((unsigned int)size);

    // Clamp FL to valid range
    if (f < 0)
        f = 0;
    if (f >= TLSF_FL_COUNT)
        f = TLSF_FL_COUNT - 1;

    *fl = f;

    // Compute SL index by subdividing the FL range
    // FL range is [2^f, 2^(f+1))
    size_t base = (size_t)1 << f;
    size_t offset = size - base;

    // Size of each SL subdivision
    size_t step = base >> TLSF_SL_BITS; // base / SL_COUNT

    if (step == 0)
    {
        // For very small FL ranges, just put everything in SL 0
        *sl = 0;
        return;
    }

    int s = offset / step;
    if (s >= TLSF_SL_COUNT)
        s = TLSF_SL_COUNT - 1;

    *sl = s;
}

// mapping_search: find the smallest (fl, sl) that can fit "size".
// -----------------------------------------------------------------------------
// mapping_search: find the smallest (fl, sl) bucket that can satisfy "size".
// Returns 1 on success, 0 if no suitable block exists.
// -----------------------------------------------------------------------------
static int mapping_search(TlsfControl *ctl, size_t size, int *fl, int *sl)
{
    int f, s;

    // First, map the requested size to its natural bucket.
    mapping_insert(size, &f, &s);

    // Mask out all FLs below f (we need >= requested size).
    uint32_t fl_map = ctl->fl_bitmap & (~0u << f);
    if (!fl_map)
        return 0; // No FL with blocks >= requested size.

    // Find the first FL that has any free blocks.
    f = __builtin_ctz(fl_map);

    // Now look at the SL bitmap for this FL.
    uint32_t sl_map = ctl->sl_bitmap[f];

    if (f == *fl)
    {
        // Same FL as the natural bucket: mask out SLs < s.
        sl_map &= (~0u << s);
    }

    if (!sl_map)
    {
        // No SL in this FL can satisfy the request.
        // Try the next FL that has any free blocks.
        fl_map &= ~(1u << f);
        if (!fl_map)
            return 0;

        f = __builtin_ctz(fl_map);
        sl_map = ctl->sl_bitmap[f];
        if (!sl_map)
            return 0;
    }

    // Find the first SL with a free block.
    s = __builtin_ctz(sl_map);

    *fl = f;
    *sl = s;
    return 1;
}

// -----------------------------------------------------------------------------
// Free list management
// -----------------------------------------------------------------------------

static void insert_free_block(TlsfControl *ctl, TlsfFreeBlock *block)
{
    int fl, sl;
    size_t size = block_get_size(&block->header);

    mapping_insert(size, &fl, &sl);

    // Insert at head of free list
    block->prev_free = NULL;
    block->next_free = ctl->free_lists[fl][sl];

    if (block->next_free)
        block->next_free->prev_free = block;

    ctl->free_lists[fl][sl] = block;

    // Update bitmaps
    ctl->fl_bitmap |= (1u << fl);
    ctl->sl_bitmap[fl] |= (1u << sl);
}

static void remove_free_block(TlsfControl *ctl, TlsfFreeBlock *block)
{
    int fl, sl;
    size_t size = block_get_size(&block->header);

    mapping_insert(size, &fl, &sl);

    TlsfFreeBlock *prev = block->prev_free;
    TlsfFreeBlock *next = block->next_free;

    // Unlink from list
    if (prev)
        prev->next_free = next;
    if (next)
        next->prev_free = prev;

    // If this block was the head, update the list head
    if (ctl->free_lists[fl][sl] == block)
        ctl->free_lists[fl][sl] = next;

    // If the list is now empty, clear the SL and possibly FL bits
    if (!ctl->free_lists[fl][sl])
    {
        ctl->sl_bitmap[fl] &= ~(1u << sl);

        if (ctl->sl_bitmap[fl] == 0)
            ctl->fl_bitmap &= ~(1u << fl);
    }

    block->prev_free = NULL;
    block->next_free = NULL;
}

// -----------------------------------------------------------------------------
// Block splitting and coalescing
// -----------------------------------------------------------------------------

static TlsfBlockHeader *split_block(TlsfControl *ctl, TlsfBlockHeader *block, size_t size)
{
    size_t block_size = block_get_size(block);

    if (block_size >= size + sizeof(TlsfFreeBlock))
    {
        // Create a new free block from the tail.
        TlsfBlockHeader *new_block = (TlsfBlockHeader *)((uint8_t *)block + size);
        new_block->size = (block_size - size) | BLOCK_FREE_FLAG;
        new_block->prev_phys = block;

        // Update next physical neighbor's prev_phys if exists.
        TlsfBlockHeader *next = (TlsfBlockHeader *)((uint8_t *)new_block + block_get_size(new_block));
        if ((uint8_t *)next < (uint8_t *)ctl->heap_start + ctl->heap_size)
        {
            next->prev_phys = new_block;
        }

        // Shrink original block.
        block->size = size | (block->size & BLOCK_FREE_FLAG);

        // Insert new free block into free lists.
        insert_free_block(ctl, as_free_block(new_block));
    }

    return block;
}

static TlsfBlockHeader *coalesce_block(TlsfControl *ctl, TlsfBlockHeader *block)
{
    // Coalesce with next block if free.
    TlsfBlockHeader *next = (TlsfBlockHeader *)((uint8_t *)block + block_get_size(block));

    if ((uint8_t *)next < (uint8_t *)ctl->heap_start + ctl->heap_size)
    {
        if (next->prev_phys != block)
        {
            debug_printf("[TLSF] ERROR: prev_phys mismatch: %p -> %p (expected %p)\n",
                         next, next->prev_phys, block);
        }
    }

    if ((uint8_t *)next < (uint8_t *)ctl->heap_start + ctl->heap_size && block_is_free(next))
    {
        remove_free_block(ctl, as_free_block(next));
        block->size = (block_get_size(block) + block_get_size(next)) | BLOCK_FREE_FLAG;
    }

    // Coalesce with previous block if free.
    TlsfBlockHeader *prev = block->prev_phys;
    if (prev && block_is_free(prev))
    {
        remove_free_block(ctl, as_free_block(prev));
        prev->size = (block_get_size(prev) + block_get_size(block)) | BLOCK_FREE_FLAG;
        block = prev;
    }

    return block;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

void TLSF_Init(TlsfControl *ctl)
{
    memset(ctl, 0, sizeof(*ctl));

    ctl->heap_start = &__heap_start__;
    ctl->heap_size = (size_t)(&__heap_end__ - &__heap_start__);

    // Create a single large free block covering the entire heap.
    TlsfFreeBlock *initial = (TlsfFreeBlock *)ctl->heap_start;
    size_t total_size = TLSF_ALIGN_UP(ctl->heap_size);

    initial->header.size = total_size | BLOCK_FREE_FLAG;
    initial->header.prev_phys = NULL;
    initial->prev_free = initial->next_free = NULL;

    insert_free_block(ctl, initial);
}

void *TLSF_Malloc(TlsfControl *ctl, size_t size)
{
    size_t req = adjust_request_size(size);
    if (req == 0)
        return NULL;

    int fl = 0, sl = 0;
    if (!mapping_search(ctl, req, &fl, &sl))
        return NULL;

    TlsfFreeBlock *block = ctl->free_lists[fl][sl];
    if (!block)
        return NULL;

    remove_free_block(ctl, block);

    TlsfBlockHeader *h = &block->header;
    h = split_block(ctl, h, req);
    block_set_used(h);

    return block_to_ptr(h);
}

void TLSF_Free(TlsfControl *ctl, void *ptr)
{
    if (!ptr)
        return;

    if ((uint8_t *)ptr < (uint8_t *)ctl->heap_start ||
        (uint8_t *)ptr >= (uint8_t *)ctl->heap_start + ctl->heap_size)
    {
        debug_printf("[TLSF] ERROR: free(%p) outside heap range!\n", ptr);
        return;
    }

    if (((uintptr_t)ptr & (TLSF_ALIGN - 1)) != 0)
    {
        debug_printf("[TLSF] ERROR: free(%p) not aligned!\n", ptr);
        return;
    }

    if (block_is_free(h))
    {
        debug_printf("[TLSF] ERROR: double free detected at %p\n", ptr);
        return;
    }

    TlsfBlockHeader *h = ptr_to_block(ptr);

    size_t bsize = block_get_size(h);
    if (bsize < sizeof(TlsfFreeBlock) || (bsize & 0x3) != 0)
    {
        debug_printf("[TLSF] ERROR: corrupted block header at %p (size=%u)\n",
                     h, (unsigned)bsize);
        return;
    }

    block_set_free(h);

    h = coalesce_block(ctl, h);
    TlsfFreeBlock *fb = as_free_block(h);

    if (fb->prev_free || fb->next_free)
    {
        debug_printf("[TLSF] ERROR: free block %p has non-null links before insertion\n", fb);
    }
    insert_free_block(ctl, fb);
}

void *TLSF_Calloc(TlsfControl *ctl, size_t n, size_t size)
{
    size_t total;
    if (__builtin_mul_overflow(n, size, &total))
        return NULL;

    void *ptr = TLSF_Malloc(ctl, total);
    if (ptr)
        memset(ptr, 0, total);
    return ptr;
}

void *TLSF_Realloc(TlsfControl *ctl, void *ptr, size_t newSize)
{
    if (!ptr)
        return TLSF_Malloc(ctl, newSize);
    if (newSize == 0)
    {
        TLSF_Free(ctl, ptr);
        return NULL;
    }

    TlsfBlockHeader *h = ptr_to_block(ptr);
    size_t old_size = block_get_size(h) - sizeof(TlsfBlockHeader);
    size_t req = adjust_request_size(newSize);

    // If the current block is already big enough, optionally shrink/split.
    if (block_get_size(h) >= req)
    {
        // TODO: optionally split the block if there's enough excess space.
        return ptr;
    }

    // Otherwise, allocate a new block and copy.
    void *new_ptr = TLSF_Malloc(ctl, newSize);
    if (!new_ptr)
        return NULL;

    size_t copy_size = old_size < newSize ? old_size : newSize;
    memcpy(new_ptr, ptr, copy_size);
    TLSF_Free(ctl, ptr);
    return new_ptr;
}