#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>  // for size_t

// Internal seed state
static unsigned int __rand_seed = 1;

// Exit current thread/task
void Thread_Exit(int code);

static inline void exit(int code) {
    Thread_Exit(code);
}

/**
 * Seed the pseudo-random number generator.
 */
static inline void srand(unsigned int seed) {
    __rand_seed = seed ? seed : 1;
}

/**
 * Generate a pseudo-random number (32-bit).
 */
static inline uint32_t rand(void) {
    __rand_seed = __rand_seed * 1664525 + 1013904223;
    return __rand_seed;
}

// Absolute value
int abs(int x) {
    if (x < 0) {
        return 0-x;
    } else {
        return x;
    }
}

#ifdef __cplusplus
}
#endif