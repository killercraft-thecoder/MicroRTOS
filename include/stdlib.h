#ifndef __STDLIB_H_
#define __STDLIB_H_

#include <stddef.h> // for size_t

/* Null pointer constant */
#ifndef NULL
#  define NULL ((void*)0)
#endif

/* Exit status codes */
#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

/* Maximum value returned by rand() */
#define RAND_MAX 0xFFFFFFFF  // 4294967295

/* Types you may want to define here or in <stddef.h> */
typedef unsigned int size_t;
typedef int          wchar_t;

/* Division result types */
typedef struct { int quot; int rem; } div_t;
typedef struct { long quot; long rem; } ldiv_t;
typedef struct { long long quot; long long rem; } lldiv_t;



    // Internal seed state
    static unsigned int __rand_seed = 1;

    // Exit current thread/task
    void Thread_Exit(int code);

    static inline void exit(int code)
    {
        Thread_Exit(code);
    }

    /**
     * Seed the pseudo-random number generator.
     */
    static inline void srand(unsigned int seed)
    {
        __rand_seed = seed ? seed : 1;
    }

    /**
     * Generate a pseudo-random number (32-bit).
     */
    static inline uint32_t rand(void)
    {
        __rand_seed = __rand_seed * 1664525 + 1013904223;
        return __rand_seed;
    }

    // Absolute value
    int abs(int x)
    {
        if (x < 0)
        {
            return 0 - x;
        }
        else
        {
            return x;
        }
    }

#endif