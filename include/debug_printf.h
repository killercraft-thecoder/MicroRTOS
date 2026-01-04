#ifndef DEBUG_PRINTF_H
#define DEBUG_PRINTF_H

#ifdef HAS_PRINTF
#include <printf.h>
#define debug_printf(...) printf(__VA_ARGS__)
#else
#define debug_printf(...)
#endif

#endif