#ifndef MUSTINCLUDE_H
#define MUSTINCLUDE_H

#ifdef UF2_BOOTLOADER
  #define USER_VECT_TAB_ADDRESS  0x2000U  // 8 KB offset
#else
  #define USER_VECT_TAB_ADDRESS  0x00U
#endif

#include "CMSIS/system_stm32f4xx.h"

#endif