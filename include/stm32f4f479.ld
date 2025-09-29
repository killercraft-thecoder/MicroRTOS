/* STM32F479NI Linker Script for RTOS + UF2 Bootloader */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 256K
  CCMRAM (rw) : ORIGIN = 0x10000000, LENGTH = 64K
}

ENTRY(Reset_Handler)

SECTIONS
{
  /* Interrupt Vector Table */
  .isr_vector :
  {
    KEEP(*(.isr_vector))  /* Vector table must be retained */
  } >FLASH

  /* Code and Read-Only Data */
  .text :
  {
    *(.text*)             /* Application code */
    *(.rodata*)           /* Read-only data */
    KEEP(*(.init))
    KEEP(*(.fini))
  } >FLASH

  /* Exception Index Table (used by unwinding) */
  .ARM.exidx :
  {
    *(.ARM.exidx*)
  } >FLASH

  /* Initialized Data */
  .data : AT (ADDR(.text) + SIZEOF(.text))
  {
    __data_start__ = .;
    *(.data*)
    __data_end__ = .;
  } >RAM

  /* Uninitialized Data */
  .bss :
  {
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    __bss_end__ = .;
  } >RAM

  /* Stack Section */
  .stack (COPY):
  {
    __stack_start__ = .;
    . = . + 0x1000;       /* 4 KB stack */
    __stack_end__ = .;
  } >RAM

  /* Heap Section */
  .heap (COPY):
  {
    __heap_start__ = .;
    . = . + 0x1000;       /* 4 KB heap */
    __heap_end__ = .;
  } >RAM

  .ramfunc :
  {
    KEEP(*(.ramfunc))
  } >RAM

  /* Symbols for startup code */
  PROVIDE(_end = .);
  PROVIDE(__heap_base = __heap_start__);
  PROVIDE(__heap_limit = __heap_end__);
}