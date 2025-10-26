/* STM32F401XE Linker Script for RTOS + UF2 Bootloader */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08002000, LENGTH = 496K  /* 512K total - 8K bootloader */
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 96K
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
    . = ALIGN(4);
    __data_start__ = .;
    *(.data*)
    . = ALIGN(4);
    __data_end__ = .;
  } >RAM

  /* Uninitialized Data */
  .bss :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } >RAM

  /* Stack Section */
  .stack (COPY):
  {
    . = ALIGN(4);
    __stack_start__ = .;
    . = . + 0x1000;       /* 4 KB stack */
    . = ALIGN(4);
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

  .executable :
  {
    KEEP(*(.executable))
  } >RAM

  /* Pure kernel code section */
  __kernel_start__ = ALIGN(32);
  .kernel :
  {
    . = ALIGN(32);
    __kernel_text_start__ = .;

    *(.text.kernel .text.kernel.*)

    /* No rodata in .kernel: keep pure code only */
    . = ALIGN(32);
    __kernel_text_end__ = .;
  } >FLASH

  .api_table 0x0803F000 :
    {
        . = ALIGN(4);
        __api_table_start = .;
        KEEP(*(.api_table))
        . = ALIGN(4);
        __api_table_end = .;
    } >FLASH



  /* Symbols for startup code */
  PROVIDE(_end = .);
  PROVIDE(__heap_base = __heap_start__);
  PROVIDE(__heap_limit = __heap_end__);
}