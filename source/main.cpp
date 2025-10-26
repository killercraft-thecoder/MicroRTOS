#include <thread.h>
#include <stdint.h>

// Global LFSR state
volatile uint8_t lfsr = 1;  // Start with non-zero seed

// LFSR function: 8-bit, taps at bits 7 and 5 (x^8 + x^6 + x^5 + x^4 + 1)
uint8_t next_lfsr(uint8_t val) {
    uint8_t bit = ((val >> 7) ^ (val >> 5) ^ (val >> 4) ^ (val >> 3)) & 1;
    return (val << 1) | bit;
}

// Task 1: Continuously update LFSR
void task_counter(void *) {
    while (1) {
        lfsr = next_lfsr(lfsr);
        Thread_Sleep(32); // sleep for 32 ms.
    }
}

// Task 2: Exit if LFSR reaches 32
void task_monitor(void *) {
    while (1) {
        if (lfsr == 32) {
            Thread_Exit(0);  // Exit with status 0
        }
       Thread_Sleep(32);
    }
}

int main() {
    static thread_t thread1, thread2;
    ALIGN_STACK  uint32_t stack1[256], stack2[256];

    Create_Thread(&thread1, task_counter, nullptr, stack1, sizeof(stack1), 1);
    Create_Thread(&thread2, task_monitor, nullptr, stack2, sizeof(stack2), 2);

    Init_Scheduler();  // Starts the scheduler and never returns
    return 0;          // Unreachable
}