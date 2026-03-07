#include <thread.h>
#include <stdint.h>

// ------------------------------------------------------------
//  GLOBALS
// ------------------------------------------------------------

// LFSR state
volatile uint8_t lfsr = 1;

// Message queue buffer
static uint8_t queueBuffer[16];

// Semaphore
static semaphore_t sem;

// Timer ID
static uint8_t timerId;

// ------------------------------------------------------------
//  UTIL: 8-bit LFSR
// ------------------------------------------------------------
uint8_t next_lfsr(uint8_t v) {
    uint8_t bit = ((v >> 7) ^ (v >> 5) ^ (v >> 4) ^ (v >> 3)) & 1;
    return (v << 1) | bit;
}

// ------------------------------------------------------------
//  THREAD 1: LFSR generator
// ------------------------------------------------------------
void thread_lfsr(void *) {
    while (1) {
        lfsr = next_lfsr(lfsr);
        Thread_Sleep(10);
    }
}

// ------------------------------------------------------------
//  THREAD 2: Queue producer
// ------------------------------------------------------------
void thread_producer(void *) {
    queue_t q;
    Queue_Init(&q, queueBuffer, sizeof(queueBuffer), 1);

    while (1) {
        uint8_t value = lfsr;
        Queue_TrySend(&q, &value);   // non-blocking send
        Thread_Sleep(50);
    }
}

// ------------------------------------------------------------
//  THREAD 3: Queue consumer + semaphore
// ------------------------------------------------------------
void thread_consumer(void *) {
    queue_t q;
    Queue_Init(&q, queueBuffer, sizeof(queueBuffer), 1);

    while (1) {
        uint8_t value;
        if (Queue_TryReceive(&q, &value)) {
            if (value == 0xAA) {
                Semaphore_Signal(&sem);
            }
        }
        Thread_Sleep(20);
    }
}

// ------------------------------------------------------------
//  THREAD 4: Timer + dynamic memory + exit
// ------------------------------------------------------------
void thread_timer(void *) {
    timerId = Timer_Create(1000); // 1 second periodic timer

    while (1) {
        if (Timer_IsDone(timerId)) {
            Timer_Reset(timerId);

            // Allocate something just to show kmalloc works
            uint8_t *buf = (uint8_t *)kmalloc(32);
            if (buf) {
                for (int i = 0; i < 32; i++)
                    buf[i] = lfsr;
                kfree(buf);
            }

            // If semaphore was signaled, exit
            if (Semaphore_Get(&sem)) {
                Thread_Exit(0);
            }
        }
        Thread_Sleep(5);
    }
}

// ------------------------------------------------------------
//  MAIN
// ------------------------------------------------------------
int main() {
    static thread_t t1, t2, t3, t4;

    ALIGN_STACK uint32_t stack1[256];
    ALIGN_STACK uint32_t stack2[256];
    ALIGN_STACK uint32_t stack3[256];
    ALIGN_STACK uint32_t stack4[256];

    Semaphore_Init(&sem, 0);

    Create_Thread(&t1, thread_lfsr,     nullptr, stack1, sizeof(stack1), 1);
    Create_Thread(&t2, thread_producer, nullptr, stack2, sizeof(stack2), 2);
    Create_Thread(&t3, thread_consumer, nullptr, stack3, sizeof(stack3), 2);
    Create_Thread(&t4, thread_timer,    nullptr, stack4, sizeof(stack4), 3);

    Init_Scheduler(); // never returns
    return 0;
}