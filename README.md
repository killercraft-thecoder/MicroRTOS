# **MicroRTOS**

MicroRTOS is a small, modular real‑time operating system for writing simple embedded applications. It provides a clean C API for threads, IPC, timers, device I/O, memory allocation, and a prefix‑based virtual filesystem.

This project is still a work‑in‑progress, but the API surface is stable enough to build programs on top of it.

---

## **🚀 Writing Programs for MicroRTOS**

MicroRTOS exposes a simple set of C functions. All system calls are made through these APIs; the kernel handles the privileged transitions internally.

Example:

```c
void TaskA(void *arg) {
    int fd = FS_Open("/sd/data.txt", 0);
    FS_Write(fd, "Hello", 5);
    FS_Close(fd);

    while (1) {
        Thread_Sleep(1000);
    }
}
```

You write normal C code — MicroRTOS handles the rest.

---

# **📘 MicroRTOS API Reference**

Below is the complete list of all public API functions, grouped by subsystem.

---

# **🧵 Thread & Process API**

| Function | Parameters | Description |
|---------|------------|-------------|
| `Thread_Create` | `(ThreadEntry entry, void *arg, uint32_t stackSize)` | Create a new thread |
| `Thread_Sleep` | `(uint32_t ms)` | Sleep for milliseconds |
| `Thread_Yield` | `()` | Yield CPU voluntarily |
| `Process_Add` | `(ProcessEntry entry, void *arg)` | Create a new process |
| `Process_Exit` | `()` | Terminate current process |
| `Process_Remove` | `(ProcessHandle p)` | Remove a process |

---

# **🔒 Mutex API**

| Function | Parameters |
|----------|------------|
| `Mutex_Create` | `()` |
| `Mutex_Lock` | `(Mutex m)` |
| `Mutex_Unlock` | `(Mutex m)` |
| `Mutex_Destroy` | `(Mutex m)` |
| `Mutex_ReadFromThread` | `(ThreadHandle t)` |

---

# **🔑 Semaphore API**

| Function | Parameters |
|----------|------------|
| `Semaphore_Get` | `()` |
| `Semaphore_Wait` | `(Semaphore s)` |
| `Semaphore_Signal` | `(Semaphore s)` |

---

# **📬 Message Queue API**

| Function | Parameters |
|----------|------------|
| `Queue_Create` | `(void *buffer, uint32_t size)` |
| `Queue_Send` | `(Queue q, const void *msg)` |
| `Queue_Receive` | `(Queue q, void *msg)` |
| `Queue_TrySend` | `(Queue q, const void *msg)` |
| `Queue_TryReceive` | `(Queue q, void *msg)` |

---

# **⏱ Timer API**

| Function | Parameters |
|----------|------------|
| `Timer_Create` | `(uint32_t ms)` |
| `Timer_IsDone` | `(Timer t)` |
| `Timer_Reset` | `(Timer t)` |
| `Timer_Cancel` | `(Timer t)` |
| `Timer_Remaining` | `(Timer t)` |
| `Get_Tick` | `()` |

---

# **🔌 Device I/O API**

### **GPIO**
| Function | Parameters |
|----------|------------|
| `GPIO_New` | `(int pin)` |
| `GPIO_Write` | `(GPIO g, int value)` |
| `GPIO_Read` | `(GPIO g)` |

### **UART**
| Function | Parameters |
|----------|------------|
| `UART_New` | `(uint32_t baud)` |
| `UART_Transmit` | `(UART u, const void *data, uint32_t len)` |
| `UART_Receive` | `(UART u, void *data, uint32_t len)` |

### **I2C**
| Function | Parameters |
|----------|------------|
| `I2C_New` | `()` |
| `I2C_MasterTxRx` | `(I2C bus, uint8_t addr, const void *tx, uint32_t txLen, void *rx, uint32_t rxLen)` |

### **SPI**
| Function | Parameters |
|----------|------------|
| `SPI_New` | `()` |
| `SPI_Transmit` | `(SPI s, const void *data, uint32_t len)` |
| `SPI_Receive` | `(SPI s, void *data, uint32_t len)` |
| `SPI_TransmitRecv` | `(SPI s, const void *tx, void *rx, uint32_t len)` |

---

# **📁 Filesystem API (VFS)**

### **Driver Registration**
| Function | Parameters |
|----------|------------|
| `VFS_RegisterDriver` | `(FileSystemDriver *driver)` |

### **File Operations**
| Function | Parameters |
|----------|------------|
| `FS_Open` | `(const char *path, int flags)` |
| `FS_Close` | `(int fd)` |
| `FS_Read` | `(int fd, void *buf, int size)` |
| `FS_Write` | `(int fd, const void *buf, int size)` |
| `FS_List` | `(const char *path, char *out, int maxLen)` |

---

# **💾 Memory API**

| Function | Parameters |
|----------|------------|
| `Malloc` | `(size_t size)` |
| `Free` | `(void *ptr)` |
| `Calloc` | `(size_t n, size_t size)` |
| `Realloc` | `(void *ptr, size_t size)` |
