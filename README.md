# MicroRTOS  
A small, modular, preemptive real‑time operating system for STM32F4 microcontrollers.

MicroRTOS is a work‑in‑progress kernel designed for learning, experimentation, and embedded development.  
It provides a clean API, a simple syscall layer, a lightweight VFS dispatch system, and a set of RTOS primitives such as threads, mutexes, queues, and timers.

This project is not intended to be production‑ready, but it is functional and actively evolving.

---

## ✨ Features

### 🧵 **Preemptive Multithreading**
- Priority‑based scheduler  
- Manual stack‑frame construction for each thread  
- Voluntary yielding (`Yield()`)  
- Thread sleep with millisecond granularity though exact times are never gurranted (`Thread_Sleep()`)  

### 🔒 **Synchronization Primitives**
- Mutexes (`Mutex_Create`, `Mutex_Lock`, `Mutex_Unlock`)  
- Blocking and non‑blocking message queues  
  - `Queue_Send`, `Queue_Receive`  
  - `Queue_TrySend`, `Queue_TryReceive`  

### ⏱️ **Per‑Thread Software Timers**
Each thread may create up to a small fixed number of timers:
- `Timer_Create(ms)`  
- `Timer_IsDone(id)`  
- `Timer_Reset(id, ms)`  
- `Timer_Cancel(id)`  
- `Timer_Remaining(id)`  

Timers are polled by the owning thread and run independently in the background.

### 🧩 **System Call Layer (SVC‑Based)**
User threads run unprivileged and access kernel services through SVC calls.  
This includes:
- Memory allocation  
- Queue operations  
- Driver I/O  
- Tick retrieval  

### 💾 **Real‑Time TLSF Memory Allocator**
`Malloc`, `Free`, `Calloc`, and `Realloc` are syscall wrappers around a TLSF allocator:
- O(1) allocation and free  
- Low fragmentation  
- Suitable for real‑time systems  

### 📁 **Minimal Virtual File System (VFS)**
MicroRTOS includes a small prefix‑based VFS dispatch layer:
- Drivers register via `VFS_RegisterDriver()`  
- Paths are routed by prefix (e.g., `/sd/...`, `/flash/...`)  
- Prefix is stripped before calling the driver  
- Sparse FD mapping (`user_fd → driver_fd`)  
- No caching, metadata, or POSIX semantics  

Driver API includes:
- `FS_Open`  
- `FS_Close`  
- `FS_Read`  
- `FS_Write`  
- `FS_List`  

### 🔌 **Hardware Abstraction Helpers**
Convenience constructors for STM32 HAL peripherals:
- `GPIO_NEW`  
- `UART_NEW`  
- `I2C_NEW`  
- `SPI_NEW`  

RTOS‑safe I/O wrappers using SVC:
- `SPI_Transmit`, `SPI_Receive`, `SPI_TransmitReceive`  
- `UART_Transmit`, `UART_Receive`  
- GPIO read/write helpers  

### ⏲️ **System Tick**
- Millisecond tick  
- `OS_GetTick()` syscall for unprivileged threads  

---

## 🚧 Current Status

MicroRTOS is under active development.  
It may contain bugs, incomplete features, or unstable behavior and may not compile.  
The API is subject to change as the kernel evolves. but it is unlikey that old api functions are changed.
