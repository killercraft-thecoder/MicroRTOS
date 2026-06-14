# MicroRTOS

A compact, modular, preemptive real‑time operating system targeted at STM32F4-class microcontrollers.

MicroRTOS is designed for learning, experimentation, and fast iteration. It provides a small, auditable kernel with a clear syscall boundary, a lightweight virtual file system, and a set of core RTOS primitives useful for embedded projects and teaching.

Key design goals
- Minimalism: small trusted computing base and straightforward APIs.
- Observability: lightweight tracing and fault diagnostic helpers.
- Extensibility: driver registration via a prefix‑based VFS.

Core features
- Preemptive, priority‑based scheduler with PSP stack frames for unprivileged threads.
- Mutexes, semaphores, and blocking/non‑blocking message queues.
- Per‑thread soft timers and millisecond system tick.
- TLSF allocator exposed via SVC wrappers: `Malloc`, `Free`, `Calloc`, `Realloc`.
- Small prefix‑based VFS: drivers implement `FS_Open`, `FS_Close`, `FS_Read`, `FS_Write`, `FS_List` and register with `VFS_RegisterDriver()`.
- RTOS‑safe hardware helpers and SVC‑mediated I/O wrappers for common STM32 HAL peripherals.

Notes
- This project is experimental and evolving; it is not intended as a production‑grade RTOS.
- The `include/CMSIS/` directory contains generic CMSIS headers and a subset of the STM32F4 HAL used for builds and examples. See [include/CMSIS/](include/CMSIS/) for details.
- This repository was "vibe coded" — developed with human direction and assisted by AI tools to speed iteration and prototyping.

Contributing
- Issues, PRs, and small reproducible patches are welcome. For larger design changes (scheduler, allocator, MPU model) please open an issue first to discuss the approach. (please dont make a billon though.)

License
- Check the `LICENSE` file in the project root for license terms.

---
