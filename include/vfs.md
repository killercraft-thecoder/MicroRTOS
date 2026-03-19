# **MicroRTOS Virtual File System (VFS) Overview**

This document describes the design, behavior, and quirks of the MicroRTOS Virtual File System. It is intended for developers writing filesystem drivers or working on kernel‑level file operations.

The VFS in MicroRTOS is intentionally small, predictable, and modular. It does not attempt to replicate POSIX semantics; instead, it provides a clean dispatch layer that routes file operations to the correct filesystem driver.

---

## **1. Driver Model**

A filesystem driver registers itself with the kernel by providing a `FileSystemDriver` structure. Each driver must define:

- A unique prefix (e.g., `"sd"`, `"flash"`, `"rom"`)
- Function pointers for:
  - `open`
  - `close`
  - `read`
  - `write`
  - `list`

Drivers are stored in a **fixed array** inside the global kernel data structure (`g_kernel.fsDrivers`). The array size is defined in the kernel configuration.

Drivers are expected to manage their own internal file handles. The VFS does not track per‑driver state.

---

## **2. Prefix‑Based Routing**

Paths are routed to drivers based on the prefix immediately following the leading slash.

Example:

```
/sd/music/song.mp3
```

Prefix extracted: `sd`  
Driver selected: the one whose `prefix` field matches `"sd"`.

If no prefix matches, the VFS falls back to the **first mounted driver** in `g_kernel.fsDrivers`.

This allows:

- Multiple filesystems to coexist
- Pseudo‑filesystems (`/dev`, `/proc`, etc.)
- Simple routing without a full mount table

---

## **3. Path Stripping**

Before calling a driver’s `open` or `list` function, the VFS strips the prefix from the path.

Example:

```
Input path:  /sd/music/song.mp3
Driver sees: music/song.mp3
```

This keeps drivers simple and unaware of global mount structure.

---

## **4. File Descriptor Mapping**

The VFS does not maintain a fixed‑size FD table. Instead, it uses a **sparse linked list** that maps:

```
user_fd → (driver pointer, driver_fd)
```

This design:

- Avoids RAM waste
- Allows unlimited open files
- Keeps the kernel’s memory footprint small
- Leaves file handle management to the driver

User FDs are assigned sequentially starting at 3 (0, 1, 2 reserved).

---

## **5. VFS Responsibilities**

The VFS layer is intentionally minimal. It performs:

- Prefix parsing
- Driver selection
- Path stripping
- FD mapping
- Dispatching to driver functions

The VFS does **not**:

- Cache file data
- Track file metadata
- Implement directories
- Enforce permissions
- Manage storage devices

All of these are the responsibility of the filesystem driver.

---

## **6. Driver Requirements**

A filesystem driver must:

- Provide a unique prefix
- Implement all required function pointers
- Manage its own internal file handles
- Return negative values on error
- Never block indefinitely (unless explicitly designed to)

Drivers may assume:

- Paths passed to them are already prefix‑stripped
- They will only receive valid pointers from the kernel
- The VFS will not modify their internal state

---

## **7. Error Handling**

The VFS returns:

- Negative values for errors
- Non‑negative file descriptors for successful `open`
- Byte counts for `read` and `write`

Drivers should follow the same convention.

---

## **8. Security and Validation**

The kernel validates driver function pointers during registration. A driver will be rejected if any callback pointer:

- Is null
- Points outside allowed memory regions
- Falls into unmapped or user memory

This prevents malicious or corrupted drivers from compromising the kernel.

---

## **9. Future Extensions**

The current VFS design intentionally leaves room for:

- Mount tables
- Per‑driver root paths
- Path normalization
- Symbolic links
- Virtual filesystems
- Device files
- File metadata

The prefix‑based routing model is compatible with all of these.
