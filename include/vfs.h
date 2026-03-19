#ifndef MICRO_RTOS_VFS_H
#define MICRO_RTOS_VFS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Filesystem driver interface for VFS registration.
 */
typedef struct
{
    const char *name;
    const char *prefix;   // e.g. "sd", "flash", "rom", "dev"


    int (*open)(const char *path, int flags);
    int (*close)(int fd);
    int (*read)(int fd, void *buf, int size);
    int (*write)(int fd, const void *buf, int size);
    int (*list)(const char *path, char *out, int maxLen);
} FileSystemDriver;

/**
 * @brief Register a filesystem driver with the VFS.
 * @param driver Pointer to a FileSystemDriver structure.
 * @return 0 on success, negative on error.
 */
int VFS_RegisterDriver(FileSystemDriver *driver);

/**
 * @brief Open a file or resource via the VFS.
 * @param path Path to the file.
 * @param flags Access mode flags.
 * @return File descriptor, or negative on error.
 */
int VFS_Open(const char *path, int flags);

/**
 * @brief Close a file descriptor via the VFS.
 * @param fd File descriptor.
 * @return 0 on success, negative on error.
 */
int VFS_Close(int fd);

/**
 * @brief Read from a file descriptor via the VFS.
 * @param fd File descriptor.
 * @param buffer Destination buffer.
 * @param size Maximum number of bytes to read.
 * @return Number of bytes read, or negative on error.
 */
int VFS_Read(int fd, void *buffer, int size);

/**
 * @brief Write to a file descriptor via the VFS.
 * @param fd File descriptor.
 * @param buffer Source buffer.
 * @param size Number of bytes to write.
 * @return Number of bytes written, or negative on error.
 */
int VFS_Write(int fd, const void *buffer, int size);

/**
 * @brief List directory contents via the VFS.
 * @param path Directory path.
 * @param outBuffer Output buffer for listing.
 * @param maxLen Maximum buffer length.
 * @return Number of bytes written, or negative on error.
 */
int VFS_List(const char *path, char *outBuffer, int maxLen);

#endif // MICRO_RTOS_VFS_H