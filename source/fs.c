#include "thread.h"
#include "fs.h"

API_FUNCTION(FS_Open)
int FS_Open(const char *path, int flags)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)path;
    register uint32_t r1 __asm__("r1") = (uint32_t)flags;

    __asm volatile("svc %0" ::"i"(SVC_FS_OPEN), "r"(r0), "r"(r1) : "memory");

    int fd;
    __asm volatile("mov %0, r0" : "=r"(fd));
    return fd;
}

API_FUNCTION(FS_Close)
int FS_Close(int fd)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)fd;

    __asm volatile("svc %0" ::"i"(SVC_FS_CLOSE), "r"(r0) : "memory");

    int result;
    __asm volatile("mov %0, r0" : "=r"(result));
    return result;
}

API_FUNCTION(FS_Read)
int FS_Read(int fd, void *buffer, int size)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)fd;
    register uint32_t r1 __asm__("r1") = (uint32_t)buffer;
    register uint32_t r2 __asm__("r2") = (uint32_t)size;

    __asm volatile("svc %0" ::"i"(SVC_FS_READ), "r"(r0), "r"(r1), "r"(r2) : "memory");

    int bytes;
    __asm volatile("mov %0, r0" : "=r"(bytes));
    return bytes;
}

API_FUNCTION(FS_Write)
int FS_Write(int fd, const void *buffer, int size)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)fd;
    register uint32_t r1 __asm__("r1") = (uint32_t)buffer;
    register uint32_t r2 __asm__("r2") = (uint32_t)size;

    __asm volatile("svc %0" ::"i"(SVC_FS_WRITE), "r"(r0), "r"(r1), "r"(r2) : "memory");

    int bytes;
    __asm volatile("mov %0, r0" : "=r"(bytes));
    return bytes;
}

API_FUNCTION(FS_List)
int FS_List(const char *path, char *outBuffer, int maxLen)
{
    register uint32_t r0 __asm__("r0") = (uint32_t)path;
    register uint32_t r1 __asm__("r1") = (uint32_t)outBuffer;
    register uint32_t r2 __asm__("r2") = (uint32_t)maxLen;

    __asm volatile("svc %0" ::"i"(SVC_FS_LIST), "r"(r0), "r"(r1), "r"(r2) : "memory");

    int bytes;
    __asm volatile("mov %0, r0" : "=r"(bytes));
    return bytes;
}

