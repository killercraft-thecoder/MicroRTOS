#include "vfs.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------
// FD map (linked list, unlimited)
// ---------------------------------------------------------

typedef struct VFS_FdMap
{
    int userFd;
    FileSystemDriver *driver;
    int driverFd;
    struct VFS_FdMap *next;
} VFS_FdMap;

static VFS_FdMap *fdMapHead = NULL;
static int nextUserFd = 3; // 0,1,2 reserved for stdin/out/err

// ---------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------

static VFS_FdMap *VFS_FindFd(int userFd)
{
    VFS_FdMap *n = fdMapHead;
    while (n)
    {
        if (n->userFd == userFd)
            return n;
        n = n->next;
    }
    return NULL;
}

static int VFS_AddFd(FileSystemDriver *drv, int driverFd)
{
    VFS_FdMap *node = malloc(sizeof(VFS_FdMap));
    if (!node)
        return -1;

    node->userFd  = nextUserFd++;
    node->driver  = drv;
    node->driverFd = driverFd;

    node->next = fdMapHead;
    fdMapHead = node;

    return node->userFd;
}

static void VFS_RemoveFd(int userFd)
{
    VFS_FdMap *prev = NULL;
    VFS_FdMap *n = fdMapHead;

    while (n)
    {
        if (n->userFd == userFd)
        {
            if (prev)
                prev->next = n->next;
            else
                fdMapHead = n->next;

            free(n);
            return;
        }
        prev = n;
        n = n->next;
    }
}

static const char *VFS_StripPrefix(const char *path)
{
    if (!path || path[0] != '/')
        return path;

    const char *p = path + 1; // skip leading '/'

    // Skip prefix
    while (*p && *p != '/')
        p++;

    // Skip the slash after prefix, if present
    if (*p == '/')
        p++;

    return p; // subpath
}

static FileSystemDriver *VFS_FindDriverForPath(const char *path)
{
    if (!path || path[0] != '/')
        return NULL;

    // Extract prefix: "/sd/foo" -> "sd"
    const char *p = path + 1; // skip leading '/'
    char prefix[16];
    int idx = 0;

    while (*p && *p != '/' && idx < (int)(sizeof(prefix) - 1))
    {
        prefix[idx++] = *p++;
    }
    prefix[idx] = '\0';

    // If no prefix (path = "/foo"), fallback to first driver
    bool hasPrefix = (idx > 0);

    size_t count = sizeof(g_kernel.fsDrivers) / sizeof(g_kernel.fsDrivers[0]);

    // Try prefix match first
    if (hasPrefix)
    {
        for (size_t i = 0; i < count; i++)
        {
            FileSystemDriver *drv = g_kernel.fsDrivers[i];
            if (!drv || !drv->prefix)
                continue;

            if (strcmp(drv->prefix, prefix) == 0)
                return drv;
        }
    }

    // Fallback: first mounted driver
    for (size_t i = 0; i < count; i++)
    {
        if (g_kernel.fsDrivers[i] != NULL)
            return g_kernel.fsDrivers[i];
    }

    return NULL;
}

// ---------------------------------------------------------
// Public VFS API
// ---------------------------------------------------------

int VFS_RegisterDriver(FileSystemDriver *driver)
{
    if (!driver)
        return -1;

    size_t count = sizeof(g_kernel.fsDrivers) / sizeof(g_kernel.fsDrivers[0]);

    for (size_t i = 0; i < count; i++)
    {
        if (g_kernel.fsDrivers[i] == NULL)
        {
            g_kernel.fsDrivers[i] = driver;
            return 0;
        }
    }

    return -1; // no space
}

int VFS_Open(const char *path, int flags)
{
    FileSystemDriver *drv = VFS_FindDriverForPath(path);
    if (!drv || !drv->open)
        return -1;

    const char *subpath = VFS_StripPrefix(path);

    int driverFd = drv->open(subpath, flags);
    if (driverFd < 0)
        return -1;

    return VFS_AddFd(drv, driverFd);
}

int VFS_Close(int userFd)
{
    VFS_FdMap *entry = VFS_FindFd(userFd);
    if (!entry || !entry->driver || !entry->driver->close)
        return -1;

    int result = entry->driver->close(entry->driverFd);
    VFS_RemoveFd(userFd);
    return result;
}

int VFS_Read(int userFd, void *buffer, int size)
{
    VFS_FdMap *entry = VFS_FindFd(userFd);
    if (!entry || !entry->driver || !entry->driver->read)
        return -1;

    return entry->driver->read(entry->driverFd, buffer, size);
}

int VFS_Write(int userFd, const void *buffer, int size)
{
    VFS_FdMap *entry = VFS_FindFd(userFd);
    if (!entry || !entry->driver || !entry->driver->write)
        return -1;

    return entry->driver->write(entry->driverFd, buffer, size);
}

int VFS_List(const char *path, char *outBuffer, int maxLen)
{
    FileSystemDriver *drv = VFS_FindDriverForPath(path);
    if (!drv || !drv->list)
        return -1;

    const char *subpath = VFS_StripPrefix(path);

    return drv->list(subpath, outBuffer, maxLen);
}