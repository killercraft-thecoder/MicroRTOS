#ifndef FS_H_INCLUDED
#define FS_H_INCLUDED

#include <stdint.h>
#include "thread.h"

API_FUNCTION(FS_Open)
int FS_Open(const char *path, int flags);

API_FUNCTION(FS_Close)
int FS_Close(int fd);

API_FUNCTION(FS_Read)
int FS_Read(int fd, void *buffer, int size);

API_FUNCTION(FS_Write)
int FS_Write(int fd, const void *buffer, int size);

API_FUNCTION(FS_List)
int FS_List(const char *path, char *outBuffer, int maxLen);

#endif /* FS_H_INCLUDED */
