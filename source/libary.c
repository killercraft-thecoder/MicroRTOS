#include "library.h"
//#include "fs.h"
#include "malloc.h"
#include "thread.h"
#include <string.h>
#include <limits.h>

// Small helper to advance the file by reading and discarding bytes.
static int FS_Skip(int fd, size_t n)
{
    if (n == 0)
        return 0;
    char tmp[128];
    while (n > 0)
    {
        size_t toread = n > sizeof(tmp) ? sizeof(tmp) : n;
        int r = FS_Read(fd, tmp, (int)toread);
        if (r <= 0)
            return -1;
        n -= (size_t)r;
    }
    return 0;
}

//
// Internal helpers
//

static uint32_t read_u32(const uint8_t *buf, uint32_t off)
{
    uint32_t v;
    memcpy(&v, buf + off, sizeof(uint32_t));
    return v;
}

static void *load_section(int fd, uint32_t offset, uint32_t size)
{
    if (size == 0)
        return NULL;

    void *mem = Malloc(size);
    if (!mem)
        return NULL;
    int rr = FS_Read(fd, mem, (int)size);
    if (rr < (int)size)
    {
        Free(mem);
        return NULL;
    }
    return mem;
}

//
// ─────────────────────────────────────────────
//   LIBRARY LOADING
// ─────────────────────────────────────────────
//

Library *Library_Load(const char *path)
{
    int fd = FS_Open(path, 0);
    if (fd < 0)
        return NULL;

    uint8_t headerBuf[64];
    if (FS_Read(fd, headerBuf, sizeof(headerBuf)) < (int)sizeof(headerBuf)) {
        FS_Close(fd);
        return NULL;
    }

    uint32_t magic = read_u32(headerBuf, 0);
    if (magic != 0x4C494200u) { // "LIB\0"
        FS_Close(fd);
        return NULL;
    }

    uint32_t codeOffset  = read_u32(headerBuf, 8);
    uint32_t codeSize    = read_u32(headerBuf, 12);
    uint32_t dataOffset  = read_u32(headerBuf, 16);
    uint32_t dataSize    = read_u32(headerBuf, 20);
    uint32_t bssSize     = read_u32(headerBuf, 24);
    uint32_t metaOffset  = read_u32(headerBuf, 28);
    uint32_t relocOffset = read_u32(headerBuf, 32);
    uint32_t relocCount  = read_u32(headerBuf, 36);

    Library *lib = Malloc(sizeof(Library));
    if (!lib) {
        FS_Close(fd);
        return NULL;
    }
    memset(lib, 0, sizeof(Library));

    // Load code
    // (assumes FS_Read advances; if you need seeking, add FS_Seek)
    if (codeOffset < sizeof(headerBuf)) {
        FS_Close(fd);
        Free(lib);
        return NULL;
    }
    if (FS_Skip(fd, codeOffset - sizeof(headerBuf)) < 0) {
        FS_Close(fd);
        Free(lib);
        return NULL;
    }

    lib->code = Malloc(codeSize);
    if (!lib->code) {
        FS_Close(fd);
        Free(lib);
        return NULL;
    }
    if (FS_Read(fd, lib->code, (int)codeSize) < (int)codeSize) {
        FS_Close(fd);
        Free(lib->code);
        Free(lib);
        return NULL;
    }

    // Load data
    if (dataOffset < codeOffset + codeSize) {
        FS_Close(fd);
        if (lib->code) Free(lib->code);
        Free(lib);
        return NULL;
    }
    if (FS_Skip(fd, dataOffset - (codeOffset + codeSize)) < 0) {
        FS_Close(fd);
        if (lib->code) Free(lib->code);
        Free(lib);
        return NULL;
    }
    lib->data = NULL;
    if (dataSize > 0) {
        lib->data = Malloc(dataSize);
        if (!lib->data) {
            FS_Close(fd);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
        if (FS_Read(fd, lib->data, (int)dataSize) < (int)dataSize) {
            FS_Close(fd);
            Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
    }

    // Allocate BSS
    lib->bss = NULL;
    if (bssSize > 0) {
        lib->bss = Malloc(bssSize);
        if (!lib->bss) {
            FS_Close(fd);
            if (lib->data) Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
        memset(lib->bss, 0, bssSize);
    }

    // Load metadata (name, version, exports)
    if (metaOffset < dataOffset + dataSize) {
        FS_Close(fd);
        if (lib->data) Free(lib->data);
        Free(lib->code);
        Free(lib);
        return NULL;
    }
    if (FS_Skip(fd, metaOffset - (dataOffset + dataSize)) < 0) {
        FS_Close(fd);
        if (lib->data) Free(lib->data);
        Free(lib->code);
        Free(lib);
        return NULL;
    }
    uint8_t metaBuf[32];
    if (FS_Read(fd, metaBuf, (int)sizeof(metaBuf)) < (int)sizeof(metaBuf)) {
        FS_Close(fd);
        if (lib->data) Free(lib->data);
        Free(lib->code);
        Free(lib);
        return NULL;
    }

    uint32_t exportCount       = read_u32(metaBuf, 0);
    uint32_t exportTableOffset = read_u32(metaBuf, 4);
    uint32_t nameOffset        = read_u32(metaBuf, 8);
    lib->versionMajor          = read_u32(metaBuf, 12);
    lib->versionMinor          = read_u32(metaBuf, 16);

    // Load name
    if (nameOffset < metaOffset + sizeof(metaBuf)) {
        FS_Close(fd);
        if (lib->data) Free(lib->data);
        Free(lib->code);
        Free(lib);
        return NULL;
    }
    if (FS_Skip(fd, nameOffset - (metaOffset + sizeof(metaBuf))) < 0) {
        FS_Close(fd);
        if (lib->data) Free(lib->data);
        Free(lib->code);
        Free(lib);
        return NULL;
    }
    if (FS_Read(fd, lib->name, (int)sizeof(lib->name)) < (int)sizeof(lib->name)) {
        FS_Close(fd);
        if (lib->data) Free(lib->data);
        Free(lib->code);
        Free(lib);
        return NULL;
    }

    // Load export table
    lib->exportCount = exportCount;
    lib->exports = NULL;
    if (exportCount > 0) {
        if (exportCount > 1024) { // sane upper bound
            FS_Close(fd);
            if (lib->bss)  Free(lib->bss);
            if (lib->data) Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
        lib->exports = Malloc(sizeof(LibraryExport) * exportCount);
        if (!lib->exports) {
            FS_Close(fd);
            if (lib->bss)  Free(lib->bss);
            if (lib->data) Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
        if (exportTableOffset < nameOffset + 32) {
            FS_Close(fd);
            if (lib->exports) Free(lib->exports);
            if (lib->bss)  Free(lib->bss);
            if (lib->data) Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
        if (FS_Skip(fd, exportTableOffset - (nameOffset + 32)) < 0) {
            FS_Close(fd);
            if (lib->exports) Free(lib->exports);
            if (lib->bss)  Free(lib->bss);
            if (lib->data) Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
        if (FS_Read(fd, lib->exports, (int)(sizeof(LibraryExport) * exportCount)) < (int)(sizeof(LibraryExport) * exportCount)) {
            FS_Close(fd);
            if (lib->exports) Free(lib->exports);
            if (lib->bss)  Free(lib->bss);
            if (lib->data) Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
    }

    // Load relocation table
    lib->relocCount = relocCount;
    lib->relocs = NULL;
    if (relocCount > 0) {
        if (relocCount > 65536) {
            FS_Close(fd);
            if (lib->exports) Free(lib->exports);
            if (lib->bss)     Free(lib->bss);
            if (lib->data)    Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
        lib->relocs = Malloc(sizeof(RelocEntry) * relocCount);
        if (!lib->relocs) {
            FS_Close(fd);
            if (lib->exports) Free(lib->exports);
            if (lib->bss)     Free(lib->bss);
            if (lib->data)    Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
        if (relocOffset < exportTableOffset + sizeof(LibraryExport) * exportCount) {
            FS_Close(fd);
            if (lib->relocs) Free(lib->relocs);
            if (lib->exports) Free(lib->exports);
            if (lib->bss)     Free(lib->bss);
            if (lib->data)    Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
        if (FS_Skip(fd, relocOffset - (exportTableOffset + sizeof(LibraryExport) * exportCount)) < 0) {
            FS_Close(fd);
            if (lib->relocs) Free(lib->relocs);
            if (lib->exports) Free(lib->exports);
            if (lib->bss)     Free(lib->bss);
            if (lib->data)    Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }
        if (FS_Read(fd, lib->relocs, (int)(sizeof(RelocEntry) * relocCount)) < (int)(sizeof(RelocEntry) * relocCount)) {
            FS_Close(fd);
            if (lib->relocs) Free(lib->relocs);
            if (lib->exports) Free(lib->exports);
            if (lib->bss)     Free(lib->bss);
            if (lib->data)    Free(lib->data);
            Free(lib->code);
            Free(lib);
            return NULL;
        }

        /* Apply relocations (ensure implementations validate entries) */
        apply_relocations(lib->code, lib->data, lib->relocs, lib->relocCount);
    }

    FS_Close(fd);
    return lib;
}


void Library_Unload(Library *lib)
{
    if (!lib)
        return;

    if (lib->code) Free(lib->code);
    if (lib->data) Free(lib->data);
    if (lib->bss)  Free(lib->bss);
    if (lib->exports) Free(lib->exports);

    Free(lib);
}

void *Library_FindSymbol(Library *lib, const char *name)
{
    if (!lib || !name)
        return NULL;

    for (uint32_t i = 0; i < lib->exportCount; i++)
    {
        if (strcmp(lib->exports[i].name, name) == 0)
        {
            uint32_t off = lib->exports[i].offset;
            return (uint8_t *)lib->code + off;
        }
    }
    return NULL;
}

int Library_Call(Library *lib, const char *symbol, void *args)
{
    void *fn = Library_FindSymbol(lib, symbol);
    if (!fn)
        return -1;

    typedef int (*LibFunc)(void *);
    LibFunc f = (LibFunc)fn;
    return f(args);
}

//
// ─────────────────────────────────────────────
//   EXECUTABLE LOADING
// ─────────────────────────────────────────────
//

Executable *Executable_Load(const char *path)
{
    int fd = FS_Open(path, 0);
    if (fd < 0)
        return NULL;

    uint8_t headerBuf[64];
    if (FS_Read(fd, headerBuf, sizeof(headerBuf)) < (int)sizeof(headerBuf)) {
        FS_Close(fd);
        return NULL;
    }

    uint32_t magic = read_u32(headerBuf, 0);
    if (magic != 0x45584500u) { // "EXE\0"
        FS_Close(fd);
        return NULL;
    }

    uint32_t codeOffset  = read_u32(headerBuf, 8);
    uint32_t codeSize    = read_u32(headerBuf, 12);
    uint32_t dataOffset  = read_u32(headerBuf, 16);
    uint32_t dataSize    = read_u32(headerBuf, 20);
    uint32_t bssSize     = read_u32(headerBuf, 24);
    uint32_t metaOffset  = read_u32(headerBuf, 28);
    uint32_t relocOffset = read_u32(headerBuf, 32);
    uint32_t relocCount  = read_u32(headerBuf, 36);

    Executable *exe = Malloc(sizeof(Executable));
    if (!exe) {
        FS_Close(fd);
        return NULL;
    }
    memset(exe, 0, sizeof(Executable));

    // Load code
    FS_Read(fd, NULL, codeOffset - sizeof(headerBuf));
    exe->code = Malloc(codeSize);
    if (!exe->code) {
        FS_Close(fd);
        Free(exe);
        return NULL;
    }
    FS_Read(fd, exe->code, codeSize);

    // Load data
    FS_Read(fd, NULL, dataOffset - (codeOffset + codeSize));
    exe->data = NULL;
    if (dataSize > 0) {
        exe->data = Malloc(dataSize);
        if (!exe->data) {
            FS_Close(fd);
            Free(exe->code);
            Free(exe);
            return NULL;
        }
        FS_Read(fd, exe->data, dataSize);
    }

    // Allocate BSS
    exe->bss = NULL;
    if (bssSize > 0) {
        exe->bss = Malloc(bssSize);
        if (!exe->bss) {
            FS_Close(fd);
            if (exe->data) Free(exe->data);
            Free(exe->code);
            Free(exe);
            return NULL;
        }
        memset(exe->bss, 0, bssSize);
    }

    // Load metadata (entry offset)
    FS_Read(fd, NULL, metaOffset - (dataOffset + dataSize));
    uint8_t metaBuf[8];
    FS_Read(fd, metaBuf, sizeof(metaBuf));

    uint32_t entryOffset = read_u32(metaBuf, 0);
    exe->entry = (void (*)(void))((uint8_t *)exe->code + entryOffset);

    // Load relocation table
    exe->relocCount = relocCount;
    exe->relocs = NULL;
    if (relocCount > 0) {
        exe->relocs = Malloc(sizeof(RelocEntry) * relocCount);
        if (!exe->relocs) {
            FS_Close(fd);
            if (exe->bss)  Free(exe->bss);
            if (exe->data) Free(exe->data);
            Free(exe->code);
            Free(exe);
            return NULL;
        }
        FS_Read(fd, NULL, relocOffset - (metaOffset + sizeof(metaBuf)));
        FS_Read(fd, exe->relocs, sizeof(RelocEntry) * relocCount);

        apply_relocations(exe->code, exe->data, exe->relocs, exe->relocCount);
    }

    FS_Close(fd);
    return exe;
}


int Executable_Start(Executable *exe)
{
    if (!exe || !exe->entry)
        return -1;

    // Create a thread for the executable
    static thread_t t;
    static uint32_t stack[512];

    Create_Thread(&t, (void (*)(void *))exe->entry, NULL,
                  stack, sizeof(stack), 1, "exe");

    exe->mainThread = &t;
    return 0;
}

void Executable_Unload(Executable *exe)
{
    if (!exe)
        return;

    if (exe->code) Free(exe->code);
    if (exe->data) Free(exe->data);
    if (exe->bss)  Free(exe->bss);

    Free(exe);
}
