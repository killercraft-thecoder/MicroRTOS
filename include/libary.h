#ifndef MICRO_RTOS_LIBRARY_H
#define MICRO_RTOS_LIBRARY_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef enum
{
    RELOC_ABS32 = 0,     // *(base + offset) += base
    RELOC_DATA_ABS32 = 1 // same but for data section
} RelocType;

typedef struct
{
    uint32_t offset;  // where to patch
    uint16_t type;    // RelocType
    uint16_t section; // 0 = code, 1 = data
} RelocEntry;

static void apply_relocations(void *codeBase,
                              void *dataBase,
                              RelocEntry *relocs,
                              uint32_t relocCount)
{
    for (uint32_t i = 0; i < relocCount; i++)
    {
        RelocEntry *r = &relocs[i];

        uint8_t *base = (r->section == 0)
                            ? (uint8_t *)codeBase
                            : (uint8_t *)dataBase;

        uint32_t *patch = (uint32_t *)(base + r->offset);

        switch (r->type)
        {
        case RELOC_ABS32:
        case RELOC_DATA_ABS32:
            *patch += (uint32_t)base;
            break;

        default:
            // unknown reloc type -> ignore or error
            break;
        }
    }
}

//
// ─────────────────────────────────────────────
//   Runtime Structures (User-Facing)
// ─────────────────────────────────────────────
//

// Represents a loaded dynamic library (.LIB)
typedef struct Library
{
    // Memory regions allocated by the loader
    void *code; // executable code
    void *data; // initialized data
    void *bss;  // zero-initialized data

    // Identity information
    char name[32];
    uint32_t versionMajor;
    uint32_t versionMinor;
    uint32_t uuid[4];

    // Exported functions (resolved at load time)
    struct LibraryExport *exports;
    uint32_t exportCount;

    // inside Library
    RelocEntry *relocs;
    uint32_t relocCount;

    // inside Executable
    RelocEntry *relocs;
    uint32_t relocCount;

} Library;

// Represents a loaded executable (.EXE)
typedef struct Executable
{
    // Memory regions allocated by the loader
    void *code;
    void *data;
    void *bss;

    // Entry point for the program
    void (*entry)(void);

    // Main thread created when the executable is started
    struct thread_t *mainThread;

    // inside Library
    RelocEntry *relocs;
    uint32_t relocCount;

    // inside Executable
    RelocEntry *relocs;
    uint32_t relocCount;

} Executable;

// Export table entry for a library
typedef struct LibraryExport
{
    char name[32];   // e.g. "Set_Pixel"
    uint32_t offset; // offset into the library's code region
} LibraryExport;

//
// ─────────────────────────────────────────────
//   Public Loader API
// ─────────────────────────────────────────────
//

// Load a dynamic library (.LIB)
Library *Library_Load(const char *path);

// Unload a library
void Library_Unload(Library *lib);

// Look up a function exported by a library
void *Library_FindSymbol(Library *lib, const char *name);

// Call a library function by name (args is library‑specific)
int Library_Call(Library *lib, const char *symbol, void *args);

// Load an executable (.EXE)
Executable *Executable_Load(const char *path);

// Start an executable (creates its main thread)
int Executable_Start(Executable *exe);

// Unload an executable after it exits
void Executable_Unload(Executable *exe);

#endif // MICRO_RTOS_LIBRARY_H
