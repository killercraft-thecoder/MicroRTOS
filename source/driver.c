#include "driver.h"
#include "thread.h"
#include <string.h>

#define MAX_DRIVERS 32

static Driver *g_drivers[MAX_DRIVERS];
static size_t g_driver_count = 0;

int Driver_Register(Driver *drv)
{
    if (!drv || !drv->name)
        return -1;

    // simple duplicate check
    for (size_t i = 0; i < g_driver_count; i++)
    {
        if (strcmp(g_drivers[i]->name, drv->name) == 0)
            return -1; // already registered
    }

    if (g_driver_count >= MAX_DRIVERS)
        return -1;

    g_drivers[g_driver_count++] = drv;
    return 0;
}

Driver *Driver_Find(const char *name)
{
    if (!name)
        return NULL;
    for (size_t i = 0; i < g_driver_count; i++)
    {
        if (strcmp(g_drivers[i]->name, name) == 0)
            return g_drivers[i];
    }
    return NULL;
}

int Driver_InitAll(void)
{
    int rc = 0;
    for (size_t i = 0; i < g_driver_count; i++)
    {
        Driver *d = g_drivers[i];
        if (d->init)
        {
            int r = d->init(d);
            if (r != 0)
                rc = r; // keep going but record a non-zero
        }
    }
    return rc;
}

size_t Driver_List(Driver **out, size_t max)
{
    size_t n = (g_driver_count < max) ? g_driver_count : max;
    for (size_t i = 0; i < n; i++)
        out[i] = g_drivers[i];
    return n;
}
