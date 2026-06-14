#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRIVER_TYPE_UNKNOWN = 0,
    DRIVER_TYPE_UART,
    DRIVER_TYPE_I2C,
    DRIVER_TYPE_SPI,
    DRIVER_TYPE_GPIO,
    DRIVER_TYPE_BLOCK,
    DRIVER_TYPE_CHAR,
} DriverType;

typedef struct Driver Driver;

typedef int (*driver_init_t)(Driver *drv);
typedef int (*driver_shutdown_t)(Driver *drv);
typedef int (*driver_open_t)(Driver *drv, void *ctx);
typedef int (*driver_close_t)(Driver *drv, void *ctx);
typedef int (*driver_read_t)(Driver *drv, void *buf, size_t len);
typedef int (*driver_write_t)(Driver *drv, const void *buf, size_t len);
typedef int (*driver_ioctl_t)(Driver *drv, int cmd, void *arg);

struct Driver {
    const char *name;            // human-readable name (not owned)
    DriverType type;             // hardware type hint
    uint32_t caps;               // capability flags (driver-defined)

    /* lifecycle callbacks (optional) */
    driver_init_t init;
    driver_shutdown_t shutdown;

    /* standard device ops (optional) */
    driver_open_t open;
    driver_close_t close;
    driver_read_t read;
    driver_write_t write;
    driver_ioctl_t ioctl;

    /* opaque driver context (driver-managed) */
    void *context;

    /* If non-zero, driver requests a temporary MPU region of this size
       while running to access special buffers. Kernel may honor this. */
    size_t mpu_region_size;
};

/* Registry API */
int Driver_Register(Driver *drv);
Driver *Driver_Find(const char *name);
int Driver_InitAll(void);
size_t Driver_List(Driver **out, size_t max);

#ifdef __cplusplus
}
#endif

#endif // DRIVER_H
