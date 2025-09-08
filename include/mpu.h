#ifndef MPU_H
#define MPU_H

#include <stdint.h>
#include "mpu_armv7.h"

/**
 * MPU region configuration structure.
 * - baseAddress: start address of the region
 * - size: size in bytes (will be rounded up to nearest power of two >= 32)
 * - attributes: pre-encoded RASR attribute bits (AP, TEX, C, B, S, XN, SRD)
 *               SIZE and ENABLE bits are added automatically.
 */
typedef struct {
    uint32_t baseAddress;
    uint32_t size;
    uint32_t attributes;
} MPURegion;

/**
 * Initialize MPU: disables MPU, clears all regions.
 */
void MPU_Init(void);

/**
 * Configure a specific MPU region.
 * @param regionNumber Index of the region (0..N-1)
 * @param region Pointer to MPURegion struct
 */
void MPU_ConfigureRegion(uint8_t regionNumber, const MPURegion *region);

/**
 * Enable MPU with background default map for privileged accesses.
 */
void MPU_Enable(void);

/**
 * Disable MPU entirely.
 */
void MPU_Disable(void);

/**
 * Get number of supported MPU regions.
 */
uint32_t MPU_GetRegionCount(void);

#endif // MPU_H