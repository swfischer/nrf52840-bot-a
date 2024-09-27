/*****************************************************************************
 * imu_al.h - An abstraction layer (al) interface to an IMU (Inertial
 * Measurement Unit) sensor.
 *
 * The point of this abstraction layer is to modularize the code wrapping the
 * OS sensor driver to project an easy to use interface and allow multiple
 * implementations based on the OS the code is running under.
 *****************************************************************************/

#ifndef IMU_AL_H
#define IMU_AL_H

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C"
{
#endif

// A structure intended to abtract the initialzation data as it changes based
// on the platform being compiled for.
typedef struct {
#if defined(ZEPHYR_BUILD)
    const struct device *imu_dev;
#elif defined(MAC_BUILD)
    // TBD
#elif defined(WINDOWS_BUILD)
    // TBD
#endif
} imu_al_hw_t;

// Units are mpss (meters per second squared)
// Note: 1 g == 9.80665 mpss
typedef struct {
    float x_mpss;
    float y_mpss;
    float z_mpss;
} imu_accel_t;

// Units are dps (degrees per second)
typedef struct {
    float x_dps;
    float y_dps;
    float z_dps;
} imu_gyro_t;

// Units are degrees changed since last reset
// Note: "int" is for "integration" or sumation of results
typedef struct {
    float x_deg;
    float y_deg;
    float z_deg;
} imu_gyro_int_t;

// Returns 0 on success, otherwise -1
extern int imu_al_init(const imu_al_hw_t *hw);
extern void imu_al_exit(int hndl);

// Returns 0 on success, otherwise -1
extern int imu_al_accel_read(int hndl, imu_accel_t *accel);
// Returns 0 on success, otherwise -1
extern int imu_al_get_gyro_int(int hndl, imu_gyro_int_t *gyro_int);
// Returns 0 on success, otherwise -1
extern int imu_al_gyro_read(int hndl, imu_gyro_t *gyro, imu_gyro_int_t *gyro_int);
// Returns 0 on success, otherwise -1
extern int imu_al_read(int hndl, imu_accel_t *accel, imu_gyro_t *gyro);
extern void imu_al_reset_gyro_int(int hndl);

#ifdef __cplusplus
}
#endif

#endif // IMU_AL_H
