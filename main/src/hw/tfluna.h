/*****************************************************************************
 * tfluna.h - The interface to communicate with TF-Luna LIDAR sensor
 * 
 * The point of this interface is to modularize the TF-Luna code and present
 * an easy to use interface to the application.
 *****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#if defined(ZEPHYR_BUILD)
#include <zephyr/device.h>
#endif

#ifndef TFLUNA_H
#define TFLUNA_H

#define TFLUNA_I2C_ADDR (0x10)

// I2C register list
#define TFL_REG_DIST_LOW (0x00)
#define TFL_REG_DIST_HIGH (0x01)
#define TFL_REG_AMP_LOW (0x02)
#define TFL_REG_AMP_HIGH (0x03)
#define TFL_REG_TEMP_LOW (0x04)
#define TFL_REG_TEMP_HIGH (0x05)
#define TFL_REG_TICK_LOW (0x06)
#define TFL_REG_TICK_HIGH (0x07)
#define TFL_REG_ERROR_LOW (0x08)
#define TFL_REG_ERROR_HIGH (0x09)
#define TFL_REG_VERSION_REV (0x0A)
#define TFL_REG_VERSION_MINOR (0x0B)
#define TFL_REG_VERSION_MAJOR (0x0C)
#define TFL_REG_SN_0_ASCII (0x10)
#define TFL_REG_SN_1_ASCII (0x11)
#define TFL_REG_SN_2_ASCII (0x12)
#define TFL_REG_SN_3_ASCII (0x13)
#define TFL_REG_SN_4_ASCII (0x14)
#define TFL_REG_SN_5_ASCII (0x15)
#define TFL_REG_SN_6_ASCII (0x16)
#define TFL_REG_SN_7_ASCII (0x17)
#define TFL_REG_SN_8_ASCII (0x18)
#define TFL_REG_SN_9_ASCII (0x19)
#define TFL_REG_SN_10_ASCII (0x1A)
#define TFL_REG_SN_11_ASCII (0x1B)
#define TFL_REG_SN_12_ASCII (0x1C)
#define TFL_REG_SN_13_ASCII (0x1D)
#define TFL_REG_UL_PWR (0x1F)
#define TFL_REG_SAVE (0x20)
#define TFL_REG_REBOOT (0x21)
#define TFL_REG_SLAVE_ADDR (0x22)
#define TFL_REG_MODE (0x23)
#define TFL_REG_TRIG_ONE_SHOT (0x24)
#define TFL_REG_ENABLE (0x25)
#define TFL_REG_FPS_LOW (0x26)
#define TFL_REG_FPS_HIGH (0x27)
#define TFL_REG_LOW_PWR (0x28)
#define TFL_REG_RESTORE (0x29)
#define TFL_REG_AMP_THR_LOW (0x2A)
#define TFL_REG_AMP_THR_HIGH (0x2B)
#define TFL_REG_DUMMY_DIST_LOW (0x2C)
#define TFL_REG_DUMMY_DIST_HIGH (0x2D)
#define TFL_REG_MIN_DIST_LOW (0x2E)
#define TFL_REG_MIN_DIST_HIGH (0x2F)
#define TFL_REG_MAX_DIST_LOW (0x30)
#define TFL_REG_MAX_DIST_HIGH (0x31)
#define TFL_REG_SIGNATURE_L (0x3C)
#define TFL_REG_SIGNATURE_U (0x3D)
#define TFL_REG_SIGNATURE_N (0x3E)
#define TFL_REG_SIGNATURE_A (0x3F)

// Amplitude range limits
static const uint16_t TFL_AMPLITUDE_TOO_LOW = 100; // Too dark to get a reliable reading
static const uint16_t TFL_AMPLITUDE_TOO_HIGH = 30000; // Too bright to get a reliable reading

// A structure intended to abtract the initialzation data as it changes based
// on the platform being compiled for.
typedef struct {
    uint32_t i2c_addr;
#if defined(ZEPHYR_BUILD)
    const struct device *i2c_dev;
#elif defined(MAC_BUILD)
    const char *uart;
    uint32_t baud;
#elif defined(WINDOWS_BUILD)
    const char *uart;
    uint32_t baud;
#endif
} tfluna_hardware_t;

#define TFL_MAX_CAL_POINTS (6)
#define TFL_UNUSED_CAL_CM (0)
typedef struct {
    uint16_t cm[TFL_MAX_CAL_POINTS];
    float  factor[TFL_MAX_CAL_POINTS];
} tfluna_cal_t;

// Returns an instance handle for the created driver on success, otherwise -1
extern int tfluna_init(const tfluna_hardware_t *hw);
extern void tfluna_exit(int hndl);

// Return 0 on success, otherwise -1
extern int tfluna_get(int hndl, uint16_t *cm, uint16_t *amplitude);

// Returns 0 on success and status via passed pointers, otherwise -1
typedef struct {
    uint16_t cm; // Measured distance in centimeters
    uint16_t amplitude; // Signal amplitude
    uint16_t temp; // Sensor tempurature in 0.01C units
    uint16_t ticks; // Timestamp
    uint16_t error; // Error value
    uint16_t limit_min; // Minimum distance expected
    uint16_t limit_max; // Maximum distance expected
    tfluna_cal_t cal; // Calibration data
} tfluna_status_t;
extern int tfluna_get_status(int hndl, tfluna_status_t *status);

// Returns 0 on success, otherwise -1
extern int tfluna_get_limits(int hndl, uint16_t *min, uint16_t *max);
// Return 0 on success, otherwise -1
extern int tfluna_set_cal_point(int hndl, uint8_t idx, uint16_t cm, float factor);

// These are really for debug and test purposes
// Returns 0 on success (and read data via *data), otherwise -1
extern int tfluna_reg_read(int hndl, uint8_t reg, uint8_t *data);
// Return 0 on success, otherwise -1
extern int tfluna_reg_write(int hndl, uint8_t reg, uint8_t data);

#endif // TFLUNA_H
