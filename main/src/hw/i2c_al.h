/*****************************************************************************
 * i2c_al.h - An abstraction layer (al) interface to an I2C bus.
 * 
 * The point of this abstraction layer is to modularize the code wrapping the
 * OS I2C driver to project an easy to use interface and allow multiple
 * implementations based on the OS the code is running under.
 *****************************************************************************/

#ifndef I2C_AL_H
#define I2C_AL_H

#include <stddef.h>
#include <stdint.h>
#if defined(ZEPHYR_BUILD)
#include <zephyr/device.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

// A structure intended to abtract the initialzation data as it changes based
// on the platform being compiled for.
typedef struct {
    uint8_t i2c_addr;
#if defined(ZEPHYR_BUILD)
    const struct device *i2c_dev;
#elif defined(MAC_BUILD)
    const char *uart;
    uint32_t baud;
#elif defined(WINDOWS_BUILD)
    const char *uart;
    uint32_t baud;
#endif
} i2c_al_device_t;

// Returns a instance handle on success, otherwise -1
extern int i2c_al_init(const i2c_al_device_t *device);
extern void i2c_al_exit(int hndl);
// Leave "wbuf" as null to just perform a read (no write first) 
// Returns 0 on success, otherwise -1
extern int i2c_al_read(int hndl, uint8_t *rbuf, uint16_t rsize, const uint8_t *wbuf, uint16_t wsize);
// Returns 0 on success, otherwise -1
extern int i2c_al_write(int hndl, const uint8_t *wbuf, uint16_t wsize);

#ifdef __cplusplus
}
#endif

#endif  // I2C_AL_H
