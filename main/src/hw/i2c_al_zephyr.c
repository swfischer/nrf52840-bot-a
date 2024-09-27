/*****************************************************************************
 * i2c_al_zephyr.c - An abstraction layer (al) interface to an I2C bus.
 * 
 * The point of this abstraction layer is to modularize the code wrapping the
 * OS I2C driver to project an easy to use interface and allow multiple
 * implementations based on the OS the code is running under.
 * 
 * It is assumed that the user of this interface will not call public
 * functions while another public function is running
 *****************************************************************************/

#include "i2c_al.h"

#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>  // for usleep()
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#include "logger.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

typedef struct
{
    bool initialized;
    const struct device *i2c_dev;
    uint8_t i2c_addr;

} i2c_al_data_t;

#define MAX_NUMBER_OF_HANDLES (1)
static i2c_al_data_t i2c_al_fd_data[MAX_NUMBER_OF_HANDLES] = {0};

/*******************************************************************************
 * @brief i2c_al_init - initialize the I2C AL for use.
 *
 * @details A routine used to initialize the given I2C for the given i2c_addr
 * and prepare the I2C bus for use.
 *
 * @param device - a device denoting the I2C bus to initialize.
 *
 * @return returns an instance handle on success, otherwise -1.
 ******************************************************************************/

int i2c_al_init(const i2c_al_device_t *device)
{
    int hndl = -1;
    int tmp_hndl = 0; // Default to the first and only handle

    if (device == NULL) {
        LOG_WRN("Invalid params");
    } else if (i2c_al_fd_data[tmp_hndl].initialized != false) {
        LOG_INF("Already initialized");
    } else { 
        i2c_al_data_t *fd_data = &i2c_al_fd_data[tmp_hndl];

        fd_data->i2c_dev = device->i2c_dev;

        if (!device_is_ready(fd_data->i2c_dev)) {
            LOG_WRN("I2C device not found");
        } else {
            fd_data->initialized = true;
            fd_data->i2c_addr = device->i2c_addr;
            hndl = tmp_hndl;
        }
    }

    return hndl;
}

/*******************************************************************************
 * @brief i2c_al_exit - de-initialize the I2C bus
 *
 * @details A routine used to revert the initialization of the I2C bus.
 *
 * @param hndl - the instance handle to close.
 *
 * @return none.
 ******************************************************************************/

void i2c_al_exit(int hndl)
{
    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (i2c_al_fd_data[hndl].initialized == true) {
        i2c_al_fd_data[hndl].initialized = false;
    }
}

/*******************************************************************************
 * @brief i2c_al_read - read data from the I2C bus
 *
 * @details A routine used to read or write/read a number of bytes from the I2C
 * bus and return the bytes via the passed in rbuf pointer.  To perform just a
 * read, pass wbuf as null and set wsize to 0.
 *
 * @param hndl - the instance handle of the UART to read from.
 * @param rbuf - a pointer to the buffer to return the read data in.
 * @param rsize - the number of bytes to read.
 * @param wbuf - a pointer to the buffer of data to write prior to the read.
 * @param wsize - the number of bytes to write.
 *
 * @return returns 0 on success, otherwise -1.
 ******************************************************************************/

int i2c_al_read(int hndl, uint8_t *rbuf, uint16_t rsize, const uint8_t *wbuf, uint16_t wsize)
{
    int rc = -1;

    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (i2c_al_fd_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
    } else if ((rbuf == NULL) || (rsize == 0U)) {
        // Just return an error
    } else {
        i2c_al_data_t *fd_data = &i2c_al_fd_data[hndl];

        int err = -1;

        if ((wbuf == NULL) || (wsize == 0U)) {
            err = i2c_read(fd_data->i2c_dev, rbuf, rsize, fd_data->i2c_addr);
        } else {
            err = i2c_write_read(fd_data->i2c_dev, fd_data->i2c_addr, wbuf, wsize, rbuf, rsize);
        }

        if (err != 0) {
            LOG_ERR("Write/read error: %d", err);
        } else {
            usleep(1000);  // This delay is often necessary to avoid i2c errors
            rc = 0;
        }
    }

    return rc;
}

/*******************************************************************************
 * @brief i2c_al_write - write data to the I2C bus
 *
 * @details A routine used to write a number of bytes from the passed buffer to
 * the I2C bus.
 *
 * @param hndl - the instance handle of the I2C bus to write to.
 * @param wbuf - a pointer to the buffer containing the data to write.
 * @param wsize - the number of bytes to write.
 *
 * @return returns 0 on success, otherwise -1
 ******************************************************************************/
int i2c_al_write(int hndl, const uint8_t *wbuf, uint16_t wsize)
{
    int rc = -1;

    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (i2c_al_fd_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
    } else if ((wbuf == NULL) || (wsize == 0U)) {
        // Just return an error
    } else {
        i2c_al_data_t *fd_data = &i2c_al_fd_data[hndl];

        int err = i2c_write(fd_data->i2c_dev, wbuf, wsize, fd_data->i2c_addr);
        if (err != 0) {
            LOG_ERR("Write error: %d", err);
        } else {
            usleep(1000);  // This delay is often necessary to avoid i2c errors
            rc = 0;
        }
    }

    return rc;
}
