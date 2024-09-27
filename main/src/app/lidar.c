/*****************************************************************************
 * lidar.h - The interface to read distances from the LIDAR sensor
 * 
 * The point of this interface is to provide the application with a simple
 * means of reading from the LIDAR sensor.
 *****************************************************************************/

#include "lidar.h"

#include <pthread.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "logger.h"
#include "tfluna.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

#if defined(ZEPHYR_BUILD)
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
//static const struct device *i2c_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(i2c1));
#define LIDAR_DT_NODE DT_NODELABEL(lidar)
#define LIDAR_I2C_NODE DT_BUS(LIDAR_DT_NODE)
#define LIDAR_I2C_ADDR DT_REG_ADDR(LIDAR_DT_NODE)
#elif defined(MAC_BUILD)
static const char uart_dev[] = "/dev/cu.usbserial-0002";
#define UART_BAUD_RATE (115200)
#elif defined(WINDOWS_BUILD)
static const char uart_dev[] = "COM2";
#define UART_BAUD_RATE (115200)
#endif

//! This mutex is used to serialize public function calls
static pthread_mutex_t lidar_api_lock = PTHREAD_MUTEX_INITIALIZER;

// Centimeters from the middle of sensor to the servo pivot axis
static const float LIDAR_SENSOR_TO_PIVOT_AXIS = 1.27f;

static int tfluna_hndl = -1;

static uint16_t tfluna_limit_min = 0;
static uint16_t tfluna_limit_max = 0;

static lidar_status_t get_status(uint16_t cm, uint16_t amplitude);
static int set_cals(int hndl);
static int setup_hardware(tfluna_hardware_t *hw);

/*****************************************************************************
 * @brief lidar_init - initialize the LIDAR module
 * 
 * @details Performs initialization of the LIDAR module
 * 
 * @return Returns 0 on success, otherwise -1
 *****************************************************************************/

int lidar_init(void)
{
    int rc = -1;

    pthread_mutex_lock(&lidar_api_lock);

    if (tfluna_hndl != -1) {
		LOG_WRN("Driver already init'd");
    } else {
        tfluna_hardware_t hw;

        if (setup_hardware(&hw) != 0) {
            LOG_WRN("HW setup failed");
        } else {
            int hndl = tfluna_init(&hw);
            if (hndl < 0) {
                LOG_WRN("Init failed");
            } else if (set_cals(hndl) != 0) {
                LOG_WRN("Set cals failed");
            } else if (tfluna_get_limits(hndl, &tfluna_limit_min, &tfluna_limit_max) != 0) {
                LOG_WRN("Get limits failed");
            } else {
                tfluna_hndl = hndl;
                rc = 0;
            }
        }
    }

    pthread_mutex_unlock(&lidar_api_lock);

    return rc;
}

/*****************************************************************************
 * @brief lidar_exit - de-initialize the LIDAR module
 * 
 * @details Performs a de-initialization of the LIDAR module
 *****************************************************************************/

void lidar_exit(void)
{
    pthread_mutex_lock(&lidar_api_lock);

    if (tfluna_hndl == -1) {
		LOG_INF("Driver already exited");
    } else {
        tfluna_exit(tfluna_hndl);
        tfluna_hndl = -1;
    }

    pthread_mutex_unlock(&lidar_api_lock);
}

/*****************************************************************************
 * @brief lidar_get_limits - retrieve the LIDAR sensor limits
 * 
 * @details Retrieves the limits, in centimeters, of the LIDAR sensor used by
 * this module
 * 
 * @return Returns 0 on success, otherwise -1
 *****************************************************************************/

int lidar_get_limits(uint16_t *min, uint16_t *max)
{
    int rc = -1;

    pthread_mutex_lock(&lidar_api_lock);

    if (tfluna_hndl == -1) {
		LOG_WRN("Driver not init'd");
    } else {
        if (min != NULL) {
            *min = tfluna_limit_min;
        }
        if (max != NULL) {
            *max = tfluna_limit_max;
        }
    }

    pthread_mutex_unlock(&lidar_api_lock);

    return rc;
}

/*****************************************************************************
 * @brief lidar_get_distance - retrieves a LIDAR sensor measurement
 * 
 * @details Retrieves a distance measure, in meters, from the LIDAR sensor
 * used by this module
 * 
 * @return Returns a lidar_status_t value denoting the status of the reading
 *****************************************************************************/

lidar_status_t lidar_get_distance(float *meters)
{
    lidar_status_t rc = LIDAR_FAILURE;

    pthread_mutex_lock(&lidar_api_lock);

    if (tfluna_hndl == -1) {
		LOG_WRN("Driver not init'd");
    } else if (meters == NULL) {
        LOG_WRN("Invalid params");
    } else {
        uint16_t cm;
        uint16_t amplitude;

        if (tfluna_get(tfluna_hndl, &cm, &amplitude) != 0) {
		    LOG_WRN("Get distance failed");
        } else {
            rc = get_status(cm, amplitude);
            if (rc == LIDAR_SUCCESS) {
                *meters = (((float)cm) + LIDAR_SENSOR_TO_PIVOT_AXIS) / 100.0f;
            }
        }
    }

    pthread_mutex_unlock(&lidar_api_lock);

    return rc;
}

/*****************************************************************************
 * @brief lidar_get_raw_cm - retrieves a LIDAR sensor measurement
 * 
 * @details Retrieves a distance measure, in centimeters, from the LIDAR
 * sensor used by this module
 * 
 * @return Returns a lidar_status_t value denoting the status of the reading
 *****************************************************************************/

extern lidar_status_t lidar_get_raw_cm(uint16_t *cm)
{
    lidar_status_t rc = LIDAR_FAILURE;

    pthread_mutex_lock(&lidar_api_lock);

    if (tfluna_hndl == -1) {
		LOG_WRN("Driver not init'd");
    } else if (cm == NULL) {
        LOG_WRN("Invalid params");
    } else {
        uint16_t raw_cm;
        uint16_t amplitude;

        if (tfluna_get(tfluna_hndl, &raw_cm, &amplitude) != 0) {
		    LOG_WRN("Get distance failed");
        } else {
            rc = get_status(raw_cm, amplitude);
            if (rc == LIDAR_SUCCESS) {
                *cm = raw_cm;
            }
        }
    }

    pthread_mutex_unlock(&lidar_api_lock);

    return rc;
}

/*****************************************************************************/

static lidar_status_t get_status(uint16_t cm, uint16_t amplitude)
{
    lidar_status_t status = LIDAR_FAILURE;

    if (cm < tfluna_limit_min) {
        status = LIDAR_TOO_CLOSE;
    } else if (cm > tfluna_limit_max) {
        status = LIDAR_TOO_FAR;
    } else if (amplitude < TFL_AMPLITUDE_TOO_LOW) {
        status = LIDAR_TOO_DARK;
    } else if (amplitude > TFL_AMPLITUDE_TOO_HIGH) {
        status = LIDAR_TOO_BRIGHT;
    } else {
        status = LIDAR_SUCCESS;
    }

    return status;
}

static int set_cals(int hndl)
{
    int rc = -1;

    if (tfluna_set_cal_point(hndl, 0, 13, 1.258065f) != 0) {
        LOG_ERR("Set cal 0 failed");
    } else if (tfluna_set_cal_point(hndl, 1, 27, 1.094595f) != 0) {
        LOG_ERR("Set cal 1 failed");
    } else if (tfluna_set_cal_point(hndl, 2, 47, 1.036765f) != 0) {
        LOG_ERR("Set cal 1 failed");
    } else if (tfluna_set_cal_point(hndl, 3, 100, 1.0f) != 0) {
        LOG_ERR("Set cal 1 failed");
    } else if (tfluna_set_cal_point(hndl, 4, 207, 1.014706f) != 0) {
        LOG_ERR("Set cal 1 failed");
    } else if (tfluna_set_cal_point(hndl, 5, 401, 01.003336f) != 0) {
        LOG_ERR("Set cal 1 failed");
    } else {
        rc = 0;
    }

    return rc;
}

static int setup_hardware(tfluna_hardware_t *hw)
{
    int rc = -1;

#if defined(ZEPHYR_BUILD)
    static const struct device *i2c_bus = DEVICE_DT_GET(LIDAR_I2C_NODE);
    hw->i2c_dev = i2c_bus;
    hw->i2c_addr = LIDAR_I2C_ADDR;
    rc = 0;
#elif defined(MAC_BUILD)
    hw->uart = uart_dev;
    hw->baud = UART_BAUD_RATE;
    hw->i2c_addr = TFLUNA_I2C_ADDR;
    rc = 0;
#elif defined(WINDOWS_BUILD)
    hw->uart = uart_dev;
    hw->baud = UART_BAUD_RATE;
    hw->i2c_addr = TFLUNA_I2C_ADDR;
    rc = 0;
#endif

    return rc;
}
