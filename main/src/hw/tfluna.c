/*****************************************************************************
 * tfluna.c - The source to communicate with TF-Luna LIDAR sensor
 * 
 * The point of this interface is to modularize the TF-Luna code and present
 * an easy to use interface to the application.
 *****************************************************************************/

#include "tfluna.h"

#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "logger.h"
#include "i2c_al.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

#define GET_UINT16_LSB(x) (x & 0x00FF)
#define GET_UINT16_MSB(x) ((x >> 8) & 0x00FF)
#define MAKE_UINT16(msb,lsb) ((((uint16_t)msb) << 8) + lsb) 

// Distance range limits
static const uint16_t TFL_RANGE_CM_MIN = 10; // Somewhat arbitrary
static const uint16_t TFL_RANGE_CM_MAX = 800;

//! This mutex is used to serialize public function calls
static pthread_mutex_t tfl_api_lock = PTHREAD_MUTEX_INITIALIZER;

typedef struct
{
    bool initialized;
    int i2c_hndl;
    uint16_t limit_min;
    uint16_t limit_max;
    tfluna_cal_t cal;
    uint16_t cal_min_cm;
    uint16_t cal_max_cm;
    uint8_t cal_cnt;

} tfl_data_t;

#define MAX_NUMBER_OF_HANDLES (1)
static tfl_data_t tfl_fd_data[MAX_NUMBER_OF_HANDLES] = {0};
#define MAX_NUMBER_OF_RD_WR_BYTES (16)
static uint8_t tfl_buf[MAX_NUMBER_OF_RD_WR_BYTES];

// Returns a cm value with calibration applied
static float apply_calibration(int hndl, uint16_t cm);
// Returns the calibration factor for the given cm value
static float get_cal_interpolated_factor(int hndl, uint16_t cm);
// Returns true if valid, otherwise false
static bool isValidCalIdxAndCm(int hndl, uint8_t idx, uint16_t cm);
static void log_device_details(int hndl);
// Returns 0 on success, otherwise -1
static int reg_read(int hndl, uint8_t reg, uint8_t *data, uint8_t size);
// Returns 0 on success, otherwise -1
static int reg_write(int hndl, uint8_t reg, const uint8_t *data, uint8_t size);
// Returns 0 on success, otherwise -1
static int setup_device(const tfluna_hardware_t *hw, i2c_al_device_t *dev);
static void update_cal_params(int hndl);
// Returns 0 on success, otherwise -1
static int verify_device(int hndl);

/*****************************************************************************
 * @brief tfluna_init - open an instance of the TF-Luna driver
 * 
 * @details Performs initialization of an instance of the TF-Luna driver
 *
 * @return Returns an instance handle on success, otherwise -1
 *****************************************************************************/

int tfluna_init(const tfluna_hardware_t *hw)
{
    i2c_al_device_t dev;
    int hndl = -1;
    int tmp_hndl = 0; // Default to the first and only FD
    int i2c_hndl = -1;

    pthread_mutex_lock(&tfl_api_lock);

    if (tfl_fd_data[tmp_hndl].initialized != false) {
        LOG_WRN("No resources available");
    } else if (setup_device(hw, &dev) != 0) {
        LOG_WRN("Device setup failed");
    } else if ((i2c_hndl = i2c_al_init(&dev)) < 0) {
        LOG_WRN("I2C init failed");
    } else {
        tfl_fd_data[tmp_hndl].i2c_hndl = i2c_hndl;
        if (verify_device(tmp_hndl) == 0) {
            hndl = tmp_hndl;
            tfl_fd_data[hndl].initialized = true;
            memset(&tfl_fd_data[hndl].cal, 0, sizeof(tfl_fd_data[hndl].cal));
            update_cal_params(hndl);
            tfl_fd_data[hndl].limit_min = (uint16_t) apply_calibration(hndl, TFL_RANGE_CM_MIN);
            tfl_fd_data[hndl].limit_max = (uint16_t) apply_calibration(hndl, TFL_RANGE_CM_MAX);
            log_device_details(hndl);
        }
    }

    pthread_mutex_unlock(&tfl_api_lock);

    return hndl;
}

/*******************************************************************************
 * @brief tfluna_exit - close/exit an open instance of the TF-Luna driver
 *
 * @details Performs de-initialization of an instance of the TF-Luna driver
 *
 * @return None
 ******************************************************************************/

void tfluna_exit(int hndl)
{
    pthread_mutex_lock(&tfl_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else {
        // Shutdown the uart
        i2c_al_exit(tfl_fd_data[hndl].i2c_hndl);
        // Mark as exit'd
        tfl_fd_data[hndl].initialized = false;
    }

    pthread_mutex_unlock(&tfl_api_lock);
}

/*******************************************************************************
 * @brief tfluna_get - perform a distance measurement
 *
 * @details Performs a distance and amplitude measurement of the TF-Luna driver
 *
 * @return Returns 0 on success, otherwise -1
 ******************************************************************************/

int tfluna_get(int hndl, uint16_t *cm, uint16_t *amplitude)
{
    int rc = -1;

    pthread_mutex_lock(&tfl_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else if (cm == NULL && amplitude == NULL) {
        LOG_WRN("Invalid params");
    } else {
        uint8_t buf[4];

        rc = reg_read(hndl, TFL_REG_DIST_LOW, buf, 4);

        if (cm != NULL) {
            uint16_t in = MAKE_UINT16(buf[1], buf[0]);
            *cm = apply_calibration(hndl, in);
        }
        if (amplitude != NULL) {
            *amplitude = MAKE_UINT16(buf[3], buf[2]);
        }
    }

    pthread_mutex_unlock(&tfl_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief tfluna_get_status - retrieve detailed status for the sensor
 *
 * @details Retrieves a set of status values for the TF-Luna sensor
 *
 * @return Returns 0 on success and status via passed pointers, otherwise -1
 ******************************************************************************/

int tfluna_get_status(int hndl, tfluna_status_t *status)
{
    int rc = -1;

    pthread_mutex_lock(&tfl_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else if (status == NULL) {
        LOG_WRN("Invalid params");
    } else {
        uint8_t buf[10];

        rc = reg_read(hndl, TFL_REG_DIST_LOW, buf, 10);

            status->cm = MAKE_UINT16(buf[1], buf[0]);
            status->amplitude = MAKE_UINT16(buf[3], buf[2]);
            status->temp = MAKE_UINT16(buf[5], buf[4]);
            status->ticks = MAKE_UINT16(buf[7], buf[6]);
            status->error = MAKE_UINT16(buf[9], buf[8]);
            memcpy(&status->cal, &tfl_fd_data[hndl].cal, sizeof(status->cal));
            status->limit_min = (uint16_t) apply_calibration(hndl, TFL_RANGE_CM_MIN);
            status->limit_max = (uint16_t) apply_calibration(hndl, TFL_RANGE_CM_MAX);
    }

    pthread_mutex_unlock(&tfl_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief tfluna_get_limits - retrieve working sensor range limits
 *
 * @details Retrieves the sensor limits, min and max expected distances
 *
 * @return Returns 0 on success, otherwise -1
 ******************************************************************************/

int tfluna_get_limits(int hndl, uint16_t *min, uint16_t *max)
{
    int rc = -1;

    pthread_mutex_lock(&tfl_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else {
        if (min != NULL) {
            *min = tfl_fd_data[hndl].limit_min;
        }
        if (max != NULL) {
            *max = tfl_fd_data[hndl].limit_max;
        }
        rc = 0;
    }

    pthread_mutex_unlock(&tfl_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief tfluna_set_cal_point - set a sensor cal point for a given index
 *
 * @details Sets the sensor calibration point for the given index
 *
 * @return Returns 0 on success, otherwise -1
 ******************************************************************************/

int tfluna_set_cal_point(int hndl, uint8_t idx, uint16_t cm, float factor)
{
    int rc = -1;

    pthread_mutex_lock(&tfl_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else if (!isValidCalIdxAndCm(hndl, idx, cm)) {
        // Already reported the issue
    } else {
        tfl_fd_data[hndl].cal.cm[idx] = cm;
        tfl_fd_data[hndl].cal.factor[idx] = factor;
        update_cal_params(hndl);
        tfl_fd_data[hndl].limit_min = (uint16_t) apply_calibration(hndl, TFL_RANGE_CM_MIN);
        tfl_fd_data[hndl].limit_max = (uint16_t) apply_calibration(hndl, TFL_RANGE_CM_MAX);
        rc = 0;
    }

    pthread_mutex_unlock(&tfl_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief tfluna_reg_read - test/debug function to perform a register read
 *
 * @details Perform a register read of the sensor for test/debug purposes  
 *
 * @return Returns 0 on success (and read data via *data), otherwise -1
 ******************************************************************************/

int tfluna_reg_read(int hndl, uint8_t reg, uint8_t *data)
{
    int rc = -1;

    pthread_mutex_lock(&tfl_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else if (data == NULL) {
        LOG_WRN("Invalid params");
    } else {
        rc = reg_read(hndl, reg, data, 1);
    }

    pthread_mutex_unlock(&tfl_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief tfluna_reg_write - test/debug function to perform a register write
 *
 * @details Perform a write command to the sensor for test/debug purposes  
 *
 * @return Returns 0 on success, otherwise -1
 ******************************************************************************/

int tfluna_reg_write(int hndl, uint8_t reg, uint8_t data)
{
    int rc = -1;

    pthread_mutex_lock(&tfl_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else {
        rc = reg_write(hndl, reg, &data, 1);
    }

    pthread_mutex_unlock(&tfl_api_lock);

    return rc;
}

/*****************************************************************************/

static float apply_calibration(int hndl, uint16_t cm)
{
    uint16_t adjusted = cm;
    float factor = 1.0f;
    uint8_t cnt = tfl_fd_data[hndl].cal_cnt;

    if (cnt == 0) {
        // There are no cals, so return the input value
    } else if (cnt == 1) {
        // Apply the same factor to all given cm values
        adjusted = (uint16_t)(cm * tfl_fd_data[hndl].cal.factor[0]);
    } else {
        // Apply below min cm
        if (cm <= tfl_fd_data[hndl].cal_min_cm) {
            factor = tfl_fd_data[hndl].cal.factor[0];
        }
        // Apply above max cm
        else if (cm >= tfl_fd_data[hndl].cal_max_cm) {
            factor = tfl_fd_data[hndl].cal.factor[cnt - 1];
        }
        // Apply interpolated factor
        else {
            factor = get_cal_interpolated_factor(hndl, cm);
        }

        adjusted = (uint16_t)(cm * factor);
    }

    return adjusted;
}

static float get_cal_interpolated_factor(int hndl, uint16_t cm)
{
    uint8_t cnt = tfl_fd_data[hndl].cal_cnt;
    float factor = 1.0f;

    // For this function to be call, there must be more than one cal point

    // Find the cal range
    int upper_idx = 1;
    for (; upper_idx < cnt; upper_idx++) {
        if (cm <= tfl_fd_data[hndl].cal.cm[upper_idx]) {
            break;
        }
    }

    if (upper_idx >= cnt) {
        // This should not occur since there was already a max check
        // Set that factor to the max value
        factor = tfl_fd_data[hndl].cal.factor[cnt - 1];
    } else {
        int lower_idx = upper_idx - 1;
        uint16_t cm_delta = tfl_fd_data[hndl].cal.cm[upper_idx] - tfl_fd_data[hndl].cal.cm[lower_idx];
        float factor_delta = tfl_fd_data[hndl].cal.factor[upper_idx] - tfl_fd_data[hndl].cal.factor[lower_idx];

        float interp_factor = (((float)cm) - tfl_fd_data[hndl].cal.cm[lower_idx]) / ((float)cm_delta);
        factor = tfl_fd_data[hndl].cal.factor[lower_idx] + (factor_delta * interp_factor);
    }

    return factor;
}

static bool isValidCalIdxAndCm(int hndl, uint8_t idx, uint16_t cm)
{
    bool valid = false;

    if (idx >= TFL_MAX_CAL_POINTS) {
        LOG_WRN("Index to large");
    } else if ((idx > 0) && (tfl_fd_data[hndl].cal.cm[idx - 1] == TFL_UNUSED_CAL_CM)) {
        LOG_WRN("Invalid cal index");
    } else if ((cm < TFL_RANGE_CM_MIN) || (cm > TFL_RANGE_CM_MAX)) {
        LOG_WRN("Invalid cal cm");
    } else if ((idx > 0) && (cm <= tfl_fd_data[hndl].cal.cm[idx - 1])) {
        LOG_WRN("Invalid cm, too small for idx");
    } else if ((idx < (TFL_MAX_CAL_POINTS - 1)) && (tfl_fd_data[hndl].cal.cm[idx + 1] != TFL_UNUSED_CAL_CM) && (cm >= tfl_fd_data[hndl].cal.cm[idx + 1])) {
        LOG_WRN("Invalid cm, too large for idx");
    } else {
        valid = true;
    }

    return valid;
}

static void log_device_details(int hndl)
{
    uint8_t buf[15];

    if (reg_read(hndl, TFL_REG_VERSION_REV, buf, 3) == 0) {
        LOG_INF("TF-Luna VER .: %u.%u.%u", buf[2], buf[1], buf[0]);
    }
    if (reg_read(hndl, TFL_REG_SN_0_ASCII, buf, 14) == 0) {
        buf[14] = 0; // Terminate the string
        LOG_INF("TF-Luna SN ..: %s", buf);
    }
}

static int reg_read(int hndl, uint8_t reg, uint8_t *data, uint8_t size)
{
    int rc = -1;

    if (size >= (MAX_NUMBER_OF_RD_WR_BYTES - 1)) {
        LOG_WRN("Invalid params");
    } else {
        tfl_buf[0] = reg;

        rc = i2c_al_write(tfl_fd_data[hndl].i2c_hndl, tfl_buf, 1);
        if (rc != 0) {
            LOG_ERR("Write failed");
        } else {
            rc = i2c_al_read(tfl_fd_data[hndl].i2c_hndl, data, size, NULL, 0);
            if (rc != 0) {
                LOG_ERR("Read failed");
            }
        }
    }

    return rc;
}

static int reg_write(int hndl, uint8_t reg, const uint8_t *data, uint8_t size)
{
    int rc = -1;

    if (size >= (MAX_NUMBER_OF_RD_WR_BYTES - 1)) {
        LOG_WRN("Invalid params");
    } else {
        tfl_buf[0] = reg;
        for (int i = 0; i < size; i++) {
            tfl_buf[i + 1] = data[i];
        }

        rc = i2c_al_write(tfl_fd_data[hndl].i2c_hndl, tfl_buf, (size + 1));
        if (rc != 0) {
            LOG_ERR("Write failed");
        }
    }

    return rc;
}

static int setup_device(const tfluna_hardware_t *hw, i2c_al_device_t *dev)
{
    int rc = -1;

    if (hw == NULL) {
        LOG_INF("Invalid params");
    } else {
#if defined(ZEPHYR_BUILD)
        dev->i2c_dev = hw->i2c_dev;
        dev->i2c_addr = hw->i2c_addr;
        rc = 0;
#elif defined(MAC_BUILD)
#elif defined(WINDOWS_BUILD)
#endif
    }

    return rc;
}

static void update_cal_params(int hndl)
{
    // Determine the cal count
    uint8_t cnt = 0;
    for (int i = 0; i < TFL_MAX_CAL_POINTS; i++) {
        if (tfl_fd_data[hndl].cal.cm[i] == TFL_UNUSED_CAL_CM) {
            break;
        }
        cnt ++;
    }

    // Set cal params
    tfl_fd_data[hndl].cal_cnt = cnt;
    if (cnt == 0) {
        tfl_fd_data[hndl].cal_min_cm = 0;
        tfl_fd_data[hndl].cal_max_cm = 0;
    } else {
        tfl_fd_data[hndl].cal_min_cm = tfl_fd_data[hndl].cal.cm[0];
        tfl_fd_data[hndl].cal_max_cm = tfl_fd_data[hndl].cal.cm[cnt - 1];
    }
}

static int verify_device(int hndl)
{
    int rc = -1;

    uint8_t buf[4];

    if (reg_read(hndl, TFL_REG_SIGNATURE_L, buf, 4) == 0) {
        if (buf[0] == 'L' && buf[1] == 'U' && buf[2] == 'N' && buf[3] == 'A') {
            rc = 0;
        }
    }

    return rc;
}
