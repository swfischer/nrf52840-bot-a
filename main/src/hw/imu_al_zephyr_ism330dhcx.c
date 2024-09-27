/*****************************************************************************
 * imu_al_zephyr_ism330dhcx.h - An abstraction layer (al) interface to an IMU
 * (Inertial Measurement Unit) sensor, specifically an ST ISM330DHCX.
 *
 * The point of this abstraction layer is to modularize the code wrapping the
 * OS sensor driver to project an easy to use interface and allow multiple
 * implementations based on the OS the code is running under.
 *****************************************************************************/

#include "imu_al.h"

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#include "logger.h"
#include "os_al.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

//! The number of milliseconds before the gyro integration data is reset.
//! If the last gyro sample is more than this number of ms ago, the integration
//! data will be reset.
#define IMU_GYRO_INTEGRATION_RESET_TIME_MS (3000)

//! Conversion factor from radians to degrees (180/PI).
#define IMU_RAD_TO_DEG (57.29578f)

typedef struct
{
    bool initialized;
    const struct device *imu_dev;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	float gyro_int[3]; // Gyro integration values
	float gyro_int_last[3]; // Gyro integration last values
	uint32_t gyro_int_last_ts;

} imu_al_data_t;

#define MAX_NUMBER_OF_HANDLES (1)
static imu_al_data_t imu_al_fd_data[MAX_NUMBER_OF_HANDLES] = {0};

// Returns 0 on success, otherwise -1
static int config_ism330dhcx(const struct device *ism330dhcx);
// Returns 0 on success, otherwise -1
static int fetch_data(int hndl);
// Returns 0 on success, otherwise -1
static int get_accel_data(int hndl, imu_accel_t *accel);
// Returns 0 on success, otherwise -1
static int get_gyro_data(int hndl, imu_gyro_t *gyro);
static void get_gyro_int_data(int hndl, imu_gyro_int_t *gyro_int);
static void gyro_integration(int hndl, imu_gyro_t *gyro);

/*******************************************************************************
 * @brief imu_al_init - initialize the IMU AL for use.
 *
 * @details A routine used to initialize the given IMU for use.
 *
 * @param hw - a struct denoting the IMU hardware to initialize.
 *
 * @return returns an instance handle on success, otherwise -1.
 ******************************************************************************/

int imu_al_init(const imu_al_hw_t *hw)
{
    int hndl = -1;
    int tmp_hndl = 0; // Default to the first and only handle

    if (hw == NULL) {
        LOG_WRN("Invalid params");
    } else if (imu_al_fd_data[tmp_hndl].initialized != false) {
        LOG_INF("Already initialized");
    } else { 
        imu_al_data_t *fd_data = &imu_al_fd_data[tmp_hndl];

        fd_data->imu_dev = hw->imu_dev;

        if (!device_is_ready(fd_data->imu_dev)) {
            LOG_WRN("IMU device not found");
		} else if (config_ism330dhcx(fd_data->imu_dev) != 0) {
            LOG_WRN("IMU device config failed");
        } else {
            fd_data->initialized = true;
            hndl = tmp_hndl;
        }
    }

    return hndl;
}

/*******************************************************************************
 * @brief imu_al_exit - de-initialize the IMU device
 *
 * @details A routine used to revert the initialization of the IMU device.
 *
 * @param hndl - the instance handle to close.
 *
 * @return none.
 ******************************************************************************/

void imu_al_exit(int hndl)
{
    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (imu_al_fd_data[hndl].initialized == true) {
        imu_al_fd_data[hndl].initialized = false;
    }
}

/*******************************************************************************
 * @brief imu_al_accel_read - perform read and return accelerometer data
 *
 * @details A routine used to perform a read of the IMU device and retrieve the
 * accelerometer data.
 *
 * @param hndl - the IMU instance handle.
 * @param accel - pointer to return the accelerometer data into.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int imu_al_accel_read(int hndl, imu_accel_t *accel)
{
    int rc = -1;

    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (imu_al_fd_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
    } else if (accel == NULL) {
        LOG_WRN("Invalid param");
    } else if (fetch_data(hndl) != 0) {
		// Return an error
	} else if (get_accel_data(hndl, accel) != 0) {
		// Return an error
	} else {
		rc = 0;
	}

    return rc;
}

/*******************************************************************************
 * @brief imu_al_get_gyro_int - retrieve the integrated gyroscope data
 *
 * @details A routine used to retrieves the integrated (summed) gyroscope data.
 * The integration data will automatically reset if the time from the last
 * sampling of gyro data was >3 seconds ago.
 *
 * @param hndl - the IMU instance handle.
 * @param gyro_int - pointer to return the gyroscope intergation data into.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int imu_al_get_gyro_int(int hndl, imu_gyro_int_t *gyro_int)
{
	int rc = -1;

    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (imu_al_fd_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
    } else if (gyro_int == NULL) {
        LOG_WRN("Invalid param");
	} else {
		get_gyro_int_data(hndl, gyro_int);
		rc = 0;
	}

	return rc;
}

/*******************************************************************************
 * @brief imu_al_gyro_read - perform read and return gyroscope data
 *
 * @details A routine used to perform a read of the IMU device and retrieve the
 * gyroscope data.
 *
 * @param hndl - the IMU instance handle.
 * @param gyro - pointer to return the gyroscope data into.
 * @param gyro_int - pointer to return the gyroscope integration data into.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int imu_al_gyro_read(int hndl, imu_gyro_t *gyro, imu_gyro_int_t *gyro_int)
{
    int rc = -1;

    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (imu_al_fd_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
    } else if (gyro == NULL) {
        LOG_WRN("Invalid param");
    } else if (fetch_data(hndl) != 0) {
		// Return an error
	} else if (get_gyro_data(hndl, gyro) != 0) {
		// Return an error
	} else {
		if (gyro_int != NULL) {
			get_gyro_int_data(hndl, gyro_int);
		}
		rc = 0;
	}

    return rc;
}

/*******************************************************************************
 * @brief imu_al_read - perform read and return accelerometer & gyroscope data
 *
 * @details A routine used to perform a read of the IMU device and retrieve both
 * the accelerometer and gyroscope data.
 *
 * @param hndl - the IMU instance handle.
 * @param accel - pointer to return the accelerometer data into.
 * @param gyro - pointer to return the gyroscope data into.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int imu_al_read(int hndl, imu_accel_t *accel, imu_gyro_t *gyro)
{
    int rc = -1;

    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (imu_al_fd_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
    } else if (accel == NULL || gyro == NULL) {
        LOG_WRN("Invalid param");
    } else if (fetch_data(hndl) != 0) {
		// Return an error
	} else if (get_accel_data(hndl, accel) != 0) {
		// Return an error
	} else if (get_gyro_data(hndl, gyro) != 0) {
		// Return an error
	} else {
		rc = 0;
	}

    return rc;
}

/*******************************************************************************
 * @brief imu_al_reset_gyro_int - reset the gyro integration values
 *
 * @details A routine used to reset the gyro integration values.
 *
 * @param hndl - the IMU instance handle.
 *
 * @return None.
 ******************************************************************************/

extern void imu_al_reset_gyro_int(int hndl)
{
    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (imu_al_fd_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
	} else {
    	imu_al_data_t *fd_data = &imu_al_fd_data[hndl];
		fd_data->gyro_int[0] = 0;
		fd_data->gyro_int[1] = 0;
		fd_data->gyro_int[2] = 0;
	}
}

/*****************************************************************************/

static int config_ism330dhcx(const struct device *ism330dhcx)
{
	struct sensor_value odr_attr;
	struct sensor_value fs_attr;
	int rc = -1;

	do { // Using do/while(0) as quick escape mechanism

		// Set accel sampling frequency to 52 Hz
		odr_attr.val1 = 52;
		odr_attr.val2 = 0;
		if (sensor_attr_set(ism330dhcx, SENSOR_CHAN_ACCEL_XYZ,
					SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
			LOG_WRN("Set accel sampling freq failed");
			break;
		}

		sensor_g_to_ms2(16, &fs_attr);
		if (sensor_attr_set(ism330dhcx, SENSOR_CHAN_ACCEL_XYZ,
					SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
			LOG_WRN("Set accel scale failed\n");
			break;
		}

		// Set gyro sampling frequency to 52 Hz
		odr_attr.val1 = 52;
		odr_attr.val2 = 0;
		if (sensor_attr_set(ism330dhcx, SENSOR_CHAN_GYRO_XYZ,
					SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
			LOG_WRN("Set gyro sampling freq failed");
			break;
		}

		sensor_degrees_to_rad(250, &fs_attr);
		if (sensor_attr_set(ism330dhcx, SENSOR_CHAN_GYRO_XYZ,
					SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
			LOG_WRN("Set gyro scale failed\n");
			break;
		}

		rc = 0;

	} while (0);

	return rc;
}

static int fetch_data(int hndl)
{
	int rc = 0;

	if (sensor_sample_fetch(imu_al_fd_data[hndl].imu_dev) < 0) {
		LOG_ERR("Sample fetch error");
		rc = -1;
	}

	return rc;
}

static int get_accel_data(int hndl, imu_accel_t *accel)
{
    imu_al_data_t *fd_data = &imu_al_fd_data[hndl];
	int rc = -1;

	if (sensor_channel_get(fd_data->imu_dev, SENSOR_CHAN_ACCEL_XYZ, fd_data->accel) != 0) {
		LOG_ERR("Accel get error");
	} else {
		accel->x_mpss = sensor_value_to_float(&fd_data->accel[0]);
		accel->y_mpss = sensor_value_to_float(&fd_data->accel[1]);
		accel->z_mpss = sensor_value_to_float(&fd_data->accel[2]);
		rc = 0;
	}

	return rc;
}

static int get_gyro_data(int hndl, imu_gyro_t *gyro)
{
    imu_al_data_t *fd_data = &imu_al_fd_data[hndl];
	int rc = -1;

	if (sensor_channel_get(fd_data->imu_dev, SENSOR_CHAN_GYRO_XYZ, fd_data->gyro) != 0) {
		LOG_ERR("Gyro get error");
	} else {
		// The returned sensor value is radian/sec and we want degrees/sec
		gyro->x_dps = sensor_value_to_float(&fd_data->gyro[0]) * IMU_RAD_TO_DEG;
		gyro->y_dps = sensor_value_to_float(&fd_data->gyro[1]) * IMU_RAD_TO_DEG;
		gyro->z_dps = sensor_value_to_float(&fd_data->gyro[2]) * IMU_RAD_TO_DEG;

		gyro_integration(hndl, gyro);

		rc = 0;
	}

	return rc;
}

static void get_gyro_int_data(int hndl, imu_gyro_int_t *gyro_int)
{
    imu_al_data_t *fd_data = &imu_al_fd_data[hndl];

	gyro_int->x_deg = fd_data->gyro_int[0];
	gyro_int->y_deg = fd_data->gyro_int[1];
	gyro_int->z_deg = fd_data->gyro_int[2];
}

static void gyro_integration(int hndl, imu_gyro_t *gyro)
{
    imu_al_data_t *fd_data = &imu_al_fd_data[hndl];
	uint32_t now = os_al_get_ms32();

	// Reset the integration data if the last sample was too far in the past
	if ((now - fd_data->gyro_int_last_ts) > IMU_GYRO_INTEGRATION_RESET_TIME_MS) {
		fd_data->gyro_int[0] = 0;
		fd_data->gyro_int[1] = 0;
		fd_data->gyro_int[2] = 0;
	} else {
		// Take the mid-point between the current and last samples
//		float mid_x = ((fd_data->gyro_int_last[0] - gyro->x_dps) / 2) + gyro->x_dps;
//		float mid_y = ((fd_data->gyro_int_last[1] - gyro->y_dps) / 2) + gyro->y_dps;
//		float mid_z = ((fd_data->gyro_int_last[2] - gyro->z_dps) / 2) + gyro->z_dps;
		// Determine the sample time factor (portion of a second)
		float factor = (now - fd_data->gyro_int_last_ts) / 1000.0f;
		// Add (integrate) the new sample
//		fd_data->gyro_int[0] += (mid_x * factor);
//		fd_data->gyro_int[1] += (mid_y * factor);
//		fd_data->gyro_int[2] += (mid_z * factor);
		fd_data->gyro_int[0] += (gyro->x_dps * factor);
		fd_data->gyro_int[1] += (gyro->y_dps * factor);
		fd_data->gyro_int[2] += (gyro->z_dps * factor);
	}

	// Update the saved params
	fd_data->gyro_int_last[0] = gyro->x_dps;
	fd_data->gyro_int_last[1] = gyro->y_dps;
	fd_data->gyro_int_last[2] = gyro->z_dps;
	fd_data->gyro_int_last_ts = now;
}
