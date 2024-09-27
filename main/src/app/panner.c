/*****************************************************************************
 * panner.h - The source to pan the sensor left and right
 * 
 * The point of this interface is to provide the application with a simple
 * means of panning the sensor left and right.
 *****************************************************************************/

#include "panner.h"

#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>

#include "logger.h"
#include "lx16a.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

#if defined(ZEPHYR_BUILD)
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
#elif defined(MAC_BUILD)
static const char uart_dev[] = "/dev/cu.usbserial-0001";
#elif defined(WINDOWS_BUILD)
static const char uart_dev[] = "COM1";
#endif

#define UART_BAUD_RATE (115200)

#define PANNING_SERVO_ID (2)

// Log a warning message if the movement angle and the expected angle are
// different by more than this value
#define PANNER_WARN_DELTA (2)

//! This mutex is used to serialize public function calls
static pthread_mutex_t panner_api_lock = PTHREAD_MUTEX_INITIALIZER;

static int lx16a_hndl = -1;
static int16_t lx16a_angle_mid = 0;
static int16_t panner_limit_min = 0;
static int16_t panner_limit_max = 0;
#define POSITION_UNKNOWN (-1000)  // Should never reach this value
static int16_t panner_position = 0;

static void determine_limits(void);
static int move_to(int16_t pos);
static int move_to_valid_position(void);
static void sleep_ms(uint32_t ms);
static int update_current_position(void);

int panner_init(void)
{
    int rc = -1;

    pthread_mutex_lock(&panner_api_lock);

    if (lx16a_hndl != -1) {
		LOG_WRN("Driver already init'd");
    } else {
        lx16a_hardware_t hw;

        determine_limits(); // This can be done at anytime

        hw.uart = uart_dev;
        hw.baud = UART_BAUD_RATE;

        int hndl = lx16a_init(&hw);
        if (hndl < 0) {
            LOG_WRN("Driver init failed");
        } else {
            lx16a_hndl = hndl;
            if (update_current_position() != 0) {
                LOG_WRN("Driver access failed");
                lx16a_hndl = -1;
            } else if (move_to_valid_position() != 0) {
                LOG_WRN("Driver move failed");
                lx16a_hndl = -1;
            } else {
                rc = 0;
            }
        }
    }

    pthread_mutex_unlock(&panner_api_lock);

    return rc;
}

void panner_exit(void)
{
    pthread_mutex_lock(&panner_api_lock);

    if (lx16a_hndl == -1) {
		LOG_INF("Driver already exited");
    } else {
        lx16a_exit(lx16a_hndl);
        lx16a_hndl = -1;
    }

    pthread_mutex_unlock(&panner_api_lock);
}

void panner_get_limits(int16_t *min, int16_t *max)
{
    pthread_mutex_lock(&panner_api_lock);

    if (min != NULL) {
        *min = panner_limit_min;
    }
    if (max != NULL) {
        *max = panner_limit_max;
    }

    pthread_mutex_unlock(&panner_api_lock);
}

// Returns 0 on success, otherwise -1
int panner_enable(bool enable)
{
    int rc = -1;

    pthread_mutex_lock(&panner_api_lock);

    if (lx16a_hndl == -1) {
		LOG_WRN("Driver not init'd");
    } else {
        rc = lx16a_set_all_enable_states(lx16a_hndl, enable);
        // Give some time for the power state change to occur
        usleep(20000); // 20ms
    }

    pthread_mutex_unlock(&panner_api_lock);

    return rc;
}

// Returns 0 on success, otherwise -1
int panner_get_position(int16_t *pos)
{
    int rc = -1;

    pthread_mutex_lock(&panner_api_lock);

    if (lx16a_hndl == -1) {
		LOG_WRN("Driver not init'd");
    } else if (pos == NULL) {
		LOG_INF("Invalid param");
    } else {
        *pos = panner_position;
        rc = 0;
    }

    pthread_mutex_unlock(&panner_api_lock);

    return rc;
}

// Returns 0 on success, otherwise -1
int panner_set_position(int16_t pos)
{
    int rc = -1;

    pthread_mutex_lock(&panner_api_lock);

    if (lx16a_hndl == -1) {
		LOG_WRN("Driver not init'd");
    } else if (move_to(pos) != 0) {
		// Issue already logged
    } else {
        rc = 0;
    }

    pthread_mutex_unlock(&panner_api_lock);

    return rc;
}

/*****************************************************************************/

static void determine_limits(void)
{
    // We want the center to be 0
    float mid = floor(LX16A_ANGLE_RANGE_DEG_MAX / 2);

    lx16a_angle_mid = (int16_t) mid;
    panner_limit_max = lx16a_angle_mid;
    panner_limit_min = -panner_limit_max;
}

static int move_to(int16_t pos)
{
    int rc = -1;

    if ((pos < panner_limit_min) || (pos > panner_limit_max)) {
        LOG_WRN("Invalid params");
    } else {
        int delta_move = abs(panner_position - pos);
        uint32_t ms = LX16A_ANGLE_TIMING_RANGE_MS_MIN * delta_move;
        float angle = pos + lx16a_angle_mid;

        int ret = lx16a_set_angle(lx16a_hndl, PANNING_SERVO_ID, angle, ms);
        if (ret != 0) {
            LOG_WRN("Move failed");
        } else {
            sleep_ms(ms); // Don't return until the movement is complete
            panner_position = pos; // Just to give the update function to expected position
            update_current_position();
            rc = 0;
        }
    }

    return rc;
}

static int move_to_valid_position(void)
{
    int rc = -1;

    if (panner_position != POSITION_UNKNOWN) {
        rc = 0; // Already in a valid position
    } else if (move_to(panner_limit_max - 1) != 0) { // Abitrarily choose max limit to move to
        LOG_WRN("Move failed");
    } else if (panner_position == POSITION_UNKNOWN) {
        // Something went wrong
        LOG_WRN("Still unknown position");
    } else {
        rc = 0;
    }

    return rc;
}

static void sleep_ms(uint32_t ms)
{
    // usleep() does not support values over 1 seconds, so have to split between sec's and usec's
    uint32_t sec = ms / 1000;
    uint32_t usec = (ms % 1000) * 1000;

    if (sec > 0) {
        sleep(sec);
    }
    if (usec > 0) {
        usleep(usec);
    }
}

static int update_current_position(void)
{
    int rc = -1;

    float angle = lx16a_get_angle(lx16a_hndl, PANNING_SERVO_ID);
    if (angle < -0.1f) {
        LOG_WRN("Get angle failure (%d/%d.%03d)", panner_position, (int)angle, (((int)(angle * 1000)) % 1000));
    } else if ((angle < LX16A_ANGLE_RANGE_DEG_MIN) || (angle > LX16A_ANGLE_RANGE_DEG_MAX)) {
        panner_position = POSITION_UNKNOWN;
    } else {
        int16_t pos = ((int16_t) angle) - lx16a_angle_mid;
        // Check the limits just to be sure
        if ((pos < panner_limit_min) || (pos > panner_limit_max)) {
            panner_position = POSITION_UNKNOWN;
        } else {
            int16_t delta = panner_position - pos;
            if ((delta > PANNER_WARN_DELTA) || (delta < -PANNER_WARN_DELTA)) {
                LOG_WRN("Angle delta is >%u at %d (%d)", PANNER_WARN_DELTA, panner_position, delta);
            }
            panner_position = pos;
        }
        rc = 0;
    }

    return rc;
}
