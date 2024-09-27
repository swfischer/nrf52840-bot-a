/*****************************************************************************
 * tracks.c - source file for controlling a dual independent track mechanism
 * (think tank tracks)
 *
 *****************************************************************************/

#include "tracks.h"

#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "drivetrain.h"
#include "imu_al.h"
#include "logger.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

static const struct device *const ism330dhcx = DEVICE_DT_GET_ONE(st_ism330dhcx);

static const float PI = 3.1415926f;

// Empirically determined encoder counts to distance (centimeters) factor
static const float TRKS_ENCODER_CNTS_PER_CM = 123.5f;
// Increase the maximum pivot distance to be sure we move enough for full pivot
static const float TRKS_PIVOT_FACTOR = 2.5f;
// The distance, in centimeters, between the centers of the track
static const float TRKS_CM_BETWEEN_TRACKS = 14.5f;
// The speed percentage to slow to just prior to stopping
static const int8_t TRKS_SLOW_SPEED_PERCENTAGE = 12;
// Empirically determined typical enc cnts needed to stop from the stopping speed
static const uint32_t TRKS_SLOW_SPEED_TO_STOP_EC = 15;
// For one safety check, we make sure the left-to-right delta is < 10% of travel enc cnts
static const int TRKS_SAFETY_DELTA_PERCENTAGE = 10;
// For the left-to-right delta safety check, the minimum maximum delta is 100 enc cnts
static const int TRKS_SAFETY_DELTA_MIN_EC = 100;
// The offset in degrees to slow down the pivoting speed
static const int16_t TRKS_PIVOT_SLOW_SPD_DEG_OFFSET = 5;

//! This mutex is used to serialize public function calls
static pthread_mutex_t trks_api_lock = PTHREAD_MUTEX_INITIALIZER;
//! This mutex is used to protect many reads & writes of "trks_op"
static pthread_mutex_t trks_op_lock = PTHREAD_MUTEX_INITIALIZER;

//! Flag used to know if the module has been initialized or not
static bool trks_initialized = false;
//! Min and max encoder-counts-per-second, from drivetrain
static int16_t trks_min_ecps = 0;
static int16_t trks_max_ecps = 0;
//! The last left/right encoder counts reported via the callback function
static uint32_t trks_last_left_ec = 0;
static uint32_t trks_last_right_ec = 0;
//! The last left/right encoder counts captured prior to the last movement start
//! Used for safety check purposes
static uint32_t trks_start_left_ec = 0;
static uint32_t trks_start_right_ec = 0;
//! The current system error state
static trks_errors_t trks_system_error = TRKS_ERR_NONE;
//! The handle to the IMU used when pivoting
static int trks_imu_hndl = -1;

typedef struct {
    int16_t exp_cm; // The expected number of centimeters to move
    int16_t exp_deg; // For pivot only, expected number of degrees to pivot
    int16_t slow_deg; // For pivot only, expected number of degrees to pivot before slow speed
    int16_t speed_ecps; // The calculated move speed (enc-cnts-per-second)
    uint32_t start_ec; // The enc cnt just prior to starting
    uint32_t exp_end_ec; // The expected ending enc cnt
    uint32_t slow_delta_ec; // The delta enc cnt where we slow to the stopping speed
    uint32_t stop_delta_ec; // The delta enc cnt where the stop command should be issued
    int16_t last_run_ec; // The number of enc cnts traveled during the last move
    int16_t last_run_deg; // The number of degrees pivoted during the last pivot
} trks_op_t;

static trks_op_t trks_move;
static trks_op_t trks_pivot;
static trks_op_t* trks_op = NULL;

static struct k_work trks_callback_worker;
static struct k_work trks_slow_worker;
static struct k_work trks_stop_worker;

// The registered callback function for the tracks API
static tracks_cb_t trks_callback_func = NULL;
static tracks_event_t trks_callback_event;

// Returns the enc-cnts-per-sec value from a speed percentage
static int16_t calc_ecps(int8_t speed_percent);
static void callback_work_handler(struct k_work *work);
// Returns the desired move speed for the given distance
static int16_t determine_move_speed(int16_t cm, bool move_not_pivot);
// Returns the desired move speed for the given distance
static int16_t determine_pivot_distance(int16_t degrees);
// Returns the expected degrees for the pivot to slow down
static int16_t determine_pivot_slow_degrees(int16_t degrees);
// The periodic callback function from the drivetrain
static void drivetrain_cb(const drivetrain_status_t *status);
// Returns the expected movement ending encoder count based on distance to travel
static uint32_t get_exp_move_end_ec(uint32_t start_ec, int16_t cm);
// Returns the degrees pivoted thus far
static float get_pivoted_deg(void);
// Returns an number of encoder counts prior to the end we should slow down to the stopping speed
static uint32_t get_slow_delta_ec(int16_t ecps);
// Returns an encoder count value to be used as the entire mech position
static uint32_t get_tracks_ec(void);
// Returns 0 on success, otherwise -1
static int init_imu(void);
static void perform_safety_checks(void);
// Returns true if the movement should switch to the slow speed movement
static bool should_slow_move(uint32_t delta_to_end, float pivoted_deg);
// Returns true if the movement should stop the movement
static bool should_stop_move(uint32_t delta_to_end, float pivoted_deg);
static void slow_work_handler(struct k_work *work);
static void stop_work_handler(struct k_work *work);

/*******************************************************************************
 * @brief tracks_init - initialize the drivetrain for use.
 *
 * @details A routine used to initialize the drivetrain for use.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int tracks_init(void)
{
    int rc = -1;

    pthread_mutex_lock(&trks_api_lock);

    if (trks_initialized) {
        LOG_WRN("Already init'd");
	} else if (drivetrain_init() != 0) {
		LOG_ERR("Drivetrain init failed");
    } else if (drivetrain_reg_cb(drivetrain_cb) != 0) {
		LOG_ERR("Drivetrain cb reg failed");
        drivetrain_exit();
    } else if (init_imu() != 0) {
        // Already logged the issue
        drivetrain_dereg_cb(drivetrain_cb);
        drivetrain_exit();
    } else {
        drivetrain_get_motion_limits(&trks_min_ecps, &trks_max_ecps);
        k_work_init(&trks_callback_worker, callback_work_handler);
        k_work_init(&trks_slow_worker, slow_work_handler);
        k_work_init(&trks_stop_worker, stop_work_handler);
        trks_callback_func = NULL;
        trks_op = NULL;
        trks_system_error = TRKS_ERR_NONE;
        trks_initialized = true;
        rc = 0;
    }

    pthread_mutex_unlock(&trks_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief tracks_exit - de-initialize the drivetrain.
 *
 * @details A routine used to de-initialize the drivetrain.
 *
 * @return None.
 ******************************************************************************/

void tracks_exit(void)
{
    pthread_mutex_lock(&trks_api_lock);

    if (trks_initialized) {
        drivetrain_dereg_cb(drivetrain_cb);
        drivetrain_exit();
        if (k_work_is_pending(&trks_callback_worker)) {
            (void)k_work_cancel(&trks_callback_worker);
        }
        if (k_work_is_pending(&trks_stop_worker)) {
            (void)k_work_cancel(&trks_stop_worker);
        }
        trks_initialized = false;
    }

    pthread_mutex_unlock(&trks_api_lock);
}

/*******************************************************************************
 * @brief tracks_reg_cb - register a callback function for event cb's.
 *
 * @details A routine used to register for event callbacks during regular
 * operation.  These callbacks should be handled like ISRs in that the
 * processing done within them must be short (non-blocking) and should not call
 *  any of the tracks APIs.
 *
 * @param cb - the callback function to register.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int tracks_reg_cb(tracks_cb_t cb)
{
    int rc = -1;

    pthread_mutex_lock(&trks_api_lock);

    if (!trks_initialized) {
        LOG_WRN("Not init'd");
    } else if (trks_callback_func != NULL) {
        LOG_WRN("Callback already registered");
    } else {
        trks_callback_func = cb;
        rc = 0;
    }

    pthread_mutex_unlock(&trks_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief tracks_dereg_cb - de-register a previously registered callback.
 *
 * @details A routine used to de-register a tracks callback function.
 *
 * @return None.
 ******************************************************************************/

void tracks_dereg_cb(tracks_cb_t cb)
{
    pthread_mutex_lock(&trks_api_lock);

    if (!trks_initialized) {
        LOG_WRN("Not init'd");
    } else if (trks_callback_func != cb) {
        LOG_WRN("Callback not registered");
    } else {
        trks_callback_func = NULL;
    }

    pthread_mutex_unlock(&trks_api_lock);
}

/*******************************************************************************
 * @brief tracks_stop - stops all motion of the drive train.
 *
 * @details A routine used to halts the motion of both motors of the drivetrain.
 *
 * @return None.
 ******************************************************************************/

void tracks_stop(void)
{
    pthread_mutex_lock(&trks_api_lock);

    if (!trks_initialized) {
        LOG_WRN("Not init'd");
    } else {
        drivetrain_stop();
    }

    pthread_mutex_unlock(&trks_api_lock);
}

/*******************************************************************************
 * @brief tracks_move - move forward or backward a given distance.
 *
 * @details A routine used to move the track mechanism forward (positive values)
 * or reverse (negative values) a given distance in centimeters.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int tracks_move(int16_t centimeters)
{
    int rc = -1;

    pthread_mutex_lock(&trks_api_lock);

    if (!trks_initialized) {
        LOG_WRN("Not init'd");
    } else if (centimeters == 0) {
        LOG_WRN("Invalid param");
    } else if (trks_op != NULL) {
        LOG_WRN("Operation in progress");
    } else {
        trks_move.exp_deg = 0; // Mark as not a pivot operation
        trks_move.speed_ecps = determine_move_speed(centimeters, true);
        trks_move.start_ec = get_tracks_ec();
        trks_move.exp_end_ec = get_exp_move_end_ec(trks_move.start_ec, centimeters);
        trks_move.slow_delta_ec = get_slow_delta_ec(trks_move.speed_ecps);
        trks_move.stop_delta_ec = TRKS_SLOW_SPEED_TO_STOP_EC;
        trks_start_left_ec = trks_last_left_ec;
        trks_start_right_ec = trks_last_right_ec;
        if (drivetrain_set_motion(trks_move.speed_ecps, trks_move.speed_ecps) != 0) {
            LOG_ERR("Move failed");
        } else {
            // Cannot set exp_cm or move_ec until we know we are going to be moving
            // else tracks_last_move_cm() may change when it shouldn't
            trks_move.exp_cm = centimeters;
            trks_move.last_run_ec = 0;
            pthread_mutex_lock(&trks_op_lock);
            trks_op = &trks_move;
            pthread_mutex_unlock(&trks_op_lock);
            rc = 0;
            LOG_DBG("move,%d,%u,%u,%u,%u", trks_op->speed_ecps, trks_op->start_ec, trks_op->exp_end_ec,
                        (trks_op->exp_end_ec - trks_op->slow_delta_ec),
                        (trks_op->exp_end_ec - trks_op->stop_delta_ec));
        }
    }

    pthread_mutex_unlock(&trks_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief tracks_pivot - pivot to the CW or CCW a given number of degrees.
 *
 * @details A routine used to pivot the track mechanism clockwise (positive
 * values) or counter-clock-wise (negative values) a given number of degrees.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int tracks_pivot(int16_t degrees)
{
    // Note:
    // CW = left forward (+) and right reverse (-)
    // CCW = left reverse (-) and right forward (+)
    // So, the sign of degrees is the same sign as the left track

    int rc = -1;

    pthread_mutex_lock(&trks_api_lock);

    if (!trks_initialized) {
        LOG_WRN("Not init'd");
    } else if ((degrees > -1 && degrees < 1) || (degrees > DT_PIVOT_DEGREES_MAX) || (degrees < DT_PIVOT_DEGREES_MIN)) {
        LOG_WRN("Invalid param");
    } else if (trks_op != NULL) {
        LOG_WRN("Operation in progress");
    } else {
        trks_pivot.exp_deg = degrees; // Mark as a pivot operation
        trks_pivot.slow_deg = determine_pivot_slow_degrees(degrees);
        imu_al_reset_gyro_int(trks_imu_hndl); // Force the gyro integrator to be reset
        int16_t cm = determine_pivot_distance(degrees);
        trks_pivot.speed_ecps = determine_move_speed(cm, false);
        trks_pivot.start_ec = get_tracks_ec();
        trks_pivot.exp_end_ec = get_exp_move_end_ec(trks_pivot.start_ec, cm);
        trks_pivot.slow_delta_ec = get_slow_delta_ec(trks_pivot.speed_ecps);
        trks_pivot.stop_delta_ec = TRKS_SLOW_SPEED_TO_STOP_EC;
        trks_start_left_ec = trks_last_left_ec;
        trks_start_right_ec = trks_last_right_ec;
        if (drivetrain_set_motion(trks_pivot.speed_ecps, -trks_pivot.speed_ecps) != 0) {
            LOG_ERR("Pivot failed");
        } else {
            // Cannot set exp_cm or move_ec until we know we are going to be moving
            // else tracks_last_move_cm() may change when it shouldn't
            trks_pivot.exp_cm = cm;
            trks_pivot.last_run_deg = 0;
            pthread_mutex_lock(&trks_op_lock);
            trks_op = &trks_pivot;
            pthread_mutex_unlock(&trks_op_lock);
            rc = 0;
            LOG_DBG("pivot,%d,%u,%u,%u,%u", trks_op->speed_ecps, trks_op->start_ec, trks_op->exp_end_ec,
                        (trks_op->exp_end_ec - trks_op->slow_delta_ec),
                        (trks_op->exp_end_ec - trks_op->stop_delta_ec));
        }
    }

    pthread_mutex_unlock(&trks_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief tracks_last_move_cm - retrieve estimated distance of the last move.
 *
 * @details A routine used to retrieve the estimated distance moved (in
 * centimeters) during the last move operation.
 *
 * @return Returns the estimated distance traveled during the last move.
 ******************************************************************************/

int16_t tracks_last_move_cm(void)
{
    pthread_mutex_lock(&trks_api_lock);

    int16_t delta_cm = (int16_t)(trks_move.last_run_ec / TRKS_ENCODER_CNTS_PER_CM);
    if (trks_move.exp_cm > 0) {
        delta_cm = -delta_cm;
    }

    pthread_mutex_unlock(&trks_api_lock);

    return delta_cm;
}

/*******************************************************************************
 * @brief tracks_last_pivot_deg - retrieve estimated degrees of the last pivot.
 *
 * @details A routine used to retrieve the estimated degrees moved
 * during the last pivot operation.
 *
 * @return Returns the estimated pivot degrees during the last pivot.
 ******************************************************************************/

int16_t tracks_last_pivot_deg(void)
{
    pthread_mutex_lock(&trks_api_lock);

/*
    int16_t delta_cm = (int16_t)(trks_pivot.last_run_ec / TRKS_ENCODER_CNTS_PER_CM);
    // Apply the pivot factor for cm calculations
    delta_cm = (int16_t)(delta_cm / TRKS_PIVOT_FACTOR);
    float radians = ((float)delta_cm) / (TRKS_CM_BETWEEN_TRACKS / 2.0f);
    int16_t deg = (int16_t)(((radians * 180.0f) / PI) + 0.5f);
    if (trks_pivot.exp_deg > 0) {
        deg = -deg;
    }
*/

    int16_t deg = trks_pivot.last_run_deg;

    pthread_mutex_unlock(&trks_api_lock);

    return deg;
}

/*******************************************************************************
 * @brief tracks_is_stopped - checks if the tracks module is stopped or not.
 *
 * @details A routine used to check if the tracks module is currently stopped
 * (true) or is performing an operation (false).
 *
 * @return Returns a flag denoting if the tracks module is stopped or not.
 ******************************************************************************/

bool tracks_is_stopped(void)
{
    return (trks_op == NULL);
}

/*******************************************************************************
 * @brief tracks_error_state - retrieve the current error state.
 *
 * @details A routine used to retrieve the modules current error state.
 *
 * @return Returns the current module error state.
 ******************************************************************************/

trks_errors_t tracks_error_state(void)
{
    return (trks_system_error);
}

/*****************************************************************************/

static int16_t calc_ecps(int8_t speed_percent)
{
    return ((trks_max_ecps * ((int16_t)speed_percent)) / 100);
}

static void callback_work_handler(struct k_work *work)
{
    if (trks_callback_func != NULL) {
        trks_callback_event.moving = (trks_op != NULL);
        trks_callback_event.syserr = trks_system_error;
        trks_callback_func(&trks_callback_event);
    }
}

static int16_t determine_move_speed(int16_t cm, bool move_not_pivot)
{
    static const int8_t FULL_MOVE_SPEED_PERCENTAGE = 80;
    static const int8_t FULL_PIVOT_SPEED_PERCENTAGE = 50;
    static const int8_t MIN_SPEED_PERCENTAGE = TRKS_SLOW_SPEED_PERCENTAGE + 5;
    static const int16_t FULL_MOVE_SPEED_MIN_CM = 20;
    static const int16_t FULL_PIVOT_SPEED_MIN_CM = 10;

    int8_t percent = (move_not_pivot) ? FULL_MOVE_SPEED_PERCENTAGE : FULL_PIVOT_SPEED_PERCENTAGE;
    int16_t full_spd_cm = (move_not_pivot) ? FULL_MOVE_SPEED_MIN_CM : FULL_PIVOT_SPEED_MIN_CM;
    int16_t abs_cm = abs(cm);

    if (abs_cm < full_spd_cm) {
        percent = (int8_t)((percent * (((float)abs_cm) / full_spd_cm)) + 0.5f);
        if (percent < MIN_SPEED_PERCENTAGE) {
            percent = MIN_SPEED_PERCENTAGE;
        }
    }

    return calc_ecps((cm >= 0) ? percent : -percent);
}

static int16_t determine_pivot_distance(int16_t degrees)
{
    // The circumference of a circle is: (2*R*PI) or (D*PI)

    float circumference = TRKS_CM_BETWEEN_TRACKS * PI;
    int16_t cm = (int16_t)((circumference * abs(degrees)) / 360.0f);

    // Apply the pivot factor
    cm = cm * TRKS_PIVOT_FACTOR;

    return (degrees > 0) ? cm : -cm;
}

static int16_t determine_pivot_slow_degrees(int16_t degrees)
{
    int16_t deg = 0;

    if (degrees > TRKS_PIVOT_SLOW_SPD_DEG_OFFSET + 1) {
        deg = degrees - TRKS_PIVOT_SLOW_SPD_DEG_OFFSET;
    } else if (degrees < (-(TRKS_PIVOT_SLOW_SPD_DEG_OFFSET + 1))) {
        deg = degrees + TRKS_PIVOT_SLOW_SPD_DEG_OFFSET;
    } else {
        // If the pivot is small, just slowdown for half of the pivot degrees
        deg = degrees / 2;
    }

    return deg;
}

static void drivetrain_cb(const drivetrain_status_t *status)
{
    static bool last_moving = false;
    static bool issued_slow = false;
    static bool issued_stop = false;

    if (status != NULL) {
        trks_last_left_ec = status->left_ec;
        trks_last_right_ec = status->right_ec;

        pthread_mutex_lock(&trks_op_lock);

        perform_safety_checks();

        if (trks_system_error != TRKS_ERR_NONE) {
            // For now, just stop all movement
            if (last_moving) {
                drivetrain_stop();
                // TODO: Blink an LED to alert of the issue
            }
        } else if (trks_op != NULL) {
            uint32_t curr_ec = get_tracks_ec();
            float pivoted_deg = get_pivoted_deg();
            // Update the current/last move encoder counts or pivot degrees
            trks_op->last_run_ec = trks_op->start_ec - curr_ec;
            trks_op->last_run_deg = pivoted_deg;
            // Determine the enc cnts to the end
            uint32_t delta_to_end = trks_op->exp_end_ec - curr_ec;
            if (delta_to_end > 0xF0000000) {
                delta_to_end = 0; // Handle roll-over case
            }
            // See if we should move to the stopping speed
            if ((!issued_slow) && (should_slow_move(delta_to_end, pivoted_deg))) {
                k_work_submit(&trks_slow_worker);
                issued_slow = true;
            }
            // See if we should be ending the movement
            if ((issued_slow) && (!issued_stop) && (should_stop_move(delta_to_end, pivoted_deg))) {
                k_work_submit(&trks_stop_worker);
                issued_stop = true;
            }
        }

        if (status->moving != last_moving) {
            last_moving = status->moving;
            if (!last_moving) {
                trks_op = NULL;
                issued_slow = false;
                issued_stop = false;
            }
            k_work_submit(&trks_callback_worker);
        }

        pthread_mutex_unlock(&trks_op_lock);
    }
}

static uint32_t get_exp_move_end_ec(uint32_t start_ec, int16_t cm)
{
    return (start_ec + ((uint32_t)(((float)abs(cm)) * TRKS_ENCODER_CNTS_PER_CM)));
}

static float get_pivoted_deg(void)
{
    float deg = 0.0f;

    if (trks_op->exp_deg != 0) {
        imu_gyro_int_t gyro_int;
        imu_gyro_t gyro;
        if (imu_al_gyro_read(trks_imu_hndl, &gyro, &gyro_int) != 0) {
            LOG_WRN("Gyro read failed");
            // Just continue and use distance to stop
        } else {
            deg = gyro_int.z_deg;
        }
    }

    return deg;
}

static uint32_t get_slow_delta_ec(int16_t ecps)
{
    // Just using rough math here, no floats since there is enough
    // variability in the whole process a round error will not matter.

    int8_t speed_percentage;
    
    if (ecps > 0) {
        speed_percentage = (ecps * 100) / trks_max_ecps;
    } else {
        speed_percentage = (ecps * 100) / trks_min_ecps;
    }

    // Estimate the enc cnts needed to stop
    // At fastest speed it takes 400ms a second to fully stop
    uint32_t delta = (((abs(ecps) * (speed_percentage - TRKS_SLOW_SPEED_PERCENTAGE)) / 100) * 4) / 10;
    // Add 1/10 second worth of movemement at the slow speed
    delta += (trks_max_ecps * TRKS_SLOW_SPEED_PERCENTAGE) / 100 / 10;
    // Add the stopping counts
    delta += TRKS_SLOW_SPEED_TO_STOP_EC;

    return (delta);
}

static uint32_t get_tracks_ec(void)
{
    // Considered using the average encoder count value between the left and
    // right values, but since roll-over is possible, it was thought that
    // using just one side would be more consistent.  Arbitrarily chose left.
    return trks_last_left_ec;
}

static int init_imu(void)
{
    int rc = -1;

    imu_al_hw_t hw;
    hw.imu_dev = ism330dhcx;

    trks_imu_hndl = imu_al_init(&hw);
    if (trks_imu_hndl < 0) {
        LOG_ERR("IMU init failed");
    } else {
        rc = 0;
    }

    return rc;
}

static void perform_safety_checks(void)
{
    // The caller must ensure that the trks_op_lock is locked

    if (trks_op == NULL) {
        // No operations in progress, so just return
    } else if (trks_system_error != TRKS_ERR_NONE) {
        // Already in an error state
    } else {
        // At this time the right and left always move together, so verify that
        // the delta movements are close to the same.
        uint32_t delta_left = trks_last_left_ec - trks_start_left_ec;
        uint32_t delta_right = trks_last_right_ec - trks_start_right_ec;
        uint32_t delta_left_to_right = abs(delta_left - delta_right);
        // Max delta left-to-right is TRKS_SAFETY_DELTA_PERCENTAGE of total distance
        uint32_t max_delta = (trks_op->exp_cm * TRKS_ENCODER_CNTS_PER_CM * TRKS_SAFETY_DELTA_PERCENTAGE) / 100;
        // Force a minimum max delta to prevent errors on short distances
        if (max_delta < TRKS_SAFETY_DELTA_MIN_EC) {
            max_delta = TRKS_SAFETY_DELTA_MIN_EC;
        }

        if (delta_left_to_right > max_delta) {
            trks_system_error = TRKS_ERR_MOVEMENT_LOST_SYNC;
            LOG_ERR("SYSTEM ERROR: Lost sync between tracks (%u/%u)", delta_left_to_right, max_delta);
        }
    }
}

static bool should_slow_move(uint32_t delta_to_end, float pivoted_deg)
{
    bool slow = false;

    if ((trks_op->exp_deg != 0) && (pivoted_deg != 0.0f)) {
        int16_t slow_deg = trks_op->slow_deg;
        if (slow_deg < 0) {
            if (pivoted_deg < ((float)slow_deg)) {
                slow = true;
            }
        } else {
            if (pivoted_deg > ((float)slow_deg)) {
                slow = true;
            }
        }
    } else if ((delta_to_end <= trks_op->slow_delta_ec)) {
        slow = true;
    }

    return slow;
}

static bool should_stop_move(uint32_t delta_to_end, float pivoted_deg)
{
    int16_t exp_deg = trks_op->exp_deg;
    bool stop = false;

    if ((exp_deg != 0) && (pivoted_deg != 0.0f)) {
        if (exp_deg < 0) {
            if (pivoted_deg < (((float)exp_deg) + 0.2f)) {
                stop = true;
            }
        } else {
            if (pivoted_deg > (((float)exp_deg) - 0.2f)) {
                stop = true;
            }
        }
    } else if ((delta_to_end <= trks_op->stop_delta_ec)) {
        stop = true;
    }

    return stop;
}

static void slow_work_handler(struct k_work *work)
{
    int16_t last_l = 0;
    int16_t last_r = 0;
    int16_t spd = calc_ecps(TRKS_SLOW_SPEED_PERCENTAGE);

    drivetrain_get_motion(&last_l, &last_r);
    int ret = drivetrain_set_motion(((last_l >= 0) ? spd : -spd), ((last_r >= 0) ? spd : -spd));
    LOG_DBG("Issued slow %u", get_tracks_ec());
    if (ret != 0) {
        LOG_WRN("Slow speed failed");
    }
}

static void stop_work_handler(struct k_work *work)
{
    drivetrain_stop();
    LOG_DBG("Issued stop %u", get_tracks_ec());
}
