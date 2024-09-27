/*****************************************************************************
 * drivetrain.c - source file for a dual independent drivetrain (think
 * tank tracks)
 *
 * For each motor, the motor speed must be stopped before changing directions.
 *****************************************************************************/

#include "drivetrain.h"

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>  // for usleep()
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include "dcme.h"
#include "logger.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

// Motor parameters
#define DT_MOTOR_REVS_PER_SHAFT_REV  (98)
#define DT_ENC_CNTS_PER_MOTOR_REV    (12)
#define DT_ENC_CNTS_PER_SHAFT_REV    (DT_MOTOR_REVS_PER_SHAFT_REV * DT_ENC_CNTS_PER_MOTOR_REV)

// The rate at which we call refresh on the motors
#define DT_MOTOR_REFRESH_RATE_MS (20)

// The "1.5" is an adjustment based on testing for the maximum reachable speed
static const uint32_t DT_MAX_ENC_CNTS_PER_SEC  = (1.5f * DT_ENC_CNTS_PER_SHAFT_REV);
static const float DT_MAX_ENC_CNTS_PER_LOOP = (((float)DT_MAX_ENC_CNTS_PER_SEC) / (1000.0f / DT_MOTOR_REFRESH_RATE_MS));

// Number of refresh loops for a 100ms delay
#define DT_DELAY_COUNTS (100 / DT_MOTOR_REFRESH_RATE_MS)

static const long DT_NSEC_PER_SEC = 1000000000;
static const long DT_NSEC_PER_USEC = 1000;
static const long DT_USEC_PER_SEC = 1000000;
static const long DT_USEC_PER_MSEC = 1000;

// Left motor resources
static const struct pwm_dt_spec dt_mtr_pwm_left = PWM_DT_SPEC_GET_OR(DT_NODELABEL(mtr_spd_left), {});
static const struct gpio_dt_spec dt_mtr_enc_left = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(mtr_enc_left), gpios, {0});
static struct gpio_dt_spec dt_mtr_fwd_left = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(mtr_fwd_left), gpios, {0});
static struct gpio_dt_spec dt_mtr_rev_left = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(mtr_rev_left), gpios, {0});
// Right motor resources
static const struct pwm_dt_spec dt_mtr_pwm_right = PWM_DT_SPEC_GET_OR(DT_NODELABEL(mtr_spd_right), {});
static const struct gpio_dt_spec dt_mtr_enc_right = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(mtr_enc_right), gpios, {0});
static struct gpio_dt_spec dt_mtr_fwd_right = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(mtr_fwd_right), gpios, {0});
static struct gpio_dt_spec dt_mtr_rev_right = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(mtr_rev_right), gpios, {0});

//! This mutex is used to serialize public function calls
static pthread_mutex_t dt_api_lock = PTHREAD_MUTEX_INITIALIZER;
//! The application thread
static pthread_t dt_thread = {0};

#define DT_THREAD_STACK_SIZE (2000)
static K_THREAD_STACK_DEFINE(dt_thread_stack, DT_THREAD_STACK_SIZE);

//! Flag used to force the application thread to exit
static bool dt_thread_exit = false;
//! Flag used to know if the module has been initialized or not
static bool dt_initialized = false;

// When the mode changes, we always go to OFF/OFF first
enum
{ MODE_NONE = 0 // Nothing to do state for the queue
, MODE_DELAY // Force a 100ms delay
, MODE_RATE_CHG // Leave mode as is but change left/right percentages
, MODE_L_OFF_R_OFF
, MODE_L_FWD_R_FWD
, MODE_L_REV_R_REV
, MODE_L_FWD_R_REV
, MODE_L_REV_R_FWD
, MODE_L_FWD_R_OFF
, MODE_L_OFF_R_FWD
, MODE_L_REV_R_OFF
, MODE_L_OFF_R_REV
};

typedef struct
{
  int8_t mode;
  int16_t left;
  int16_t right;
} dt_q_item_t;

// A difference between sModeCurr and sModeNext means we are ramping from Curr to Next.
static dt_q_item_t dt_mode_curr = { MODE_L_OFF_R_OFF, 0, 0 };
static dt_q_item_t dt_mode_next = { MODE_L_OFF_R_OFF, 0, 0 };
// This queue holds the sequence of things to do after the current ramp is complete
#define DT_MODE_QUEUE_SIZE (4)
static dt_q_item_t dt_mode_queue[DT_MODE_QUEUE_SIZE] = {0};

// The left and right motor handles
static int dt_hndl_left = 0;
static int dt_hndl_right = 0;

// The motion input values in terms of encoder-counts-per-second
static int16_t dt_motion_left = 0;
static int16_t dt_motion_right = 0;

// A delay will only occur while dt_mode_curr and dt_mode_next are OFF/OFF
static uint8_t dt_delay_cnt = 0;

// The registered callback function for the drivetrain API
static drivetrain_cb_t dt_callback_func = NULL;
static drivetrain_status_t dt_callback_status;

static void delay_from_ref(const struct timespec *ref_ts, uint16_t delay_ms);
// Return not used
static void *dt_thread_loop(void *arg);
// Returns 0 on success, otherwise -1
static int dt_thread_setup_and_start(void);
static void dt_thread_wake_and_join(void);
static void handle_mode_change(int mode);
static void handle_mode_or_rate_change(void);
static void handle_mode_queue(void);
static void handle_next_queue_item(void);
static void handle_rate_change(void);
static void q_item_copy(dt_q_item_t *to, const dt_q_item_t *from);
static void q_item_set(dt_q_item_t *item, int8_t mode, int16_t left, int16_t right);
// Returns a rounded value from a current range (0 to curr_max) to a new range (0 to new_max)
static float rounded_map(float input, float curr_max, float new_max);
// Returns 0 on success, otherwise -1
static int set_motion_direction_and_speed(int8_t mode, int16_t left, int16_t right);
// Returns 0 on success, otherwise -1
static int set_motion_speed(int16_t left, int16_t right);

/*******************************************************************************
 * @brief drivetrain_init - initialize the drivetrain for use.
 *
 * @details A routine used to initialize the drivetrain for use.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int drivetrain_init(void)
{
    int rc = -1;

    pthread_mutex_lock(&dt_api_lock);

    if (dt_initialized) {
        LOG_WRN("Already init'd");
    } else {
        static dcme_hw_t hw_left = {&dt_mtr_pwm_left, &dt_mtr_enc_left, &dt_mtr_fwd_left, &dt_mtr_rev_left, false};
        static dcme_hw_t hw_right = {&dt_mtr_pwm_right, &dt_mtr_enc_right, &dt_mtr_fwd_right, &dt_mtr_rev_right, false};

        dt_hndl_left = dcme_init(&hw_left, DT_MOTOR_REFRESH_RATE_MS);
        dt_hndl_right = dcme_init(&hw_right, DT_MOTOR_REFRESH_RATE_MS);
        if ((dt_hndl_left < 0) || (dt_hndl_right < 0)) {
            LOG_WRN("DC motor init failed");
            dcme_exit(dt_hndl_left);
            dcme_exit(dt_hndl_right);
        } else if (dt_thread_setup_and_start() != 0) {
            LOG_WRN("Thread start failed");
            dcme_exit(dt_hndl_left);
            dcme_exit(dt_hndl_right);
        } else {
            // Motion params
            q_item_set(&dt_mode_curr, MODE_L_OFF_R_OFF, 0, 0);
            q_item_set(&dt_mode_next, MODE_L_OFF_R_OFF, 0, 0);
            dt_delay_cnt = 0;
            dt_motion_left = 0;
            dt_motion_right = 0;
            dt_callback_func = NULL;
            dt_initialized = true;
            rc = 0;
        }
    }

    pthread_mutex_unlock(&dt_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief drivetrain_exit - de-initialize the drivetrain.
 *
 * @details A routine used to de-initialize the drivetrain.
 *
 * @return None.
 ******************************************************************************/

void drivetrain_exit(void)
{
    pthread_mutex_lock(&dt_api_lock);

    if (dt_initialized) {
        // TODO: Should probably ensure everything is stopped first
        dt_thread_wake_and_join();
        dcme_exit(dt_hndl_left);
        dcme_exit(dt_hndl_right);
        dt_initialized = false;
    }

    pthread_mutex_unlock(&dt_api_lock);
}

/*******************************************************************************
 * @brief drivetrain_reg_cb - register a callback function for periodic cb's.
 *
 * @details A routine used to register for periodic callbacks during regular
 * drivetrain operations.  These callbacks should be handled like ISRs in that
 * the processing done within them must be short (non-blocking) and should not
 * call any of the drivertrain APIs.
 *
 * @param cb - the callback function to register.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int drivetrain_reg_cb(drivetrain_cb_t cb)
{
    int rc = -1;

    pthread_mutex_lock(&dt_api_lock);

    if (!dt_initialized) {
        LOG_WRN("Not init'd");
    } else if (dt_callback_func != NULL) {
        LOG_WRN("Callback already registered");
    } else {
        dt_callback_func = cb;
        rc = 0;
    }

    pthread_mutex_unlock(&dt_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief drivertrain_dereg_cb - de-register a previously registered callback.
 *
 * @details A routine used to de-register a drivetrain callback function.
 *
 * @return None.
 ******************************************************************************/

void drivetrain_dereg_cb(drivetrain_cb_t cb)
{
    pthread_mutex_lock(&dt_api_lock);

    if (!dt_initialized) {
        LOG_WRN("Not init'd");
    } else if (dt_callback_func != cb) {
        LOG_WRN("Callback not registered");
    } else {
        dt_callback_func = NULL;
    }

    pthread_mutex_unlock(&dt_api_lock);
}

/*******************************************************************************
 * @brief drivetrain_get_motion_limits - retrieve the motion limits.
 *
 * @details A routine used to retrieve the limits on the inputs to the
 * drivetrain_motion() routine.
 *
 * @return None.
 ******************************************************************************/

void drivetrain_get_motion_limits(int16_t *min_ecps, int16_t *max_ecps)
{
    if (min_ecps != NULL) {
        *min_ecps = -((int16_t)DT_MAX_ENC_CNTS_PER_SEC);
    }
    if (max_ecps != NULL) {
        *max_ecps = ((int16_t)DT_MAX_ENC_CNTS_PER_SEC);
    }
}

/*******************************************************************************
 * @brief drivetrain_get_motion - retrieve the drivetrain motion parameters.
 *
 * @details A routine used to retrieve the drivetrain motion parameters.
 *
 * @param left_ecps - the pointer to retrieve the left motion setting.
 * @param right_ecps - the pointer to retireve the right motion setting.
 *
 * @return None.
 ******************************************************************************/

void drivetrain_get_motion(int16_t *left_ecps, int16_t *right_ecps)
{
    int16_t left = 0;
    int16_t right = 0;

    pthread_mutex_lock(&dt_api_lock);

    if (!dt_initialized) {
        LOG_WRN("Not init'd");
    } else {
        left = dt_motion_left;
        right = dt_motion_right;
    }

    if (left_ecps != NULL) {
        *left_ecps = left;
    }
    if (right_ecps != NULL) {
        *right_ecps = right;
    }

    pthread_mutex_unlock(&dt_api_lock);
}

/*******************************************************************************
 * @brief drivetrain_set_motion - set the motion parameters for drivetrain.
 *
 * @details A routine used to set the drivetrain motion parameters.  This is
 * effectively the gas pedal(s) for the drivetrain.
 *
 * @param left_ecps - the left motion setting (enc cnts-per-sec).
 * @param right_ecps - the right motion setting (enc-cnt-per-sec).
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int drivetrain_set_motion(int16_t left_ecps, int16_t right_ecps)
{
    int rc = -1;

    pthread_mutex_lock(&dt_api_lock);

    if (!dt_initialized) {
        LOG_WRN("Not init'd");
    } else if ( (left_ecps < -((int16_t)DT_MAX_ENC_CNTS_PER_SEC))
                || (left_ecps > ((int16_t)DT_MAX_ENC_CNTS_PER_SEC))
                || (right_ecps < -((int16_t)DT_MAX_ENC_CNTS_PER_SEC))
                || (right_ecps > ((int16_t)DT_MAX_ENC_CNTS_PER_SEC))) {
        LOG_WRN("Invalid param");
    } else {
        dt_motion_left = left_ecps;
        dt_motion_right = right_ecps;
        rc = 0;
    }

    pthread_mutex_unlock(&dt_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief drivetrain_stop - stops all motion of the drive train.
 *
 * @details A routine used to halts the motion of both motors of the drivetrain.
 *
 * @return None.
 ******************************************************************************/

void drivetrain_stop(void)
{
    pthread_mutex_lock(&dt_api_lock);

    if (!dt_initialized) {
        LOG_WRN("Not init'd");
    } else {
        dt_motion_left = 0;
        dt_motion_right = 0;
    }

    pthread_mutex_unlock(&dt_api_lock);
}

/*******************************************************************************
 * @brief drivetrain_get_enc_cnts - retrieve the current motor encoder counts.
 *
 * @details A routine used to retrieve the current motor encoder counts.
 *
 * @return Returns current left/right encoder count values.
 ******************************************************************************/

void drivetrain_get_enc_cnts(uint32_t *left_ec, uint32_t *right_ec)
{
    // Not locking the API mutex as this may be used a lot and locking is not
    // super important here.

    if (!dt_initialized) {
        LOG_WRN("Not init'd");
    } else {
        if (left_ec != NULL) {
            *left_ec = dcme_get_enc_cnt(dt_hndl_left);
        }
        if (right_ec != NULL) {
            *right_ec = dcme_get_enc_cnt(dt_hndl_right);
        }
    }
}

/*******************************************************************************
 * @brief drivetrain_get_dcme_handle - retrieves a DCME handle.
 *
 * @details A routine used to retrieve the left or right DC motor (DCME) handle.
 * This routine is for debug purposes only.
 *
 * @return Returns a DCME handle on success, otherwise -1.
 ******************************************************************************/

int drivetrain_get_dcme_handle(bool left)
{
    int hndl = -1;

    if (!dt_initialized) {
        LOG_WRN("Not init'd");
    } else {
        hndl = (left) ? dt_hndl_left : dt_hndl_right;
    }

    return hndl;
}

/*****************************************************************************/

static void delay_from_ref(const struct timespec *ref_ts, uint16_t delay_ms)
{
    struct timespec now;

    // Get the current time
    clock_gettime(CLOCK_MONOTONIC, &now);

    // This math works as long seconds have not rolled-over twice
    uint32_t delta_usec;
    if (now.tv_nsec >= ref_ts->tv_nsec) {
        delta_usec = (now.tv_nsec - ref_ts->tv_nsec) / DT_NSEC_PER_USEC;
    } else {
        delta_usec = ((DT_NSEC_PER_SEC - ref_ts->tv_nsec) + now.tv_nsec) / DT_NSEC_PER_USEC;
    }

    uint32_t delay_usec = delay_ms * DT_USEC_PER_MSEC;

    if (delta_usec > delay_usec) {
        // The delay has already been reached, so no additional delay needed
    } else {
        uint32_t usec_to_delay = delay_usec - delta_usec;
        if (usec_to_delay < DT_USEC_PER_SEC) {
            usleep(usec_to_delay);
        } else {
            uint32_t sec_to_delay = usec_to_delay / DT_USEC_PER_SEC;
            usec_to_delay = usec_to_delay % DT_USEC_PER_SEC;
            sleep(sec_to_delay);
            usleep(usec_to_delay);
        }
    }
}

static int determine_mode(void)
{
    int mode = MODE_L_OFF_R_OFF;

    if (dt_motion_left == 0 || dt_motion_right == 0) {
        if (dt_motion_left > 0) {
            mode = MODE_L_FWD_R_OFF;
        } else if (dt_motion_right > 0) {
            mode = MODE_L_OFF_R_FWD;
        } else if (dt_motion_left < 0) {
            mode = MODE_L_REV_R_OFF;
        } else if (dt_motion_right < 0) {
            mode = MODE_L_OFF_R_REV;
        } else {
            mode = MODE_L_OFF_R_OFF;
        }
    } else if (dt_motion_left > 0 && dt_motion_right > 0) {
        mode = MODE_L_FWD_R_FWD;
    } else if (dt_motion_left < 0 && dt_motion_right < 0) {
        mode = MODE_L_REV_R_REV;
    } else if (dt_motion_left > 0 && dt_motion_right < 0) {
        mode = MODE_L_FWD_R_REV;
    } else if (dt_motion_left < 0 && dt_motion_right > 0) {
        mode = MODE_L_REV_R_FWD;
    }

    return mode;
}

static void *dt_thread_loop(void *arg)
{
    (void)arg;  // Not used

    struct timespec reference_ts = {0};

    dt_thread_exit = false;

    // Wait for the module to be fully initialized
    while (!dt_initialized && !dt_thread_exit) {
        usleep(100000); // 100ms
    }

    // Get the reference time
    clock_gettime(CLOCK_MONOTONIC, &reference_ts);

    // Loop until requested to exit
    while (!dt_thread_exit) {

        // Perform a motor refresh
        handle_mode_or_rate_change();
        handle_mode_queue();
        dcme_refresh(dt_hndl_left);
        dcme_refresh(dt_hndl_right);

        // Support a periodic callback API
        if (dt_callback_func != NULL) {
            dt_callback_status.moving = (dt_mode_curr.mode != MODE_L_OFF_R_OFF);
            dt_callback_status.left_ecps = dt_mode_curr.left;
            dt_callback_status.right_ecps = dt_mode_curr.right;
            dt_callback_status.left_ec = dcme_get_enc_cnt(dt_hndl_left);
            dt_callback_status.right_ec = dcme_get_enc_cnt(dt_hndl_right);
            dt_callback_func(&dt_callback_status);
        }

        // Delay for a refresh period from the reference time
        delay_from_ref(&reference_ts, DT_MOTOR_REFRESH_RATE_MS);
        // Update the reference time
        clock_gettime(CLOCK_MONOTONIC, &reference_ts);
    }

    return NULL;
}

static int dt_thread_setup_and_start(void)
{
    static pthread_attr_t thread_attr = {0};

    int ret = 0;
    int rc = -1;

    if ((ret = pthread_attr_init(&thread_attr)) != 0) {
        LOG_WRN("Failed attr init: %d", ret);
    } else if ((ret = pthread_attr_setstack(&thread_attr, dt_thread_stack, DT_THREAD_STACK_SIZE)) != 0) {
        LOG_WRN("Failed attr stack: %d", ret);
    } else if ((ret = pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_JOINABLE)) != 0) {
        LOG_WRN("Failed attr joinable: %d", ret);
    } else if ((ret = pthread_create(&dt_thread, &thread_attr, dt_thread_loop, NULL)) != 0) {
        LOG_WRN("Failed to start: %d", ret);
    } else {
        // For some reason MacOS does not correctly support this interface
        (void)pthread_setname_np(dt_thread, "drivetrain");
        rc = 0;
    }

    return rc;
}

static void dt_thread_wake_and_join(void)
{
    // Only attempt to join if the thread is running (!exit)
    if (dt_thread_exit == false) {
        // Set the exit flag for the thread
        dt_thread_exit = true;
        // Join the thread
        int ret = pthread_join(dt_thread, NULL);
        if (ret != 0) {
            LOG_INF("Join failed %d", ret);
        }
    }
}

static void handle_mode_change(int mode)
{
    // See if we are currently OFF/OFF without an active delay
    if (  (dt_mode_curr.mode == dt_mode_next.mode)
        && (dt_mode_curr.mode == MODE_L_OFF_R_OFF)
        && (dt_delay_cnt == 0)
        )
    {
        // Add the mode to the queue
        q_item_set(&dt_mode_queue[0], mode, dt_motion_left, dt_motion_right);
        for (int i = 1; i < DT_MODE_QUEUE_SIZE; i++) {
            q_item_set(&dt_mode_queue[i], MODE_NONE, 0, 0);
        }
        // Execute the queue
        handle_next_queue_item();
    }
    // We must go to the OFF/OFF mode before moving the to new mode
    else {
        if (dt_mode_next.mode != MODE_L_OFF_R_OFF)
        {
            // Set the OFF/OFF mode
            q_item_set(&dt_mode_next, MODE_L_OFF_R_OFF, 0, 0);
            // Start transition to off
            if (dcme_set_speed(dt_hndl_left, 0) != 0) {
                LOG_WRN("Set speed failed");
            }
            if (dcme_set_speed(dt_hndl_right, 0) != 0) {
                LOG_WRN("Set speed failed");
            }
        }
        // Setup the queue appropriately
        q_item_set(&dt_mode_queue[0], MODE_DELAY, 0, 0); // Always delay after reaching OFF/OFF
        q_item_set(&dt_mode_queue[1], mode, dt_motion_left, dt_motion_right);
        for (int i = 2; i < DT_MODE_QUEUE_SIZE; i++)
        {
            q_item_set(&dt_mode_queue[i], MODE_NONE, 0, 0);
        }
    }
}

static void handle_mode_or_rate_change(void)
{
    static int lastMode = MODE_L_OFF_R_OFF;
    static int8_t lastLeft = 0;
    static int8_t lastRight = 0;

    int mode = determine_mode();

    if (mode != lastMode) {
        // Mode Change
        lastMode = mode;
        lastLeft = dt_motion_left;
        lastRight = dt_motion_right;
        handle_mode_change(mode);
    } else if ((lastLeft != dt_motion_left) || (lastRight != dt_motion_right)) {
        // Rate Change
        lastLeft = dt_motion_left;
        lastRight = dt_motion_right;
        handle_rate_change();
    }
}

static void handle_mode_queue(void)
{
    if (dt_delay_cnt > 0) {
        dt_delay_cnt--;
        if (dt_delay_cnt == 0) {
            // We've completed the delay
            handle_next_queue_item();
        }
    } else if ((dt_mode_curr.mode != MODE_L_OFF_R_OFF) && (dt_mode_next.mode == MODE_L_OFF_R_OFF)) {
        // We wait for the motors to stop
        if ( (dcme_get_state(dt_hndl_left) == DCME_STATE_OFF)
           && (dcme_get_state(dt_hndl_right) == DCME_STATE_OFF)
           )
        {
            q_item_copy(&dt_mode_curr, &dt_mode_next);
            handle_next_queue_item();
        } else {
            // Just keep setting the speed to zero, no harm in doing this
            if (dcme_set_speed(dt_hndl_left, 0) != 0) {
                LOG_WRN("Set speed failed");
            }
            if (dcme_set_speed(dt_hndl_right, 0) != 0) {
                LOG_WRN("Set speed failed");
            }
        }
    } else {
        handle_next_queue_item();
    }
}

static void handle_next_queue_item(void)
{
    if (dt_mode_queue[0].mode != MODE_NONE)
    {
        dt_q_item_t item;
        q_item_copy(&item, &dt_mode_queue[0]);

        // Adjust the queue (pop)
        for (int i = 0; i < DT_MODE_QUEUE_SIZE; i++)
        {
            if ((i + 1) < DT_MODE_QUEUE_SIZE) {
                q_item_copy(&dt_mode_queue[i], &dt_mode_queue[i + 1]);
            } else {
                q_item_set(&dt_mode_queue[i], MODE_NONE, 0, 0);
            }
        }

        // Reset the delay count, althought it should already be zero
        dt_delay_cnt = 0;

        if (item.mode == MODE_DELAY)
        {
            dt_delay_cnt = DT_DELAY_COUNTS;
        }
        else if (item.mode == MODE_RATE_CHG)
        {
            // Keep the current mode but ramp the speed
            dt_mode_next.left = item.left;
            dt_mode_next.right = item.right;

            if (set_motion_speed(item.left, item.right) != 0) {
                LOG_WRN("Set speed failed");
            }
        }
        else
        {
            // Set the HW
            if (set_motion_direction_and_speed(item.mode, item.left, item.right) != 0) {
                LOG_WRN("Set direction and speed failed");
            }
            // Set the mode
            q_item_copy(&dt_mode_curr, &item);
            // TODO: Figure out why next gets set to curr in this case?
            q_item_copy(&dt_mode_next, &dt_mode_curr);
        }
    }
}

static void handle_rate_change(void)
{
    // We only want one MODE_RATE_CHG at the end of the queue
    int empty = -1;
    int idx = (DT_MODE_QUEUE_SIZE - 1);
    for (; idx >= 0; idx--) {
        if (dt_mode_queue[idx].mode == MODE_RATE_CHG) {
            // Found one so update the left/right percentages
            dt_mode_queue[idx].left = dt_motion_left;
            dt_mode_queue[idx].right = dt_motion_right;
            break;
        } else if (dt_mode_queue[idx].mode == MODE_NONE) {
            empty = idx;
        }
    }

    // If we didn't find an existing entry, add it to empty
    if (idx < 0 && empty != -1) {
        // Add a MODE_RATE_CHG entry at the empty slot
        q_item_set(&dt_mode_queue[empty], MODE_RATE_CHG, dt_motion_left, dt_motion_right);
    }
}

static void q_item_copy(dt_q_item_t *to, const dt_q_item_t *from)
{
    // TODO: Consider if a memcpy would be faster
    to->mode = from->mode;
    to->left = from->left;
    to->right = from->right;
}

static void q_item_set(dt_q_item_t *item, int8_t mode, int16_t left, int16_t right)
{
    item->mode = mode;
    item->left = left;
    item->right = right;
}

// This assumes zero indexed maxes
static float rounded_map(float input, float curr_max, float new_max)
{
    // Calc percentage of input to its max, then apply that percent to the new max, then round
    return ((input / curr_max) * new_max) + 0.5f;
}

static int set_motion_direction_and_speed(int8_t mode, int16_t left, int16_t right)
{
    int rc = 0;

    dcme_state_t left_state = DCME_STATE_OFF;
    dcme_state_t right_state = DCME_STATE_OFF;
    int16_t left_speed = left;
    int16_t right_speed = right;

    switch (mode)
    {
    case MODE_L_FWD_R_FWD:
        left_state = DCME_STATE_FWD;
        right_state = DCME_STATE_FWD;
        break;
    case MODE_L_REV_R_REV:
        left_state = DCME_STATE_REV;
        right_state = DCME_STATE_REV;
        break;
    case MODE_L_FWD_R_REV:
        left_state = DCME_STATE_FWD;
        right_state = DCME_STATE_REV;
        break;
    case MODE_L_REV_R_FWD:
        left_state = DCME_STATE_REV;
        right_state = DCME_STATE_FWD;
        break;
    case MODE_L_FWD_R_OFF:
        left_state = DCME_STATE_FWD;
        right_state = DCME_STATE_OFF;
        right_speed = 0;
        break;
    case MODE_L_OFF_R_FWD:
        left_state = DCME_STATE_OFF;
        right_state = DCME_STATE_FWD;
        left_speed = 0;
        break;
    case MODE_L_REV_R_OFF:
        left_state = DCME_STATE_REV;
        right_state = DCME_STATE_OFF;
        right_speed = 0;
        break;
    case MODE_L_OFF_R_REV:
        left_state = DCME_STATE_OFF;
        right_state = DCME_STATE_REV;
        left_speed = 0;
        break;
    case MODE_L_OFF_R_OFF:
        left_state = DCME_STATE_OFF;
        right_state = DCME_STATE_OFF;
        left_speed = 0;
        right_speed = 0;
        break;
    default:
        LOG_WRN("Invalid mode %d", mode);
        rc = -1;
        break;
    };

    if (rc == 0)
    {
        int lret = dcme_set_state(dt_hndl_left, left_state);
        int rret = dcme_set_state(dt_hndl_right, right_state);
        int mret = set_motion_speed(left_speed, right_speed);

        if (lret != 0) {
            LOG_WRN("Set state failed");
        } else if (rret != 0) {
            LOG_WRN("Set state failed");
        } else if (mret == 0) {
            rc = 0;
        }
    }

    return rc;
}

static int set_motion_speed(int16_t left, int16_t right)
{
    static int16_t last_left_spd = 0;
    static int16_t last_right_spd = 0;
    float left_spd = 0.0f;
    float right_spd = 0.0f;
    int rc = -1;

    if ((left == last_left_spd) || (right == last_right_spd)) {
        // Not change, so just return success
        rc = 0;
    } else {
        if (left >= 0) {
            left_spd = rounded_map(left, DT_MAX_ENC_CNTS_PER_SEC, DT_MAX_ENC_CNTS_PER_LOOP);
        } else {
            left_spd = rounded_map(-left, DT_MAX_ENC_CNTS_PER_SEC, DT_MAX_ENC_CNTS_PER_LOOP);
        }

        if (right >= 0) {
            right_spd = rounded_map(right, DT_MAX_ENC_CNTS_PER_SEC, DT_MAX_ENC_CNTS_PER_LOOP);
        } else {
            right_spd = rounded_map(-right, DT_MAX_ENC_CNTS_PER_SEC, DT_MAX_ENC_CNTS_PER_LOOP);
        }

        // Set the speed, but only if the state allows it (is not OFF)
        int ret_left = 0;
        int ret_right = 0;
        dcme_state_t left_state = dcme_get_state(dt_hndl_left);
        dcme_state_t right_state = dcme_get_state(dt_hndl_right);
        if (left_state != DCME_STATE_OFF) {
            ret_left = dcme_set_speed(dt_hndl_left, left_spd);
        }
        if (right_state != DCME_STATE_OFF) {
            ret_right = dcme_set_speed(dt_hndl_right, right_spd);
        }

        if (ret_left != 0) {
            LOG_WRN("Speed left set failed");
        }
        if (ret_right != 0) {
            LOG_WRN("Speed right set failed");
        }
        
        if ((ret_left == 0) && (ret_right == 0)) {
            last_left_spd = left;
            last_right_spd = right;
            rc = 0;
        }
    }

    return rc;
}
