/*****************************************************************************
 * dcme.c - source file for DC Motor with Encoder module
 *
 * This module is for a DC Motor with an attached quadrature encoder.
 *
 * This module will only use one of the encoder inputs since the direction
 * of movement should really already be known, thus we don't need both
 * inputs.  We will count of both edges of the encoder input thus doubling
 * the available cnts.
 *
 * This module will use a PID control loop.
 *
 * Control of the motor is via a direction input and an expected encoder
 * counts per refresh period.  A call to the "refresh()" function must be
 * made every periodically (every x ms).
 *
 * The motor speed must be allow to go to zero before changing directions.
 * In order to do this, set the expected encoder counts to 0 and wait for
 * the motor state to transition to OFF.
 *****************************************************************************/

#include "dcme.h"

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>  // for usleep()
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include "logger.h"
#include "os_al.h"
#include "pid.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

// Uncomment to enable capturing of motor movement debug details
#define DCME_DBG_CAPTURE_DETAILS
// Uncomment to only capture start of movement details
//#define DCME_DBG_CAPTURE_START_ONLY
// Uncomment to output debug logging
//#define DCME_DBG_LOGGING

//! This mutex is used to serialize public function calls
static pthread_mutex_t dcme_api_lock = PTHREAD_MUTEX_INITIALIZER;

// The number of samples (refreshes) with no movement before transitioning to
// the motor OFF state
#define DCME_SAMPLES_TO_OFF (10)
// The current speed IIR filter size
#define DCME_IIR_FILTER_ELEMENT_CNT (16)
// The maximum number of steps for a motor acceleration or decceleration ramp
#define DCME_MAX_RAMP_CNT (16)
// The minimum number of steps for a motor acceleration or decceleration ramp
#define DCME_MIN_RAMP_CNT (2)

// The PID gains for the motor.  These should probably live elsewhere.
static const float DCME_PID_GAIN_KP = 15.0f;
static const float DCME_PID_GAIN_KI = 10.0f;
static const float DCME_PID_GAIN_KD = 0.0f;

// Need a value to compare a float against to check for near zero
static const float DCME_FLOAT_ALMOST_ZERO = 0.000001f;

typedef struct
{
    bool initialized;
    dcme_hw_t hw;
    dcme_state_t state;
    struct gpio_callback enc_cb_data;
    float exp_enc_cnt_per_refresh;
    float iir_filtered_speed;
    bool iir_filter_reset;

    uint32_t enc_cnt;
    uint32_t enc_cnt_capture;
    uint32_t enc_cnt_capture_last;
    int32_t enc_cnt_delta;
    // Tracks encoder changes while in the DCME_STATE_OFF state
    uint32_t enc_cnt_failures;
    // The enc count when the speed transitioned to 0 for
    // calculating counts between commanding to stop and
    // when the motion finally ends.
    uint32_t enc_cnt_at_spd_zero;

    uint8_t off_transition_cnt_down;

    int pid_hndl;
    float pid_set_point;
    float pid_output;
    // Tracks failures while setting the PID set point
    uint32_t pid_setpt_failures;
    // Track PID refresh failures
    uint32_t pid_failures;
    // Tracks failures setting the PWM value
    uint32_t pwm_failures;

    uint8_t ramp_cnt;
    float ramp_inc; // If ramping up
    float ramp_dec; // If ramping down

#if defined(DCME_DBG_CAPTURE_DETAILS)
    dcme_dbg_data_t dbg_data;
    float last_setpt;
    uint8_t post_capture_count;
#endif

} dcme_data_t;

#define DCME_MAX_NUMBER_OF_HANDLES (2)
static dcme_data_t dcme_hndl_data[DCME_MAX_NUMBER_OF_HANDLES] = {0};

static uint32_t dcme_enc_isr_failures = 0;

static void calc_speed(int hndl);
static void dbg_data_capture(int hndl, bool force);
static void encoder_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void gather_data(int hndl);
// Returns 0 on success, otherwise -1
static int hw_init(int hndl, dcme_hw_t *hw, uint32_t refresh_ms);
static void hw_teardown(int hndl);
static void perform_pwm_update(int hndl);
static void process_state(int hndl);
// Returns a rounded value from a current range (0 to curr_max) to a new range (0 to new_max)
static float rounded_map(float input, float curr_max, float new_max);
static void set_motor_state(int hndl, dcme_state_t state);
static void set_pwm_duty_cycle(int hndl, float duty_cycle);

/*******************************************************************************
 * @brief dcme_init - initialize a DC Motor w/Encoder instance for use.
 *
 * @details A routine used to initialize a DC Motor w/Encoder instance for use.
 *
 * @param hw - the hardware control data for the DC motor and encoder
 * @param refresh_ms - the expected time, in milliseconds, between refresh()
 * function calls.
 *
 * @return returns an instance handle on success, otherwise -1.
 ******************************************************************************/

int dcme_init(dcme_hw_t *hw, uint32_t refresh_ms)
{
    int rc = -1;

    pthread_mutex_lock(&dcme_api_lock);

    // Find an available instance
    int tmp_hndl = 0;
    for (; tmp_hndl < DCME_MAX_NUMBER_OF_HANDLES; tmp_hndl ++) {
        if (dcme_hndl_data[tmp_hndl].initialized == false) {
            break;
        }
    }

    // If found, initialize the instance
    if (tmp_hndl < DCME_MAX_NUMBER_OF_HANDLES) {
        if (hw == NULL) {
            LOG_WRN("Invalid hw param");
        } else if ((hw->pwm == NULL) || (hw->enc == NULL) || (hw->fwd == NULL) || (hw->rev == NULL)) {
            LOG_WRN("Invalid hw pointer");
        } else if ((refresh_ms < PID_REFRESH_MS_MIN) && (refresh_ms > PID_REFRESH_MS_MAX)) {
            LOG_WRN("Invalid refresh rate");
        } else if (hw_init(tmp_hndl, hw, refresh_ms) != 0) {
            LOG_WRN("HW init failed");
        } else {
            dcme_data_t *data = &dcme_hndl_data[tmp_hndl];

            data->enc_cnt_at_spd_zero = 0;
            data->pid_setpt_failures = 0;
            data->pid_failures = 0;
            data->pwm_failures = 0;
            data->iir_filter_reset = true;
            data->initialized = true;
#if defined(DCME_DBG_CAPTURE_DETAILS)
            data->dbg_data.head = 0;
            data->dbg_data.cnt = 0;
            data->last_setpt = 0;
            data->post_capture_count = 0;
#endif
            dcme_enc_isr_failures = 0;
            rc = tmp_hndl;
        }
    }

    pthread_mutex_unlock(&dcme_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief dcme_exit - de-initialize the given motor/encoder handle
 *
 * @details A routine used to revert the initialization of the motor/encoder
 * instance.
 *
 * @param hndl - the instance handle to close.
 *
 * @return none.
 ******************************************************************************/

void dcme_exit(int hndl)
{
    pthread_mutex_lock(&dcme_api_lock);

    if ((hndl < 0) || (hndl >= DCME_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (dcme_hndl_data[hndl].initialized == false) {
        LOG_WRN("Already exit'd");
    } else {
        hw_teardown(hndl);
    }

    pthread_mutex_unlock(&dcme_api_lock);
}

/*******************************************************************************
 * @brief dcme_refresh - perform a periodic motor speed update
 *
 * @details A routine used to perform a motor speed update.  This routine should
 * be called at the refresh rate provided in the init() call, refresh_ms.
 *
 * @param hndl - the instance handle to used.
 *
 * @return none.
 ******************************************************************************/

void dcme_refresh(int hndl)
{
    pthread_mutex_lock(&dcme_api_lock);

    if ((hndl < 0) || (hndl >= DCME_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (dcme_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        gather_data(hndl);
        process_state(hndl);
        if (dcme_hndl_data[hndl].state != DCME_STATE_OFF)
        {
            perform_pwm_update(hndl);
            calc_speed(hndl);
        } else {
            dbg_data_capture(hndl, false);
        }
    }

    pthread_mutex_unlock(&dcme_api_lock);
}

/*******************************************************************************
 * @brief dcme_get_state - retrieves the current motor state
 *
 * @details A routine used to retrieve the current motor state.
 *
 * @param hndl - the instance handle to used.
 *
 * @return Returns the current motor state.
 ******************************************************************************/

dcme_state_t dcme_get_state(int hndl)
{
    dcme_state_t state = DCME_STATE_OFF;

    pthread_mutex_lock(&dcme_api_lock);

    if ((hndl < 0) || (hndl >= DCME_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (dcme_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        state = dcme_hndl_data[hndl].state;
    }

    pthread_mutex_unlock(&dcme_api_lock);

    return state;
}

/*******************************************************************************
 * @brief dcme_set_state - sets the motor state
 *
 * @details A routine used to configure the motor state to "state".
 *
 * @param hndl - the instance handle to used.
 * @param state - the motor state to set.
 *
 * @return Returns 0 if the state change is accepted, otherwise -1.
 ******************************************************************************/

int dcme_set_state(int hndl, dcme_state_t state)
{
    int rc = -1;

    pthread_mutex_lock(&dcme_api_lock);

#if defined(DCME_DBG_LOGGING)
    LOG_DBG("state %d %s", hndl, (state == 0) ? "off" : (state == 1) ? "fwd" : (state == 2) ? "rev" : "unk");
#endif

    if ((hndl < 0) || (hndl >= DCME_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (dcme_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        dcme_data_t *data = &dcme_hndl_data[hndl];

        if (state == DCME_STATE_OFF) { // Always handle a move to OFF
            data->exp_enc_cnt_per_refresh = 0;
            data->iir_filter_reset = true;
            set_motor_state(hndl, state);
            rc = 0;
        } else if (data->state == state) {
            // Nothing to do, just return success
            rc = 0;
        } else if (data->state != DCME_STATE_OFF) {
            // Just return an error since changing directly from FWD to REV or vice-versa
            LOG_WRN("Invalid state change");
        } else if (state == DCME_STATE_FWD || state == DCME_STATE_REV) {
            set_motor_state(hndl, state);
            rc = 0;

#if defined(DCME_DBG_CAPTURE_DETAILS)
            // Reset the data capture
            data->dbg_data.head = 0;
            data->dbg_data.cnt = 0;
#endif
        }
    }

    pthread_mutex_unlock(&dcme_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief dcme_get_status - get detailed status of the motor module
 *
 * @details A routine used to retrieve a detailed status of the motor module.
 *
 * @param hndl - the instance handle to used.
 * @param status - pointer to a status structure to fill in.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

extern int dcme_get_status(int hndl, dcme_status_t *status)
{
    int rc = -1;

    pthread_mutex_lock(&dcme_api_lock);

    if ((hndl < 0) || (hndl >= DCME_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (dcme_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else if (status == NULL) {
        // Can't return a status
    } else {
        status->state = dcme_hndl_data[hndl].state;
        status->exp_enc_cnt_per_refresh = dcme_hndl_data[hndl].exp_enc_cnt_per_refresh;
        status->enc_cnt = dcme_hndl_data[hndl].enc_cnt;
        status->enc_cnt_capture = dcme_hndl_data[hndl].enc_cnt_capture;
        status->enc_cnt_delta = dcme_hndl_data[hndl].enc_cnt_delta;
        status->enc_cnt_at_spd_zero = dcme_hndl_data[hndl].enc_cnt_at_spd_zero;
        status->enc_cnt_failures = dcme_hndl_data[hndl].enc_cnt_failures;
        status->enc_isr_failures = dcme_enc_isr_failures;
        status->pid_set_point = dcme_hndl_data[hndl].pid_set_point;
        status->pid_output = dcme_hndl_data[hndl].pid_output;
        status->pid_failures = dcme_hndl_data[hndl].pid_failures;
        status->pwm_failures = dcme_hndl_data[hndl].pwm_failures;
        rc = 0;
    }

    pthread_mutex_unlock(&dcme_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief dcme_get_speed - retrieve the last set motor speed value
 *
 * @details A routine used to retrieve the last set motor speed value.
 *
 * @param hndl - the instance handle to used.
 *
 * @return Returns the last set speed value.
 ******************************************************************************/

float dcme_get_speed(int hndl)
{
    float speed = 0.0f;

    pthread_mutex_lock(&dcme_api_lock);

    if ((hndl < 0) || (hndl >= DCME_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (dcme_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        speed = dcme_hndl_data[hndl].exp_enc_cnt_per_refresh;
    }

    pthread_mutex_unlock(&dcme_api_lock);

    return speed;
}

/*******************************************************************************
 * @brief dcme_get_actual_speed - retrieve the current motor speed value
 *
 * @details A routine used to retrieve the current motor speed value.
 *
 * @param hndl - the instance handle to used.
 *
 * @return Returns the actual recent movement speed value.
 ******************************************************************************/

float dcme_get_actual_speed(int hndl)
{
    float speed = 0.0f;

    pthread_mutex_lock(&dcme_api_lock);

    if ((hndl < 0) || (hndl >= DCME_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (dcme_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        speed = (dcme_hndl_data[hndl].iir_filtered_speed / DCME_IIR_FILTER_ELEMENT_CNT);
    }

    pthread_mutex_unlock(&dcme_api_lock);

    return speed;
}

/*******************************************************************************
 * @brief dcme_set_speed - set the motor speed value
 *
 * @details A routine used to set the motor speed value.
 *
 * @param hndl - the instance handle to used.
 * @param expEncCnts - the motor speed, in terms of encoder counts per refresh
 * period, to set.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int dcme_set_speed(int hndl, float exp_enc_cnts)
{
    int rc = -1;

    pthread_mutex_lock(&dcme_api_lock);

#if defined(DCME_DBG_LOGGING)
    LOG_DBG("speed %d %d.%d", hndl, (int)exp_enc_cnts, (((int)(exp_enc_cnts * 1000)) % 1000));
#endif

    if ((hndl < 0) || (hndl >= DCME_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (dcme_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        dcme_data_t *data = &dcme_hndl_data[hndl];
        float curr_exp_enc_cnts = data->exp_enc_cnt_per_refresh;

        if (data->state == DCME_STATE_OFF) {
            LOG_WRN("Motor off");
        } else if (exp_enc_cnts > DCME_MAX_EXP_ENC_CNTS_PER_REFRESH) {
            LOG_WRN("Invalid speed %d.%d", (int)exp_enc_cnts, (((int)(exp_enc_cnts * 1000)) % 1000));
        } else if (exp_enc_cnts == curr_exp_enc_cnts) {
            // Setting to existing speed, just return success
            rc = 0;
        } else {
            uint8_t ramp_cnt;
            float delta;

            if (data->exp_enc_cnt_per_refresh > DCME_FLOAT_ALMOST_ZERO && !(exp_enc_cnts > DCME_FLOAT_ALMOST_ZERO)) {
                data->enc_cnt_at_spd_zero = data->enc_cnt;
            }

            if (exp_enc_cnts > curr_exp_enc_cnts)
            {
                // Ramping up
                delta = exp_enc_cnts - curr_exp_enc_cnts;
                ramp_cnt = (uint8_t) (rounded_map(delta, (float) DCME_MAX_EXP_ENC_CNTS_PER_REFRESH, (float) DCME_MAX_RAMP_CNT));
                ramp_cnt = (ramp_cnt < DCME_MIN_RAMP_CNT) ? DCME_MIN_RAMP_CNT : ramp_cnt;
                data->ramp_cnt = ramp_cnt;
                data->ramp_inc = delta / ramp_cnt;
                data->ramp_dec = 0;
            }
            else
            {
                // Ramping down
                delta = curr_exp_enc_cnts - exp_enc_cnts;
                ramp_cnt = (uint8_t) (rounded_map(delta, (float) DCME_MAX_EXP_ENC_CNTS_PER_REFRESH, (float) DCME_MAX_RAMP_CNT));
                ramp_cnt = (ramp_cnt < DCME_MIN_RAMP_CNT) ? DCME_MIN_RAMP_CNT : ramp_cnt;
                data->ramp_cnt = ramp_cnt;
                data->ramp_inc = 0;
                data->ramp_dec = delta / data->ramp_cnt;
            }
            data->exp_enc_cnt_per_refresh = exp_enc_cnts;
            rc = 0;

#if defined(DCME_DBG_LOGGING)
            LOG_DBG("Ramp: cnt:%d; inc:%d; dec:%d", data->ramp_cnt, (int)data->ramp_inc, (int)data->ramp_dec);
#endif
        }
    }

    pthread_mutex_unlock(&dcme_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief dcme_get_enc_cnt - get the most recent captured encoder count value
 *
 * @details A routine used to retrieve the most recent captured encoder count
 * value.  This value is accumulative and should be used to compare with
 * previous values to determine a delta between two such values.
 *
 * @param hndl - the instance handle to used.
 *
 * @return Returns the most recent captured encoder count value.
 ******************************************************************************/

uint32_t dcme_get_enc_cnt(int hndl)
{
    uint32_t enc_cnts = 0;

    // Not locking the API mutex as this may be used a lot and locking is not
    // super important here.

    if ((hndl < 0) || (hndl >= DCME_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (dcme_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        enc_cnts = (dcme_hndl_data[hndl].enc_cnt_capture);
    }

    return enc_cnts;
}

/*******************************************************************************
 * @brief dcme_get_dbg_data - debug only function to retrieve captured data
 *
 * @details A routine used to retrieve certian details about the recent motor
 * movement.
 *
 * @param hndl - the instance handle to used.
 * @param data - a pointer to a pointer to retrieve a pointer to the data in.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

extern int dcme_get_dbg_data(int hndl, dcme_dbg_data_t **data)
{
    int rc = -1;

    pthread_mutex_lock(&dcme_api_lock);

#if defined(DCME_DBG_CAPTURE_DETAILS)
    if ((hndl < 0) || (hndl >= DCME_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (dcme_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else if (data == NULL) {
        LOG_WRN("Invalid param");
    } else {
        *data = &dcme_hndl_data[hndl].dbg_data;
        rc = 0;
    }
#else
    (void)hndl;
    (void)data;
#endif

    pthread_mutex_unlock(&dcme_api_lock);

    return rc;
}

/*****************************************************************************/

static void calc_speed(int hndl)
{
    dcme_data_t *data = &dcme_hndl_data[hndl];
    float iir;

    // We do not expect negative changes, just filtering in case
    float speedSample = (data->enc_cnt_delta >= 0) ? (float) data->enc_cnt_delta : 0.0f;

    iir = data->iir_filtered_speed;
    if (data->iir_filter_reset) {
        // Fill the filter with the first sample
        iir = (speedSample * DCME_IIR_FILTER_ELEMENT_CNT);
        data->iir_filter_reset = false;
    } else {
        // Substract an average sample and add the new one
        iir = iir - (iir / DCME_IIR_FILTER_ELEMENT_CNT);
        iir = iir + speedSample;
    }
    data->iir_filtered_speed = iir;
}

static void dbg_data_capture(int hndl, bool force)
{
#if defined(DCME_DBG_CAPTURE_DETAILS)
    dcme_data_t *data = &dcme_hndl_data[hndl];
    bool dbg_capture = force;

    if (data->pid_set_point != data->last_setpt) {
        data->last_setpt = data->pid_set_point;
        data->post_capture_count = 20;
        dbg_capture = true;
    } else if (data->post_capture_count > 0) {
        data->post_capture_count --;
        dbg_capture = true;
    }

    if (dbg_capture) {
#if defined(DCME_DBG_CAPTURE_START_ONLY)
        if (data->dbg_data.cnt < DCME_DBG_DATA_SIZE)
#endif
        {
            data->dbg_data.ts[data->dbg_data.head] = os_al_get_ms32();
            data->dbg_data.ramp_cnt[data->dbg_data.head] = data->ramp_cnt;
            data->dbg_data.enc_cnt[data->dbg_data.head] = data->enc_cnt;
            data->dbg_data.enc_delta[data->dbg_data.head] = data->enc_cnt_delta;
            data->dbg_data.pid_setpt[data->dbg_data.head] = data->pid_set_point;
            data->dbg_data.pid_out[data->dbg_data.head] = data->pid_output;
            data->dbg_data.head ++;
            if (data->dbg_data.head >= DCME_DBG_DATA_SIZE) {
                data->dbg_data.head = 0;
            }
            if (data->dbg_data.cnt < DCME_DBG_DATA_SIZE) {
                data->dbg_data.cnt ++;
            }
        }
    }
#else
    (void)hndl;
    (void)force;
#endif

}

static void encoder_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    (void)dev;

    // Just make sure our pin is interrupting
    if ((cb->pin_mask & pins) != 0) {
        // Find the proper hndl
        int hndl = -1;
        for (int i = 0; i < DCME_MAX_NUMBER_OF_HANDLES; i++) {
            if (cb == &dcme_hndl_data[i].enc_cb_data) {
                hndl = i;
                break;
            }
        }
        // Increase the encoder count, if hndl found
        if (hndl != -1) {
            dcme_hndl_data[hndl].enc_cnt ++;
            if (dcme_hndl_data[hndl].state == DCME_STATE_OFF) {
                dcme_hndl_data[hndl].enc_cnt_failures ++;
            }
        } else {
            // Count ISR failures
            dcme_enc_isr_failures ++;
        }
    }
}

static void gather_data(int hndl)
{
    dcme_data_t *data = &dcme_hndl_data[hndl];

    data->enc_cnt_capture_last = data->enc_cnt_capture;
    data->enc_cnt_capture = data->enc_cnt;
    if (data->enc_cnt_capture_last > data->enc_cnt_capture) { // Rollover case
        data->enc_cnt_delta = (UINT32_MAX - data->enc_cnt_capture_last) + data->enc_cnt_capture;
    } else {
        data->enc_cnt_delta = data->enc_cnt_capture - data->enc_cnt_capture_last;
    }
}

static int hw_init(int hndl, dcme_hw_t *hw, uint32_t refresh_ms)
{
    pid_init_data_t pid_data;
    int pid_hndl = -1;
    int rc = -1;

    // Setup the PID input structure
    pid_data.Kp = DCME_PID_GAIN_KP;
    pid_data.Ki = DCME_PID_GAIN_KI;
    pid_data.Kd = DCME_PID_GAIN_KD;
    pid_data.input = 0.0f;
    pid_data.output = 0.0f;
    pid_data.refresh_ms = refresh_ms;

    if (pwm_is_ready_dt(hw->pwm) != true) {
        LOG_WRN("PWM not ready");
    } else if (gpio_is_ready_dt(hw->enc) != true) {
        LOG_WRN("Encoder not ready");
    } else if (gpio_is_ready_dt(hw->fwd) != true) {
        LOG_WRN("Forward control not ready");
    } else if (gpio_is_ready_dt(hw->rev) != true) {
        LOG_WRN("Reverse control not ready");
	} else if (gpio_pin_configure_dt(hw->enc, (GPIO_INPUT | GPIO_PULL_UP)) != 0) {
		LOG_WRN("Encoder pin config failed");
	} else if (gpio_pin_interrupt_configure_dt(hw->enc, GPIO_INT_EDGE_BOTH) != 0) {
		LOG_WRN("Encoder isr config failed");
    } else if (gpio_pin_configure_dt(hw->fwd, GPIO_OUTPUT) != 0) {
		LOG_WRN("Forward pin config failed");
    } else if (gpio_pin_set_dt(hw->fwd, (hw->fwd_rev_active_low) ? 1 : 0) != 0) {
		LOG_WRN("Forward pin set failed");
    } else if (gpio_pin_configure_dt(hw->rev, GPIO_OUTPUT) != 0) {
		LOG_WRN("Reverse pin config failed");
    } else if (gpio_pin_set_dt(hw->rev, (hw->fwd_rev_active_low) ? 1 : 0) != 0) {
		LOG_WRN("Reverse pin set failed");
    } else if ((pid_hndl = pid_init(&pid_data)) < 0) {
		LOG_WRN("PID init failed");
    } else {
        dcme_data_t *data = &dcme_hndl_data[hndl];

        data->state = DCME_STATE_OFF;
        data->exp_enc_cnt_per_refresh = 0.0f;
        data->iir_filtered_speed = 0.0f;
        data->pid_hndl = pid_hndl;
        memcpy(&dcme_hndl_data[hndl].hw, hw, sizeof(dcme_hndl_data[hndl].hw));

        gpio_init_callback(&dcme_hndl_data[hndl].enc_cb_data, encoder_isr, BIT(hw->enc->pin));

	    if (gpio_add_callback(dcme_hndl_data[hndl].hw.enc->port, &dcme_hndl_data[hndl].enc_cb_data) == 0) {
            rc = 0;
        }
    }

    return rc;
}

static void hw_teardown(int hndl)
{
    gpio_remove_callback(dcme_hndl_data[hndl].hw.enc->port, &dcme_hndl_data[hndl].enc_cb_data);
}

static void perform_pwm_update(int hndl)
{
    dcme_data_t *data = &dcme_hndl_data[hndl];
    bool dbg_capture = false;

    if (data->ramp_cnt) {
        data->ramp_cnt--;
        if (data->ramp_cnt) {
            data->pid_set_point = (data->pid_set_point + data->ramp_inc) - data->ramp_dec;
        } else {
            data->pid_set_point = data->exp_enc_cnt_per_refresh;
        }
    }

    if (pid_set_setpoint(data->pid_hndl, data->pid_set_point) != 0) {
        data->pid_setpt_failures ++;
    }

    float output = 0.0f;
    if (data->pid_set_point > 0.0f) {
        if (pid_refresh(data->pid_hndl, data->enc_cnt_delta, &output) != 0) {
            data->pid_failures ++;
            output = data->pid_output; // Force the same output as the previous iteration
        }
        dbg_capture = true;
    }

    set_pwm_duty_cycle(hndl, output);

    // This must be done after the set_pwm_duty_cycle() to insure pid_output is set properly
    dbg_data_capture(hndl, dbg_capture);
}

static void process_state(int hndl)
{
    // The point here is to check for a possible transition to the OFF state

    dcme_data_t *data = &dcme_hndl_data[hndl];

    if (data->exp_enc_cnt_per_refresh != 0) {
        // Still expecting movement, so no possible transition
        data->off_transition_cnt_down = DCME_SAMPLES_TO_OFF;
    } else if (abs(data->enc_cnt_delta) > 2) {
        // Still moving, so no possible transition
        data->off_transition_cnt_down = DCME_SAMPLES_TO_OFF;
    } else if (data->off_transition_cnt_down > 0) {
        // Waiting for full count down
        data->off_transition_cnt_down--;
    } else if ((data->state != DCME_STATE_OFF) || (data->pid_output > DCME_FLOAT_ALMOST_ZERO)) {
        // We have reached the transition point
        set_motor_state(hndl, DCME_STATE_OFF);
        set_pwm_duty_cycle(hndl, 0.0f);
    }
}

// This assumes zero indexed maxes
static float rounded_map(float input, float curr_max, float new_max)
{
  // Calc percentage of input to its max, then apply that percent to the new max, then round
  return ((input / curr_max) * new_max) + 0.5f;
}

static void set_motor_state(int hndl, dcme_state_t state)
{
    dcme_data_t *data = &dcme_hndl_data[hndl];
    int fwd_val = 0;
    int rev_val = 0;

    switch (state) {
    case DCME_STATE_FWD:
        fwd_val = 1;
        rev_val = 0;
        break;
    case DCME_STATE_REV:
        fwd_val = 0;
        rev_val = 1;
        break;
    case DCME_STATE_OFF:
    default:
        fwd_val = 0;
        rev_val = 0;
        break;
    }

    if (data->hw.fwd_rev_active_low) {
        fwd_val = (fwd_val) ? 0 : 1;
        rev_val = (rev_val) ? 0 : 1;
    }

    gpio_pin_set_dt(data->hw.fwd, fwd_val);
    gpio_pin_set_dt(data->hw.rev, rev_val);

    data->state = state;
}

static void set_pwm_duty_cycle(int hndl, float duty_cycle)
{
    dcme_data_t *data = &dcme_hndl_data[hndl];

    uint32_t period = data->hw.pwm->period;
    uint32_t value = (uint32_t)(((period * duty_cycle) / PID_DEFAULT_OUTPUT_MAX) + 0.5f);

    if (pwm_set_pulse_dt(data->hw.pwm, value) != 0) {
        data->pwm_failures ++;
    } else {
        data->pid_output = duty_cycle;
    }
}