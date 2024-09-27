/*****************************************************************************
 * pid.c - source file for PID (Proportional-Integral-Derivative controller)
 * module
 *
 * It is assumed that the user of this interface will not call public
 * functions while another public function is running
 *
 * This code started from: https://github.com/br3ttb/Arduino-PID-Library
 * The original filename was "PID_v1.h"
 *****************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 *****************************************************************************/

#include "pid.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>  // for usleep()

#include "logger.h"
#include "os_al.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

typedef struct
{
    bool initialized;
    float Kp;
    float Ki;
    float Kd;
    float setpt;
    float last_input;
    float output_sum;
    float output_min;
    float output_max;
    uint32_t refresh_ms;
    uint32_t last_refresh_ms;

} pid_data_t;

#define MAX_NUMBER_OF_HANDLES (2)
static pid_data_t pid_hndl_data[MAX_NUMBER_OF_HANDLES] = {0};

static void init(int hndl, pid_init_data_t *init_data);
// Returns 0 on success and provides an output value, otherwise -1
static int perform_refresh(int hndl, float input, float *output);

/*******************************************************************************
 * @brief pid_init - initialize a pid instance for use.
 *
 * @details A routine used to initialize a PID instance for use.
 *
 * @param init_data - the PID initialization data
 *
 * @return returns an instance handle on success, otherwise -1.
 ******************************************************************************/

int pid_init(pid_init_data_t *init_data)
{
    int hndl = -1;

    // Find an available instance
    int tmp_hndl = 0;
    for (; tmp_hndl < MAX_NUMBER_OF_HANDLES; tmp_hndl ++) {
        if (pid_hndl_data[tmp_hndl].initialized == false) {
            break;
        }
    }

    // If found, initialize the instance
    if (tmp_hndl < MAX_NUMBER_OF_HANDLES) {
        if (init_data == NULL) {
            LOG_WRN("Invalid params");
        } else if ((init_data->refresh_ms < PID_REFRESH_MS_MIN) && (init_data->refresh_ms > PID_REFRESH_MS_MAX)) {
            LOG_WRN("Invalid refresh rate");
        } else {
            hndl = tmp_hndl;
            init(hndl, init_data);
            pid_hndl_data[hndl].initialized = true;
        }
    }

    return hndl;
}

/*******************************************************************************
 * @brief pid_exit - de-initialize the given PID handle
 *
 * @details A routine used to revert the initialization of the PID instance.
 *
 * @param hndl - the instance handle to close.
 *
 * @return none.
 ******************************************************************************/

void pid_exit(int hndl)
{
    if (hndl < 0 || hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid handle");
    } else if (pid_hndl_data[hndl].initialized == false) {
        LOG_WRN("Already exited");
    } else {
        pid_hndl_data[hndl].initialized = true;
    }
}

/*******************************************************************************
 * @brief pid_refresh - perform a PID refresh
 *
 * @details A routine used to perform a refresh of the PID loop.
 *
 * @param hndl - the instance handle to use.
 * @param input - the input value for the PID loop.
 * @param output - pointer where the PID loop output is to be stored.
 *
 * @return Returns 0 on success and provides an output value, otherwise -1.
 ******************************************************************************/

int pid_refresh(int hndl, float input, float *output)
{
    int rc = -1;

    if (hndl < 0 || hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid handle");
    } else if (pid_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        rc = perform_refresh(hndl, input, output);
    }

    return rc;
}

/*******************************************************************************
 * @brief pid_get_setpoint - retrieve the current set-point
 *
 * @details A routine used to retrieve the current PID loop set-point.
 *
 * @param hndl - the instance handle to use.
 *
 * @return Returns the current PID set-point.
 ******************************************************************************/

float pid_get_setpoint(int hndl)
{
    float setpt = 0.0f;

    if (hndl < 0 || hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid handle");
    } else if (pid_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        setpt = pid_hndl_data[hndl].setpt;
    }

    return setpt;
}

/*******************************************************************************
 * @brief pid_set_setpoint - set the PID loop set-point
 *
 * @details A routine used to set the PID loop set-point.
 *
 * @param hndl - the instance handle to use.
 * @param setpt - the set-point value to set.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int pid_set_setpoint(int hndl, float setpt)
{
    int rc = -1;

    if (hndl < 0 || hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid handle");
    } else if (pid_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        pid_hndl_data[hndl].setpt = setpt;
        rc = 0;
    }

    return rc;
}

/*******************************************************************************
 * @brief pid_set_output_limits - set the PID loop output value limits
 *
 * @details A routine used to set the PID loop output limits.
 *
 * @param hndl - the instance handle to use.
 * @param min - the output value minimum value.
 * @param max - the output value maximum value.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int pid_set_output_limits(int hndl, float min, float max)
{
    int rc = -1;

    if (hndl < 0 || hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid handle");
    } else if (pid_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else if (min > max) {
        LOG_WRN("Invalid params");
    } else {
        pid_hndl_data[hndl].output_min = min;
        pid_hndl_data[hndl].output_max = max;
        rc = 0;
    }

    return rc;
}

/*****************************************************************************/

static void init(int hndl, pid_init_data_t *init_data)
{
    pid_data_t *pid = &pid_hndl_data[hndl];

    pid->Kp = init_data->Kp;
    pid->Ki = init_data->Ki;
    pid->Kd = init_data->Kd;
    pid->setpt = PID_DEFAULT_SET_POINT;
    pid->output_min = PID_DEFAULT_OUTPUT_MIN;
    pid->output_max = PID_DEFAULT_OUTPUT_MAX;
    pid->last_input = init_data->input;
    pid->output_sum = init_data->output;
    if (pid->output_sum > pid->output_max)
    {
        pid->output_sum = pid->output_max;
    }
    else if (pid->output_sum < pid->output_min)
    {
        pid->output_sum = pid->output_min;
    }

    pid->refresh_ms = init_data->refresh_ms;
    pid->last_refresh_ms = os_al_get_ms32();
}

// Returns 0 on success and provides an output value, otherwise -1
static int perform_refresh(int hndl, float input, float *output)
{
    int rc = -1;

    pid_data_t *pid = &pid_hndl_data[hndl];

    unsigned long now = os_al_get_ms32();
//    unsigned long delta_ms = (now - pid->last_refresh_ms);

//    if (delta_ms >= pid->refresh_ms) {
    {
        // Compute all the working error variables
        float error = pid->setpt - input;
        float delta_input = (input - pid->last_input);
        pid->output_sum += (pid->Ki * error);
        pid->output_sum -= (pid->Kp * delta_input);
        // Truncate if necessary
        if (pid->output_sum > pid->output_max)
        {
            pid->output_sum = pid->output_max;
        }
        else if (pid->output_sum < pid->output_min)
        {
            pid->output_sum = pid->output_min;
        }

        float loop_output = pid->output_sum - (pid->Kd * delta_input);
        // Truncate as necessary
        if (loop_output > pid->output_max)
        {
            loop_output = pid->output_max;
        }
        else if (loop_output < pid->output_min)
        {
            loop_output = pid->output_min;
        }
        // Set the return PID output value
        *output = loop_output;

        // Remember some variables for next time
        pid->last_input = input;
        pid->last_refresh_ms = now;

        rc = 0;
    }

    return rc;
}
