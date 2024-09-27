/*****************************************************************************
 * pid.h - header file for PID (Proportional-Integral-Derivative controller)
 * module
 *
 * This code started from: https://github.com/br3ttb/Arduino-PID-Library
 * The original filename was "PID_v1.h"
 *****************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 *****************************************************************************/

#ifndef PID_H
#define PID_H

#include <stdint.h>

#define PID_REFRESH_MS_MIN (1)
#define PID_REFRESH_MS_MAX (2000)

static const float PID_DEFAULT_SET_POINT = 0.0f;
static const float PID_DEFAULT_OUTPUT_MIN = 0.0f;
static const float PID_DEFAULT_OUTPUT_MAX = 255.0f;

typedef struct {

    float Kp;
    float Ki;
    float Kd;
    float input;
    float output;
    uint32_t refresh_ms;

} pid_init_data_t;

// Returns an instance handle on success, otherwise -1
extern int pid_init(pid_init_data_t *init_data);
extern void pid_exit(int hndl);

// Returns 0 on success and provides an output value, otherwise -1
extern int pid_refresh(int hndl, float input, float *output);

// Returns the current PID set-point
extern float pid_get_setpoint(int hndl);
// Returns 0 on success, otherwise -1
extern int pid_set_setpoint(int hndl, float setpt);
// Returns 0 on success, otherwise -1
extern int pid_set_output_limits(int hndl, float min, float max);

#endif // PID_H
