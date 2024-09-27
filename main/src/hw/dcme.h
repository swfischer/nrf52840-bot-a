/*****************************************************************************
 * dcme.h - header file for DC Motor with Encoder module
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

#ifndef DCME_H
#define DCME_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

// Possible motor states
typedef enum { DCME_STATE_OFF = 0, DCME_STATE_FWD, DCME_STATE_REV } dcme_state_t;

typedef struct {
    const struct pwm_dt_spec *pwm; // The motor speed PWM signal
    const struct gpio_dt_spec *enc; // GPIO with encoder signal connected
    const struct gpio_dt_spec *fwd; // GPIO to enable forward movement
    const struct gpio_dt_spec *rev; // GPIO to enable reverse movement
    bool fwd_rev_active_low;
} dcme_hw_t;

typedef struct {
    dcme_state_t state;
    float exp_enc_cnt_per_refresh;
    uint32_t enc_cnt;
    uint32_t enc_cnt_capture;
    int32_t enc_cnt_delta;
    uint32_t enc_cnt_at_spd_zero;
    uint32_t enc_cnt_failures;
    uint32_t enc_isr_failures;
    float pid_set_point;
    float pid_output;
    uint32_t pid_failures;
    uint32_t pwm_failures;
} dcme_status_t;

// Returns an instance handle on success, otherwise -1
extern int dcme_init(dcme_hw_t *hw, uint32_t refresh_ms);
extern void dcme_exit(int hndl);

extern void dcme_refresh(int hndl);

// Returns the current motor state
extern dcme_state_t dcme_get_state(int hndl);
// Returns 0 if the state change is accepted, otherwise -1
extern int dcme_set_state(int hndl, dcme_state_t state);

// Returns 0 on success (with detailed motor status), otherwise -1
extern int dcme_get_status(int hndl, dcme_status_t *status);

// The maximum expected encoder counts per refresh period (speed setting)
static const float DCME_MAX_EXP_ENC_CNTS_PER_REFRESH = 40.0f;
// Returns the last set speed value
extern float dcme_get_speed(int hndl);
// Returns the actual recent movement speed value
extern float dcme_get_actual_speed(int hndl);
// Returns 0 if the speed change is accepted, otherwise -1
extern int dcme_set_speed(int hndl, float exp_enc_cnts);

// Returns the most recent captured encoder count value
extern uint32_t dcme_get_enc_cnt(int hndl);

// For debug purposes only, retrieve recent motor movement details
#define DCME_DBG_DATA_SIZE (200)
typedef struct {
    uint16_t head;
    uint16_t cnt;
    uint32_t ts[DCME_DBG_DATA_SIZE];
    uint8_t ramp_cnt[DCME_DBG_DATA_SIZE];
    uint32_t enc_cnt[DCME_DBG_DATA_SIZE];
    uint32_t enc_delta[DCME_DBG_DATA_SIZE];
    float pid_setpt[DCME_DBG_DATA_SIZE];
    float pid_out[DCME_DBG_DATA_SIZE];
} dcme_dbg_data_t;
extern int dcme_get_dbg_data(int hndl, dcme_dbg_data_t **data);

#endif // DCME_H
