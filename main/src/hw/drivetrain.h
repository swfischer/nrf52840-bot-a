/*****************************************************************************
 * drivetrain.h - header file for a dual independent drivetrain (think
 * tank tracks)
 *
 * For each motor, the motor speed must be stopped before changing directions.
 *****************************************************************************/

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <stdbool.h>
#include <stdint.h>

// Returns 0 on success, otherwise -1
extern int drivetrain_init(void);
extern void drivetrain_exit(void);

typedef struct {
    bool moving;
    int16_t left_ecps;
    int16_t right_ecps;
    uint32_t left_ec;
    uint32_t right_ec;
} drivetrain_status_t;
typedef void (*drivetrain_cb_t)(const drivetrain_status_t *status);
// Returns 0 on success, otherwise -1
extern int drivetrain_reg_cb(drivetrain_cb_t cb);
extern void drivetrain_dereg_cb(drivetrain_cb_t cb);

// Returns min/max encoder-counts-per-sec values
extern void drivetrain_get_motion_limits(int16_t *min_ecps, int16_t *max_ecps);
// Returns 0 on success, otherwise -1
extern void drivetrain_get_motion(int16_t *left_ecps, int16_t *right_ecps);
// Returns 0 on success, otherwise -1
extern int drivetrain_set_motion(int16_t left_ecps, int16_t right_ecps);
extern void drivetrain_stop(void);

// Returns current left/right encoder count values
extern void drivetrain_get_enc_cnts(uint32_t *left_ec, uint32_t *right_ec);

// For debug purposes only:
extern int drivetrain_get_dcme_handle(bool left);

#endif // DRIVETRAIN_H
