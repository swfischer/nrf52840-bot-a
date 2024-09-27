/*****************************************************************************
 * drive_ctrl.h - header file for the drive control module
 * 
 * This module will will move the system to the given position.  The movement
 * may not be a singular move, it may be done in smaller sections with some
 * intermitiate obsticle scanning done to clarify the route to take.
 * 
 * A callback mechanism is added to provide the caller with status along the
 * route and a completion status.
 *****************************************************************************/

#ifndef DRIVE_CTRL_H
#define DRIVE_CTRL_H

#include <stdint.h>

//! Limits of the "angle" parameter of the "drive_ctrl_move()" function
enum { DC_MOVE_ANGLE_DEG_MIN = -180, DC_MOVE_ANGLE_DEG_MAX = 180 };
//! Limits of the "distance" parameter of the "drive_ctrl_move()" function
static const float DC_MOVE_DIST_METERS_MIN = 0.1f;
static const float DC_MOVE_DIST_METERS_MAX = 10.0f;

typedef enum
{ DC_STATE_NONE = 0
, DC_STATE_TURNING
, DC_STATE_MOVING
, DC_STATE_SCANNING
, DC_STATE_COMPLETE
, DC_STATE_STOPPING
, DC_STATE_STOPPED
, DC_STATE_FAILURE
} dc_state_t;

typedef struct {
    dc_state_t state;
} dc_event_t;
typedef void (*drive_ctrl_cb_t)(const dc_event_t *event);

// Return 0 on success, otherwise -1
extern int drive_ctrl_init(void);
extern void drive_ctrl_exit(void);

// Returns 0 on success, otherwise -1
extern int drive_ctrl_reg_cb(drive_ctrl_cb_t cb);
extern void drive_ctrl_dereg_cb(drive_ctrl_cb_t cb);

// Returns 0 on success, otherwise -1
extern int drive_ctrl_move(int16_t angle, float distance);
extern void drive_ctrl_stop(void);

// Returns the current module state
extern dc_state_t drive_ctrl_get_state(void);

#endif // DRIVE_CTRL_H
