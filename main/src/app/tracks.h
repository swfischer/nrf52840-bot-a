/*****************************************************************************
 * tracks.h - header file for controlling a dual independent track mechanism
 * (think tank tracks)
 *
 *****************************************************************************/

#ifndef TRACKS_H
#define TRACKS_H

#include <stdbool.h>
#include <stdint.h>

// System error states
typedef enum {TRKS_ERR_NONE = 0, TRKS_ERR_MOVEMENT_LOST_SYNC } trks_errors_t;

// Returns 0 on success, otherwise -1
extern int tracks_init(void);
extern void tracks_exit(void);

typedef struct {
    bool moving;
    trks_errors_t syserr;
} tracks_event_t;
typedef void (*tracks_cb_t)(const tracks_event_t *event);
// Returns 0 on success, otherwise -1
extern int tracks_reg_cb(tracks_cb_t cb);
extern void tracks_dereg_cb(tracks_cb_t cb);

extern void tracks_stop(void);
// Positive centimeter value for forward moves and negative for reverse moves
// Returns 0 on success, otherwise -1
extern int tracks_move(int16_t centimeters);
enum { DT_PIVOT_DEGREES_MIN = -180, DT_PIVOT_DEGREES_MAX = 180 };
// Positive degrees values for CW moves and negative for CCW moves
// Returns 0 on success, otherwise -1
extern int tracks_pivot(int16_t degrees);
// Returns the estimated distance traveled during the current or last move operation
extern int16_t tracks_last_move_cm(void);
// Returns the estimated pivot degrees during the current or last pivot operation
extern int16_t tracks_last_pivot_deg(void);
// Returns true if all movement has stopped, otherwise false
extern bool tracks_is_stopped(void);
// Returns the current system error state
extern trks_errors_t tracks_error_state(void);

#endif // TRACKS_H
