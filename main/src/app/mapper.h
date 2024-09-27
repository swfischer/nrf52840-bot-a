/*****************************************************************************
 * mapper.h - header file for the mapper module
 * 
 * This module will use a servo to point a lidar sensor in different
 * directions and then capture a distance reading, thus creating a map of the
 * surfaces immediately in front of the sensor.
 * 
 * The output will be a simple list of measured distances and the angle the
 * measurement was taken.
 * 
 * The mapping behavior will be kicked off within the module via a call to the
 * mapper_start_sweep() function.
 *****************************************************************************/

#ifndef MAPPER_H
#define MAPPER_H

#include <stdint.h>

typedef enum { MAP_TYPE_FULL = 0, MAP_TYPE_SHORT } mapper_type_t;

typedef enum { MAP_STATE_NONE = 0, MAP_STATE_RUNNING, MAP_STATE_COMPLETE, MAP_STATE_FAILURE } mapper_state_t;
typedef struct {
    mapper_state_t state;
} mapper_event_t;
typedef void (*mapper_cb_t)(const mapper_event_t *event);

// The degrees between sweep measurement points
#define MAPPER_SWEEP_INCREMENT_DEGREES (1)
// Assume a maximum sweep of 360 degrees
#define MAPPER_MAX_POINTS (360 / MAPPER_SWEEP_INCREMENT_DEGREES)
// A sweep data sample with meters == -1 implies the sample was too dark
#define MAPPER_METERS_TOO_DARK (-1.0f)
// A sweep data sample with meters == -2 implies the sample was too bright
#define MAPPER_METERS_TOO_BRIGHT (-2.0f)
// A sweep data sample with meters == -3 implies the distance is too far to determine
#define MAPPER_METERS_TOO_FAR (-3.0f)
// A sweep data sample with meters == -4 implies the distance is too close to determine
#define MAPPER_METERS_TOO_CLOSE (-4.0f)

typedef struct {
    uint16_t count;
    int16_t angle[MAPPER_MAX_POINTS];
    float meters[MAPPER_MAX_POINTS];
} sweep_data_t;

// Return 0 on success, otherwise -1
extern int mapper_init(void);
extern void mapper_exit(void);

// Returns 0 on success, otherwise -1
extern int mapper_reg_cb(mapper_cb_t cb);
extern void mapper_dereg_cb(mapper_cb_t cb);

extern mapper_state_t mapper_get_state(void);

// Returns 0 on success, otherwise -1
extern int mapper_start_sweep(mapper_type_t type, sweep_data_t *sweep_data);
extern void mapper_abort_sweep(void);

#endif // MAPPER_H