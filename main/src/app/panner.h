/*****************************************************************************
 * panner.h - The interface to pan the sensor left and right
 * 
 * The point of this interface is to provide the application with a simple
 * means of panning the sensor left and right.
 *****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#ifndef PANNER_H
#define PANNER_H

// Returns 0 on success, otherwise -1
extern int panner_init(void);
extern void panner_exit(void);

extern void panner_get_limits(int16_t *min, int16_t *max);

// Returns 0 on success, otherwise -1
extern int panner_enable(bool enable);
// Returns 0 on success, otherwise -1
extern int panner_get_position(int16_t *pos);
// Returns 0 on success, otherwise -1
extern int panner_set_position(int16_t pos);

#endif // PANNER_H
