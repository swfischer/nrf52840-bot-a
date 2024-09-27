/*****************************************************************************
 * lidar.h - The interface to read distances from the LIDAR sensor
 * 
 * The point of this interface is to provide the application with a simple
 * means of reading from the LIDAR sensor.
 *****************************************************************************/

#include <stdint.h>

#ifndef LIDAR_H
#define LIDAR_H

typedef enum
{ LIDAR_FAILURE = -1
, LIDAR_SUCCESS = 0
, LIDAR_TOO_DARK = 1
, LIDAR_TOO_BRIGHT = 2
, LIDAR_TOO_FAR = 3
, LIDAR_TOO_CLOSE = 4
} lidar_status_t;

// Returns 0 on success, otherwise -1
extern int lidar_init(void);
extern void lidar_exit(void);

// Returns 0 on success, otherwise -1
extern int lidar_get_limits(uint16_t *min, uint16_t *max);

// Returns LIDAR status value
extern lidar_status_t lidar_get_distance(float *meters);
// Returns LIDAR status value
extern lidar_status_t lidar_get_raw_cm(uint16_t *cm);

#endif // LIDAR_H
