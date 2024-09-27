/*****************************************************************************
 * start_stop.h - header file for providing system start and stop indications
 *
 *****************************************************************************/

#ifndef START_STOP_H
#define START_STOP_H

#include <stdbool.h>

// Returns 0 on success, otherwise -1
extern int start_stop_init(void);
extern void start_stop_exit(void);

typedef struct {
    bool start_not_stop;
} start_stop_event_t;
typedef void (*start_stop_cb_t)(const start_stop_event_t *event);
// Returns 0 on success, otherwise -1
extern int start_stop_reg_cb(start_stop_cb_t cb);
extern void start_stop_dereg_cb(start_stop_cb_t cb);

extern void start_stop_auto_stop(void);

#endif // START_STOP_H
