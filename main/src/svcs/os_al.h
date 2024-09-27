/*****************************************************************************
 * os_al.h - The interface to some OS abstracted functions
 * 
 * The point of this interface is to provide some OS agnostic functions to
 * make coding easier.
 *****************************************************************************/

#include <stdint.h>

#ifndef OS_AL_H
#define OS_AL_H

// ************************************************************************* //
// An API to retrieve a millisecond timestamp
extern uint32_t os_al_get_ms32(void);

// ************************************************************************* //
// APIs to abstract a timer

typedef int os_al_timer_t;
typedef void (*os_al_timer_exp_cb_t)(os_al_timer_t timer);
typedef enum { TIMER_TYPE_ONE_SHOT = 1, TIMER_TYPE_PERIODIC } os_al_timer_type_t;

// Returns timer handle on success, otherwise -1
extern os_al_timer_t os_al_timer_create(os_al_timer_exp_cb_t exp_cb);
extern void os_al_timer_destroy(os_al_timer_t timer);
// Returns 0 on success, otherwise -1
extern int os_al_timer_start(os_al_timer_t timer, uint32_t ms, os_al_timer_type_t type);
extern void os_al_timer_cancel(os_al_timer_t timer);
// Returns the time, in milliseconds, before the (next) expiration
extern uint32_t os_al_timer_remaining(os_al_timer_t timer);

// ************************************************************************* //

#endif // OS_AL_H
