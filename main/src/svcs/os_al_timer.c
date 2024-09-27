/*****************************************************************************
 * os_al_timer.c - The source to an OS abstracted timer API
 * 
 * The point of this interface is to provide some OS agnostic functions to
 * make coding easier.
 *****************************************************************************/

#include "os_al.h"

#include <pthread.h>
#include <string.h>
#include <zephyr/kernel.h>

#include "logger.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

// This code is Zephyr specific, so fail for other build types
#if !defined(ZEPHYR_BUILD)
#error "OS AL Timer code is Zephyr specific"
#endif

//! This mutex is used to serialize calls to the global functions
static pthread_mutex_t os_al_timer_api_lock = PTHREAD_MUTEX_INITIALIZER;

typedef struct timer_data {
    struct k_timer rtos_timer;
    os_al_timer_exp_cb_t exp_cb;
    bool used;
} timer_data_t;

#define OS_AL_TIMER_MAX_TIMERS (20)
static timer_data_t os_al_timer_list[OS_AL_TIMER_MAX_TIMERS] = {0};

static bool os_al_timer_initialized = false;

static void os_al_timer_expiry(struct k_timer *timer);
static void os_al_timer_init(void);

/*******************************************************************************
 * @brief Allows creation of a multi-use timer.
 *
 * @details A function used to create a multi-use timer.
 *
 * @param exp_cb - the expiration callback function.
 *
 * @return Returns timer handle on success, otherwise -1
 ******************************************************************************/

os_al_timer_t os_al_timer_create(os_al_timer_exp_cb_t exp_cb)
{
    os_al_timer_t hndl = -1;

    pthread_mutex_lock(&os_al_timer_api_lock);

    if (!os_al_timer_initialized) {
        os_al_timer_init();
    }

    do {
        if (exp_cb == NULL) {
            LOG_WRN("Invalid param");
            break;
        }

        // Find an unused list entry
        int idx = -1;
        for (int i = 0; i < OS_AL_TIMER_MAX_TIMERS; i++) {
            if (!os_al_timer_list[i].used) {
                idx = i;
                break;
            }
        }

        if (idx == -1) {
            LOG_WRN("No available timers");
        } else {
            k_timer_init(&os_al_timer_list[idx].rtos_timer, os_al_timer_expiry, NULL);
            os_al_timer_list[idx].exp_cb = exp_cb;
            os_al_timer_list[idx].used = true;
            hndl = idx;
        }
    } while (0);

    pthread_mutex_unlock(&os_al_timer_api_lock);

    return hndl;
}

/*******************************************************************************
 * @brief Allows destruction of a previously created multi-use timer.
 *
 * @details A function used to destroy a previously created multi-use timer.
 *
 * @param timer - the timer to destroy.
 *
 * @return None
 ******************************************************************************/

void os_al_timer_destroy(os_al_timer_t timer)
{
    pthread_mutex_lock(&os_al_timer_api_lock);

    do {
        if (!os_al_timer_initialized) {
            LOG_WRN("No created timers");
            break;
        }

        if ((timer < 0) || (timer >= OS_AL_TIMER_MAX_TIMERS)) {
            LOG_WRN("Invalid timer");
            break;
        }

        if (!os_al_timer_list[timer].used) {
            LOG_WRN("Destroy unused timer");
            break;
        }

        (void)memset((uint8_t *)&os_al_timer_list[timer].rtos_timer, 0, sizeof(os_al_timer_list[timer].rtos_timer));
        os_al_timer_list[timer].used = false;

    } while (0);

    pthread_mutex_unlock(&os_al_timer_api_lock);
}

/*******************************************************************************
 * @brief Starts or restarts a previously created multi-use timer.
 *
 * @details A function used to start or restart a previously created multi-use
 * timer.  Starting a timer that is running is allowed.  The timer count is
 * reset with this start call.  The timer can be a one-shot timer, expires once,
 * or a periodic timer, delaying and expiring until cancelled.
 *
 * @param timer - the timer to start or restart.
 * @param ms - the timer delay in milliseconds.
 * @param type - either TIMER_TYPE_ONE_SHOT or TIMER_TYPE_PERIODIC.
 *
 * @return Returns 0 on success, otherwise -1
 ******************************************************************************/

int os_al_timer_start(os_al_timer_t timer, uint32_t ms, os_al_timer_type_t type)
{
    int rc = -1;

    pthread_mutex_lock(&os_al_timer_api_lock);

    do {
        if (!os_al_timer_initialized) {
            LOG_WRN("No created timers");
            break;
        }

        if ((timer < 0) || (timer >= OS_AL_TIMER_MAX_TIMERS)) {
            LOG_WRN("Invalid timer");
            break;
        }

        if (!os_al_timer_list[timer].used) {
            LOG_WRN("Invalid timer");
            break;
        }

        if (type == TIMER_TYPE_ONE_SHOT) {
            k_timer_start(&os_al_timer_list[timer].rtos_timer, K_MSEC(ms), K_NO_WAIT);
            rc = 0;
        } else if (type == TIMER_TYPE_PERIODIC) {
            k_timer_start(&os_al_timer_list[timer].rtos_timer, K_MSEC(ms), K_MSEC(ms));
            rc = 0;
        } else {
            LOG_WRN("Invalid type");
        }

    } while (0);

    pthread_mutex_unlock(&os_al_timer_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief Cancels, or stops, a previously created and running multi-use timer.
 *
 * @details A function used to cancel, or stop, a previously created and running
 * multi-use timer.  Cancelling a stopped timer is allowed.
 *
 * @param timer - the timer to cancel.
 *
 * @return None
 ******************************************************************************/

void os_al_timer_cancel(os_al_timer_t timer)
{
    pthread_mutex_lock(&os_al_timer_api_lock);

    do {
        if (!os_al_timer_initialized) {
            LOG_WRN("No created timers");
            break;
        }

        if ((timer < 0) || (timer >= OS_AL_TIMER_MAX_TIMERS)) {
            LOG_WRN("Invalid timer");
            break;
        }

        if (!os_al_timer_list[timer].used) {
            LOG_WRN("Invalid timer");
            break;
        }

        k_timer_stop(&os_al_timer_list[timer].rtos_timer);

    } while (0);

    pthread_mutex_unlock(&os_al_timer_api_lock);
}

/*******************************************************************************
 * @brief Retrieves the remaining time before expiration of a multi-use timer.
 *
 * @details A function used to retrieve the remaining time, in milliseconds,
 * before a currently running timer expires.  For non-running timers, zero is
 * returned.
 *
 * @param timer - the timer to retrieve the remaining time for.
 *
 * @return Returns the time, in milliseconds, before the (next) expiration
 ******************************************************************************/

uint32_t os_al_timer_remaining(os_al_timer_t timer)
{
    uint32_t remaining = 0;

    pthread_mutex_lock(&os_al_timer_api_lock);

    do {
        if (!os_al_timer_initialized) {
            LOG_WRN("No created timers");
            break;
        }

        if ((timer < 0) || (timer >= OS_AL_TIMER_MAX_TIMERS)) {
            LOG_WRN("Invalid timer");
            break;
        }

        if (!os_al_timer_list[timer].used) {
            LOG_WRN("Invalid timer");
            break;
        }

        remaining = k_timer_remaining_get(&os_al_timer_list[timer].rtos_timer);

    } while (0);

    pthread_mutex_unlock(&os_al_timer_api_lock);

    return remaining;
}

// Function called during an RTOS timer expiration which redirects to the OSAL timer callbacks
static void os_al_timer_expiry(struct k_timer *timer)
{
    // Don't lock the mutex here so as to not cause deadlocks if the callback
    // calls into the timer

    if (timer != NULL) {
        // Find the matching timer and call its callback
        int i = 0;
        for (; i < OS_AL_TIMER_MAX_TIMERS; i++) {
            if ((timer == &os_al_timer_list[i].rtos_timer) && (os_al_timer_list[i].used == true)) {
                os_al_timer_list[i].exp_cb(i);
                break;
            }
        }
        if (i >= OS_AL_TIMER_MAX_TIMERS) {
            LOG_WRN("No expiry timer found");
        }
    }
}

// One time init of the internal timer structures.
static void os_al_timer_init(void)
{
    for (int i = 0; i < OS_AL_TIMER_MAX_TIMERS; i++) {
        os_al_timer_list[i].used = false;
    }
    os_al_timer_initialized = true;
}
