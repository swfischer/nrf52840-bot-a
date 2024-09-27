/*****************************************************************************
 * drive_ctrl.c - source file for the drive control module
 * 
 * This module will will move the system to the given position.  The movement
 * may not be a singular move, it may be done in smaller sections with some
 * intermitiate obsticle scanning done to clarify the route to take.
 * 
 * A callback mechanism is added to provide the caller with status along the
 * route and a completion status.
 * 
 * It is assumed that the mapper module is initialized and available for use.
 *****************************************************************************/

#include "drive_ctrl.h"

#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

#include "logger.h"
#include "mapper.h"
#include "os_al.h"
#include "tracks.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

//! This mutex is used by the pthread and dc_cond condition
static pthread_mutex_t dc_lock = PTHREAD_MUTEX_INITIALIZER;
//! This mutex is used by the pthread and dc_move_cond condition
static pthread_mutex_t dc_move_lock = PTHREAD_MUTEX_INITIALIZER;
//! This mutex is used to prevent state update collisions
static pthread_mutex_t dc_state_lock = PTHREAD_MUTEX_INITIALIZER;
//! This condition is for waiting for a move start request
static pthread_cond_t dc_cond = PTHREAD_COND_INITIALIZER;
//! This condition is for waiting for the next operation of a move
static pthread_cond_t dc_move_cond = PTHREAD_COND_INITIALIZER;
//! The application thread
static pthread_t dc_thread = {0};

#define MAX_FORWARD_MOVEMENT_SEGMENT_CM (50.0f)
#define SAFE_SIDE_DISTANCE_CM (20.0f)

#if defined(ZEPHYR_BUILD)
#define DC_THREAD_STACK_SIZE (1000)
static K_THREAD_STACK_DEFINE(dc_thread_stack, DC_THREAD_STACK_SIZE);
#endif

//! Flag used to force the application thread to exit
static bool dc_thread_exit = false;

//! Flag used to know if the module has been initialized or not
static bool dc_initialized = false;

//! API callback function pointer
static drive_ctrl_cb_t dc_callback_func = NULL;

static dc_state_t dc_state = DC_STATE_NONE;
static int16_t dc_move_angle = 0;
static float dc_move_distance = 0.0f; // centimeters

static sweep_data_t dc_scan_data;
static bool dc_operation_failure = false;

// Return not used
static void *dc_thread_loop(void *arg);
// Returns 0 on success, otherwise -1
static int dc_thread_setup_and_start(void);
static void dc_thread_wake_and_join(void);
// Returns the number of centimeters to move or <0.0f if movement is complete
static float determine_move_distance(void);
static void force_stop(void);
// Returns true if the immediate path is clear for the next movement forward
static bool is_path_clear(void);
// Returns true if in running state, otherwise false
static bool is_running_state(void);
static void mapper_cb(const mapper_event_t *event);
// Returns 0 on success, otherwise -1
static int start_next_move_operation(void);
static void tracks_cb(const tracks_event_t *event);
static void update_state(dc_state_t new_state);

/*****************************************************************************
 * @brief drive_ctrl_init - initialize the drive control module
 * 
 * @details Performs setup and starts a thread to handle the module
 * 
 * @return Returns 0 on success, otherwise -1
 *****************************************************************************/

int drive_ctrl_init(void)
{
    int rc = -1;

    if (dc_initialized == true) {
        LOG_INF("Already started");
	} else if (tracks_init() != 0) {
		LOG_ERR("Tracks init failed");
	} else if (tracks_reg_cb(tracks_cb) != 0) {
		LOG_ERR("Tracks cb reg failed");
    } else if (mapper_reg_cb(mapper_cb) != 0) {
        LOG_WRN("Mapper reg failed");
    } else if (dc_thread_setup_and_start() != 0) {
        LOG_WRN("Thread startup failed");
    } else {
        dc_initialized = true;
        dc_state = DC_STATE_NONE;
        dc_move_angle = 0;
        dc_move_distance = 0.0f;
        dc_callback_func = NULL;
        rc = 0;
    }

    return rc;
}

/*****************************************************************************
 * @brief drive_ctrl_exit - force the drive control module to exit
 * 
 * @details Forces the drive control module to exit and does not return until
 * the module has exited.
 *****************************************************************************/

void drive_ctrl_exit(void)
{
    if (dc_initialized == false) {
        LOG_INF("Already exited");
    } else {
        bool wait_for_stopped = false;

        if ((dc_state == DC_STATE_TURNING)
                || (dc_state == DC_STATE_MOVING)
                || (dc_state == DC_STATE_SCANNING)) {
            force_stop();
            wait_for_stopped = true;
        } else if (dc_state == DC_STATE_STOPPING) {
            wait_for_stopped = true;
        }

        if (wait_for_stopped) {
            while (dc_state != DC_STATE_STOPPED) {
                usleep(20000);
            }
        }

        dc_initialized = false;
        dc_thread_wake_and_join();
    }
}

/*******************************************************************************
 * @brief drive_ctrl_reg_cb - register a callback function for event cb's.
 *
 * @details A routine used to register for event callbacks during regular
 * operation.  These callbacks should be handled like ISRs in that the
 * processing done within them must be short (non-blocking) and should not call
 * any of the other functions in this API.
 *
 * @param cb - the callback function to register.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int drive_ctrl_reg_cb(drive_ctrl_cb_t cb)
{
    int rc = -1;

    if (!dc_initialized) {
        LOG_WRN("Not init'd");
    } else if (dc_callback_func != NULL) {
        LOG_WRN("Callback already registered");
    } else {
        dc_callback_func = cb;
        rc = 0;
    }

    return rc;
}

/*******************************************************************************
 * @brief drive_ctrl_dereg_cb - de-register a previously registered callback.
 *
 * @details A routine used to de-register a callback function.
 *
 * @param cb - the callback function to de-register.
 *
 * @return None.
 ******************************************************************************/

void drive_ctrl_dereg_cb(drive_ctrl_cb_t cb)
{
    if (!dc_initialized) {
        LOG_WRN("Not init'd");
    } else if (dc_callback_func != cb) {
        LOG_WRN("Callback not registered");
    } else {
        dc_callback_func = NULL;
    }
}

/*******************************************************************************
 * @brief drive_ctrl_move - request to move a distance at an angle.
 *
 * @details A routine used to request a move from the current position.  The
 * move is based on an angle and a distance from the current position.
 *
 * @param angle - the given direction (angle from current position) to move.
 * @param distance - the given distance to move in meters.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int drive_ctrl_move(int16_t angle, float distance)
{
    int rc = -1;

    if (!dc_initialized) {
        LOG_WRN("Not init'd");
    } else if ((angle < DC_MOVE_ANGLE_DEG_MIN) || (angle > DC_MOVE_ANGLE_DEG_MAX) ||
                (distance < DC_MOVE_DIST_METERS_MIN) || (distance > DC_MOVE_DIST_METERS_MAX)) {
        LOG_WRN("Invalid params");
    } else if (dc_state != DC_STATE_NONE) {
        LOG_WRN("Already in motion"); 
    } else {
        dc_move_angle = angle;
        dc_move_distance = distance * 100.0f; // Convert to centimeters
        pthread_cond_signal(&dc_cond);
        rc = 0;
    }

    return rc;
}

/*******************************************************************************
 * @brief drive_ctrl_stop - halt any existing move requests.
 *
 * @details A routine used to request a move from the current position.  The
 * move is based on an angle and a distance from the current position.
 *
 * @return None.
 ******************************************************************************/

void drive_ctrl_stop(void)
{
    if (!dc_initialized) {
        LOG_WRN("Not init'd");
    } else if (dc_state == DC_STATE_NONE) {
        LOG_WRN("Already stopped"); 
    } else {
        force_stop();
    }
}

/*******************************************************************************
 * @brief drive_ctrl_get_state - retrieve the current module state.
 *
 * @details A routine used to retrieve the current module state.
 *
 * @return Returns the current module state.
 ******************************************************************************/

dc_state_t drive_ctrl_get_state(void)
{
    return dc_state;
}

/*****************************************************************************/

static void *dc_thread_loop(void *arg)
{
    (void)arg;  // Not used

    pthread_mutex_lock(&dc_lock);

    dc_thread_exit = false;

    do { // Loop until requested to exit

        // Wait for a move request
        pthread_cond_wait(&dc_cond, &dc_lock);

        do { // Loop waiting for the move to complete

            // No operation or wait if exiting
            if (dc_thread_exit) {
                break;
            }

            if (start_next_move_operation() != 0) {
                LOG_ERR("Next operaton failed");
                update_state(DC_STATE_FAILURE);
            } else {
                // Wait for a move operation to complete
                pthread_cond_wait(&dc_move_cond, &dc_move_lock);
                if (dc_operation_failure) {
                    LOG_ERR("Operaton failed");
                    update_state(DC_STATE_FAILURE);
                }
            }

        } while (!dc_thread_exit && is_running_state());

    } while (!dc_thread_exit);

    pthread_mutex_unlock(&dc_lock);

    return NULL;
}

static int dc_thread_setup_and_start(void)
{
    static pthread_attr_t thread_attr = {0};

    int ret = 0;
    int rc = -1;

    if ((ret = pthread_attr_init(&thread_attr)) != 0) {
        LOG_WRN("Failed attr init: %d", ret);
#if defined(ZEPHYR_BUILD)
    } else if ((ret = pthread_attr_setstack(&thread_attr, dc_thread_stack, DC_THREAD_STACK_SIZE)) != 0) {
        LOG_WRN("Failed attr stack: %d", ret);
#endif
    } else if ((ret = pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_JOINABLE)) != 0) {
        LOG_WRN("Failed attr joinable: %d", ret);
    } else if ((ret = pthread_create(&dc_thread, &thread_attr, dc_thread_loop, NULL)) != 0) {
        LOG_WRN("Failed to start: %d", ret);
    } else {
#if !defined(MAC_BUILD)
        // For some reason MacOS does not correctly support this interface
        (void)pthread_setname_np(dc_thread, "dc");
#endif
        rc = 0;
    }

    return rc;
}

static void dc_thread_wake_and_join(void)
{
    // Only attempt to join if the thread is running (!exit)
    if (dc_thread_exit == false) {
        // Set the exit flag for the thread
        dc_thread_exit = true;
        // Signal to wake up the current delay
        if (dc_state != DC_STATE_NONE) {
            pthread_mutex_lock(&dc_move_lock);
            pthread_cond_signal(&dc_move_cond);
            pthread_mutex_unlock(&dc_move_lock);
        } else {
            pthread_mutex_lock(&dc_lock);
            pthread_cond_signal(&dc_cond);
            pthread_mutex_unlock(&dc_lock);
        }
        // Join the thread
        int ret = pthread_join(dc_thread, NULL);
        if (ret != 0) {
            LOG_INF("Join failed %d", ret);
        }
    }
}

static float determine_move_distance(void)
{
    float cm = -1.0f;

    if (dc_move_distance < 0.0f) {
        // Just return the default value
    } else if (dc_move_distance > MAX_FORWARD_MOVEMENT_SEGMENT_CM) {
        cm = MAX_FORWARD_MOVEMENT_SEGMENT_CM;
        dc_move_distance -= cm;
    } else {
        cm = dc_move_distance;
        dc_move_distance = -1.0f;
    }

    return cm;
}

static void force_stop(void)
{
    update_state(DC_STATE_STOPPING);
    pthread_cond_signal(&dc_cond);
}

static bool is_path_clear(void)
{
    bool clear = true;
    int idx = 0;

    if (dc_scan_data.count >= MAPPER_MAX_POINTS) {
        // Something is wrong with the data, so just report a blockage
        clear = false;
    } else {
        while (idx < dc_scan_data.count) {
            // Check for measurement flagged scenarios
            if (dc_scan_data.meters[idx] < 0.0f) {
                if (dc_scan_data.meters[idx] != MAPPER_METERS_TOO_FAR) {
                    // All other cases are assumed to imply a blockage
                    clear = false;
                    break;
                } else {
                    // Object "too far" imply nothing to worry about from this angle
                }
            // Check the case straight-ahead, since the triangle math won't work
            } else if (dc_scan_data.angle[idx] == 0) {
                if (dc_scan_data.meters[idx] > MAX_FORWARD_MOVEMENT_SEGMENT_CM) {
                    // Nothing to block our next movement
                } else {
                    // Something is within our movement distance
                    clear = false;
                    break;
                }
            // Use a triangle method to determine if something would block our movement
            } else {
                // We have an angle and a distance (hypotenuse), determine the other sides
                // of a right triangle with this info to determine if something would
                // block our next forward movement.
                //
                // If our angle is "a" and the hypotenuse is "x":
                //
                // The adjacent side, "y", would be found via:
                //   cos(a) = (y / x)  or  y = cos(a) * x
                // The opposite side, "z", would be found via:
                //   sin(a) = (z / x)  or  y = sin(a) * x
                //
                // "y" would be the distance to the right or left of us.
                // "z" would be the distance ahead of us.

                float x = dc_scan_data.meters[idx] * 100; // Convert to centimeters
                float cosa = cosf(dc_scan_data.angle[idx]);
                float sina = sinf(dc_scan_data.angle[idx]);
                float y = cosa * x;
                float z = sina * x;

                // If the distance in front of us is < next movement
                if (z < MAX_FORWARD_MOVEMENT_SEGMENT_CM) {
                    // If the sideway distance is < safe distance, then blockage
                    if (y < SAFE_SIDE_DISTANCE_CM) {
                        clear = false;
                        break;
                    }
                }
            }

            idx ++;
        }
    }

    return clear;
}

static bool is_running_state(void)
{
    bool running = false;

    if ((dc_state == DC_STATE_TURNING)
            || (dc_state == DC_STATE_MOVING)
            || (dc_state == DC_STATE_SCANNING)
            || (dc_state == DC_STATE_STOPPING)) {
        running = true;
    }

    return running;
}

static void mapper_cb(const mapper_event_t *event)
{
	LOG_DBG("Mapper CB, state %d", event->state);

    switch (event->state) {
    case MAP_STATE_RUNNING:
        // Nothing to do
        break;
    case MAP_STATE_COMPLETE:
        pthread_cond_signal(&dc_move_cond);
        break;
    case MAP_STATE_FAILURE:
    default:
        dc_operation_failure = true;
        pthread_cond_signal(&dc_move_cond);
        break;
    }
}

static int start_next_move_operation(void)
{
    int rc = -1;

    dc_operation_failure = false;

    switch (dc_state)
    {
    case DC_STATE_NONE:
        // The first operation is always the turn to the requested direction
        if (tracks_pivot(dc_move_angle) == 0) {
            update_state(DC_STATE_TURNING);
            rc = 0;
        }
        break;
    case DC_STATE_TURNING:
        // After turning, scan for objects immediately in front of us
        if (mapper_start_sweep(MAP_TYPE_SHORT, &dc_scan_data) == 0) {
            update_state(DC_STATE_SCANNING);
            rc = 0;
        }
        break;
    case DC_STATE_SCANNING:
        // Completed a scan, so verify there is nothing blocking our progress forward
        if (!is_path_clear()) {
            // The movement is blocked
            LOG_WRN("Forward movement is blocked");
            update_state(DC_STATE_FAILURE);
            rc = 0;
        } else {
            // Start next movement
            float cm = determine_move_distance();
            if (cm < 0) {
                // The movement is complete
                update_state(DC_STATE_COMPLETE);
                rc = 0;
            } else if (tracks_move(cm) == 0) {
                update_state(DC_STATE_MOVING);
                rc = 0;
            }
        }
        break;
    case DC_STATE_MOVING:
        // Finished the move, so if there is still a distance to move, scan
        // for objects immediately in front of us
        if (dc_move_distance <= 0.0f) {
            // The movement is complete
            update_state(DC_STATE_COMPLETE);
            rc = 0;
        } else if (mapper_start_sweep(MAP_TYPE_SHORT, &dc_scan_data) == 0) {
            update_state(DC_STATE_SCANNING);
            rc = 0;
        }
        break;
    default:
        // Should not get here, so just return a failure
        break;
    }

    return rc;
}

static void tracks_cb(const tracks_event_t *event)
{
	LOG_DBG("Tracks CB, moving %d", event->moving);

    if (event->syserr != TRKS_ERR_NONE) {
        dc_operation_failure = true;
        pthread_cond_signal(&dc_move_cond);
    } else if (event->moving == true) {
        // Nothing to do
    } else {
        // Move complete
        pthread_cond_signal(&dc_move_cond);
    }
}

static void update_state(dc_state_t new_state)
{
    if (new_state != dc_state) {
        pthread_mutex_lock(&dc_state_lock);
        
        dc_state = new_state;
        dc_event_t event;
        event.state = dc_state;

        if (dc_callback_func != NULL) {
            dc_callback_func(&event);
        }

        pthread_mutex_unlock(&dc_state_lock);
    }
}
