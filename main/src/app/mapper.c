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

#include "mapper.h"

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "lidar.h"
#include "logger.h"
#include "os_al.h"
#include "panner.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

//! This mutex is used by the pthread and the timed condition
static pthread_mutex_t mapper_lock = PTHREAD_MUTEX_INITIALIZER;
//! This condition is used polling the button
static pthread_cond_t mapper_cond = PTHREAD_COND_INITIALIZER;
//! The application thread
static pthread_t mapper_thread = {0};

#if defined(ZEPHYR_BUILD)
#define MAPPER_THREAD_STACK_SIZE (1000)
static K_THREAD_STACK_DEFINE(mapper_thread_stack, MAPPER_THREAD_STACK_SIZE);
#endif

//! Flag used to force the application thread to exit
static bool mapper_thread_exit = false;
//! Flag used to know if the module has been initialized or not
static bool mapper_initialized = false;

#define MAPPER_CB_FUNC_MAX_NUM (3)
static mapper_cb_t mapper_callback_func[MAPPER_CB_FUNC_MAX_NUM] = {0};
static mapper_state_t mapper_state = MAP_STATE_NONE;
static bool mapper_abort = false;

static int16_t panner_limit_min = 0;
static int16_t panner_limit_max = 0;
#define MAPPER_EXPECTED_SHORT_LIMIT_DEG (25)
static int16_t panner_short_min = 0;
static int16_t panner_short_max = 0;

static mapper_type_t mapper_sweep_type = MAP_TYPE_FULL;
static sweep_data_t *mapper_sweep_data;
static uint32_t mapper_sweep_cnt = 0;

static float convert_to_meters_status(lidar_status_t status);
// Return not used
static void *mapper_thread_loop(void *arg);
// Returns 0 on success, otherwise -1
static int mapper_thread_setup_and_start(void);
static void mapper_thread_wake_and_join(void);
static void perform_sweep(void);
static void update_state(mapper_state_t new_state);

/*****************************************************************************
 * @brief mapper_init - initialize the mapper module
 * 
 * @details Performs setup and starts a thread to handle the module
 * 
 * @return Returns 0 on success, otherwise -1
 *****************************************************************************/

int mapper_init(void)
{
    int rc = -1;

    if (mapper_initialized == true) {
        LOG_INF("Already started");
    } else if (lidar_init() != 0) {
        LOG_INF("Lidar init failed");
    } else if (panner_init() != 0) {
        LOG_INF("Panner init failed");
        lidar_exit();
    } else if (mapper_thread_setup_and_start() != 0) {
        LOG_INF("Thread startup failed");
        panner_exit();
        lidar_exit();
    } else {
        panner_get_limits(&panner_limit_min, &panner_limit_max);
        panner_short_min = (-MAPPER_EXPECTED_SHORT_LIMIT_DEG);
        panner_short_max = MAPPER_EXPECTED_SHORT_LIMIT_DEG;
        if (panner_limit_min > (-MAPPER_EXPECTED_SHORT_LIMIT_DEG)) {
            panner_short_min = panner_limit_min;
        }
        if (panner_limit_max < MAPPER_EXPECTED_SHORT_LIMIT_DEG) {
            panner_short_max = panner_limit_max;
        }

        mapper_sweep_data = NULL;
        mapper_state = MAP_STATE_NONE;
        for (int i = 0; i < MAPPER_CB_FUNC_MAX_NUM; i++) {
            mapper_callback_func[i] = NULL;
        }
        mapper_initialized = true;
        rc = 0;
    }

    return rc;
}

/*****************************************************************************
 * @brief mapper_exit - force the mapper module to exit
 * 
 * @details Forces the mapper module to exit and does not return until the
 * module has exited.
 *****************************************************************************/

void mapper_exit(void)
{
    if (mapper_initialized == false) {
        LOG_INF("Already exited");
    } else {
        mapper_initialized = false;
        mapper_thread_wake_and_join();
        panner_exit();
        lidar_exit();
    }
}

/*******************************************************************************
 * @brief mapper_reg_cb - register a callback function for event cb's.
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

int mapper_reg_cb(mapper_cb_t cb)
{
    int rc = -1;
    int i;

    do { // Using do/while(0) as a quick escape mechanism

        if (!mapper_initialized) {
            LOG_WRN("Not init'd");
            break;
        }

        // Check if the CB is already registered
        for (i = 0; i < MAPPER_CB_FUNC_MAX_NUM; i++) {
            if (mapper_callback_func[i] == cb) {
                break;
            }
        }
        if (i < MAPPER_CB_FUNC_MAX_NUM) {
            LOG_WRN("Already registered");
            break;
        }

        // Find an empty slot
        for (i = 0; i < MAPPER_CB_FUNC_MAX_NUM; i++) {
            if (mapper_callback_func[i] == NULL) {
                mapper_callback_func[i] = cb;
                rc = 0;
                break;
            }
        }
        if (i >= MAPPER_CB_FUNC_MAX_NUM) {
            LOG_WRN("No space");
        }

    } while (0);

    return rc;
}

/*******************************************************************************
 * @brief mapper_dereg_cb - de-register a previously registered callback.
 *
 * @details A routine used to de-register a callback function.
 *
 * @return None.
 ******************************************************************************/

void mapper_dereg_cb(mapper_cb_t cb)
{
    if (!mapper_initialized) {
        LOG_WRN("Not init'd");
    } else {
        int i = 0;
        for (; i < MAPPER_CB_FUNC_MAX_NUM; i++) {
            if (mapper_callback_func[i] == cb) {
                mapper_callback_func[i] = NULL;
                break;
            }
        }

        if (i >= MAPPER_CB_FUNC_MAX_NUM) {
            LOG_WRN("Callback not registered");
        }
    }
}

/*******************************************************************************
 * @brief mapper_get_state - retrieve the current module state.
 *
 * @details A routine used to retrieve the current module state.
 *
 * @return Returns the current module state.
 ******************************************************************************/

mapper_state_t mapper_get_state(void)
{
    return mapper_state;
}

/*****************************************************************************
 * @brief mapper_start_sweep - trigger a sweep to begin
 * 
 * @details Capture data on the space in front of the LIDAR sensor by sweeping
 * the full extent of the servo and capturing distances via the LIDAR sensor.
 * 
 * @return Returns 0 on success, otherwise -1
 *****************************************************************************/

int mapper_start_sweep(mapper_type_t type, sweep_data_t *sweep_data)
{
    int rc = -1;

    if (mapper_initialized == false) {
        LOG_INF("Not init'd");
    } else if (sweep_data == NULL) {
        LOG_WRN("No data");
    } else if ((type != MAP_TYPE_FULL) && (type != MAP_TYPE_SHORT)) {
        LOG_WRN("Invalid type");
    } else {
        mapper_sweep_type = type;
        mapper_sweep_data = sweep_data;
        pthread_cond_signal(&mapper_cond);
        rc = 0;
    }
    
    return rc;
}

/*****************************************************************************
 * @brief mapper_abort_sweep - abort an in-progress sweep
 * 
 * @details Stop/abort an in-progress sweep.
 *****************************************************************************/

void mapper_abort_sweep(void)
{
    if (mapper_state == MAP_STATE_RUNNING) {
        mapper_abort = true;
    }
}

/*****************************************************************************/

static float convert_to_meters_status(lidar_status_t status)
{
    float meters = MAPPER_METERS_TOO_DARK;

    switch (status) {
    case LIDAR_TOO_DARK:
        meters = MAPPER_METERS_TOO_DARK;
        break;
    case LIDAR_TOO_BRIGHT:
        meters = MAPPER_METERS_TOO_BRIGHT;
        break;
    case LIDAR_TOO_FAR:
        meters = MAPPER_METERS_TOO_FAR;
        break;
    case LIDAR_TOO_CLOSE:
        meters = MAPPER_METERS_TOO_CLOSE;
        break;
    default:
        // Just return the default
        break;
    }

    return meters;
}

static void *mapper_thread_loop(void *arg)
{
    (void)arg;  // Not used

    pthread_mutex_lock(&mapper_lock);

    mapper_thread_exit = false;

    do {  // Loop until requested to exit

        // Wait for a signal to start the sweep
        pthread_cond_wait(&mapper_cond, &mapper_lock);

        if (mapper_sweep_data == NULL) {
            update_state(MAP_STATE_FAILURE);
        } else {
            perform_sweep();
            mapper_sweep_data = NULL;
        }

    } while (!mapper_thread_exit);

    pthread_mutex_unlock(&mapper_lock);

    return NULL;
}

static int mapper_thread_setup_and_start(void)
{
    static pthread_attr_t thread_attr = {0};

    int ret = 0;
    int rc = -1;

    if ((ret = pthread_attr_init(&thread_attr)) != 0) {
        LOG_WRN("Failed attr init: %d", ret);
#if defined(ZEPHYR_BUILD)
    } else if ((ret = pthread_attr_setstack(&thread_attr, mapper_thread_stack, MAPPER_THREAD_STACK_SIZE)) != 0) {
        LOG_WRN("Failed attr stack: %d", ret);
#endif
    } else if ((ret = pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_JOINABLE)) != 0) {
        LOG_WRN("Failed attr joinable: %d", ret);
    } else if ((ret = pthread_create(&mapper_thread, &thread_attr, mapper_thread_loop, NULL)) != 0) {
        LOG_WRN("Failed to start: %d", ret);
    } else {
#if !defined(MAC_BUILD)
        // For some reason MacOS does not correctly support this interface
        (void)pthread_setname_np(mapper_thread, "mapper");
#endif
        rc = 0;
    }

    return rc;
}

static void mapper_thread_wake_and_join(void)
{
    // Only attempt to join if the thread is running (!exit)
    if (mapper_thread_exit == false) {
        // Set the exit flag for the thread
        mapper_thread_exit = true;
        // Signal to wake up the current delay
        pthread_mutex_lock(&mapper_lock);
        pthread_cond_signal(&mapper_cond);
        pthread_mutex_unlock(&mapper_lock);
        // Join the thread
        int ret = pthread_join(mapper_thread, NULL);
        if (ret != 0) {
            LOG_INF("Join failed %d", ret);
        }
    }
}

static void perform_sweep(void)
{
    lidar_status_t lidar_ret = LIDAR_SUCCESS;
    float meters;
    int16_t pos = (mapper_sweep_type == MAP_TYPE_SHORT) ? panner_short_min : panner_limit_min;
    int16_t pos_max = (mapper_sweep_type == MAP_TYPE_SHORT) ? panner_short_max : panner_limit_max;
    int16_t inc = MAPPER_SWEEP_INCREMENT_DEGREES;

    mapper_abort = false;
    update_state(MAP_STATE_RUNNING);

    LOG_INF("Start sweep");
    uint32_t start = os_al_get_ms32();

    mapper_sweep_cnt ++;

    // Enable the servo power
    panner_enable(true);

    mapper_sweep_data->count = 0;

    while ((pos <= pos_max) && !mapper_abort) {
        if (panner_set_position(pos) != 0) {
            LOG_INF("Panner move failed");
            break;
        }

        // Allow the sensor to update the average
        usleep(10000);

        lidar_ret = lidar_get_distance(&meters);
        if (lidar_ret == LIDAR_FAILURE) {
            LOG_INF("Lidar read failed");
            break;
        } else if (lidar_ret != LIDAR_SUCCESS) {
            meters = convert_to_meters_status(lidar_ret);
        }

        mapper_sweep_data->angle[mapper_sweep_data->count] = pos;
        mapper_sweep_data->meters[mapper_sweep_data->count] = meters;
        mapper_sweep_data->count ++;

        pos += inc;
    }

    // Always leave with the sensor forward, look better that way :)
    panner_set_position(0);

    // Disable the servo power
    panner_enable(false);

    uint32_t delta_ms = os_al_get_ms32() - start;
    LOG_INF("Sweep time = %u ms", delta_ms);

    update_state(MAP_STATE_COMPLETE);
}

static void update_state(mapper_state_t new_state)
{
    if (new_state != mapper_state) {
        mapper_state = new_state;

        mapper_event_t event;
        event.state = mapper_state;

        for (int i = 0; i < MAPPER_CB_FUNC_MAX_NUM; i++) {
            if (mapper_callback_func[i] != NULL) {
                mapper_callback_func[i](&event);
            }
        }
    }
}
