/*****************************************************************************
 * roamer_app.h - header file for the roamer application
 * 
 * This application will:
 * a) Perform a mapping scan
 * b) Turn and move half-way to the fartherest point
 * c) Perform a mapping scan
 * d) Perform steps "b" and "c" again, then stop
 *
 *****************************************************************************/

#include "roamer_app.h"

#include <stdint.h>
#include <unistd.h>

#include "drive_ctrl.h"
#include "logger.h"
#include "mapper.h"
#include "rgb_led.h"
#include "start_stop.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

#define INDICATOR_BLINK_PERIOD_MS (1000)
#define INDICATOR_BLINK_ON_MS (200)
#define INDICATOR_BLINK_OFF_MS (INDICATOR_BLINK_PERIOD_MS - INDICATOR_BLINK_ON_MS)

#define ROAMER_MAX_MOVE_METERS (2)

typedef enum {
    STATE_WAIT_FOR_START = 0,
    STATE_PERFORM_SCAN,
    STATE_PERFORM_MOVE,
    STATE_COMPLETE,
    STATE_STOPPING,
    STATE_ERROR,
    STATE_NUM_OF // Must be last
} roamer_app_state_t;

typedef enum {
    EVENT_ENTER_STATE = 0,
    EVENT_START,
    EVENT_STOP,
    EVENT_SCANNING,
    EVENT_SCAN_COMPLETE,
    EVENT_MOVING,
    EVENT_MOVE_COMPLETE,
    EVENT_ERROR,
    EVENT_NUM_OF // Must be last
} roamer_app_event_t;

static roamer_app_state_t roamer_app_state = STATE_WAIT_FOR_START;

static int roamer_app_indicator_hndl = 0;

#define ROAMER_SWEEP_DATA_FRAMES (3)
static sweep_data_t roamer_sweep_data[ROAMER_SWEEP_DATA_FRAMES] = {0};
static uint8_t roamer_sweep_data_idx = 0;

// Returns 0 on success, otherwise -1
static int determine_move(int16_t *angle, float *distance);
static void drive_ctrl_cb(const dc_event_t *event);
// Returns a string representation of the event
static const char* get_event_name(roamer_app_event_t event);
// Returns a string representation of the state
static const char* get_state_name(roamer_app_state_t state);
static void handle_event(roamer_app_event_t event);
// Returns the state to move to
static roamer_app_state_t handle_complete(roamer_app_event_t event);
// Returns the state to move to
static roamer_app_state_t handle_error(roamer_app_event_t event);
// Returns the state to move to
static roamer_app_state_t handle_move(roamer_app_event_t event);
// Returns the state to move to
static roamer_app_state_t handle_scan(roamer_app_event_t event);
// Returns the state to move to
static roamer_app_state_t handle_stopping(roamer_app_event_t event);
// Returns the state to move to
static roamer_app_state_t handle_wait_for_start(roamer_app_event_t event);
static void mapper_cb(const mapper_event_t *event);
static void should_not_occur(roamer_app_event_t event);
static void start_stop_cb(const start_stop_event_t *event);

/*******************************************************************************
 * @brief start_stop_init - initialize the test application.
 *
 * @details A routine used to initialize the test application.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int roamer_app_start(void)
{
    int rc = -1;

    // We don't fail if the RGB LED is not functional
    roamer_app_indicator_hndl = rgb_led_init();

    // Must init mapper before drive ctrl as drive ctrl requires it to be init'd
	if (mapper_init() != 0) {
		LOG_ERR("Tracks init failed");
	} else if (mapper_reg_cb(mapper_cb) != 0) {
		LOG_ERR("Tracks cb reg failed");
	} else if (drive_ctrl_init() != 0) {
		LOG_ERR("Drive ctrl init failed");
	} else if (drive_ctrl_reg_cb(drive_ctrl_cb) != 0) {
		LOG_ERR("Drive ctrl cb reg failed");
	} else if (start_stop_init() != 0) {
		LOG_ERR("Start/stop init failed");
	} else if (start_stop_reg_cb(start_stop_cb) != 0) {
		LOG_ERR("Start/stop cb reg failed");
	} else {
        roamer_app_state = STATE_WAIT_FOR_START;
        handle_event(EVENT_ENTER_STATE);
        rc = 0;
	}

    return rc;
}

/*******************************************************************************
 * @brief roamer_app_exit - de-initialize the roamer application.
 *
 * @details A routine used to de-initialize the roamer application.
 *
 * @return None.
 ******************************************************************************/

void roamer_app_exit(void)
{
    start_stop_dereg_cb(start_stop_cb);
    start_stop_exit();
    drive_ctrl_dereg_cb(drive_ctrl_cb);
    drive_ctrl_exit();
    mapper_dereg_cb(mapper_cb);
    mapper_exit();

    if (roamer_app_indicator_hndl >= 0) {
        rgb_led_exit(roamer_app_indicator_hndl);
    }
}

/*****************************************************************************/

// We assume the passed in params are non-null
static int determine_move(int16_t *angle, float *distance)
{
    uint8_t sweep_idx = roamer_sweep_data_idx - 1;
    float max_found = 0.0f;
    int max_idx = -1;
    int rc = -1;

    if (sweep_idx >= ROAMER_SWEEP_DATA_FRAMES) {
        LOG_WRN("Invalid sweep index");
    } else if (roamer_sweep_data[sweep_idx].count >= MAPPER_MAX_POINTS) {
        LOG_WRN("Invalid sweep data");
    } else {
        // Find the longest distance
        for (int i = 0; i < roamer_sweep_data[sweep_idx].count; i++) {
            if (roamer_sweep_data[sweep_idx].meters[i] == MAPPER_METERS_TOO_FAR) {
                // Found "too far" so move forward with this entry
                *angle = roamer_sweep_data[sweep_idx].angle[i];
                // Assume the maximum move distance
                *distance = (float) ROAMER_MAX_MOVE_METERS;
                max_idx = -1;
                break;
            } else if (roamer_sweep_data[sweep_idx].meters[i] > max_found) {
                max_found = roamer_sweep_data[sweep_idx].meters[i];
                max_idx = i;
            }
        }

        if (max_idx > -1) {
            *angle = roamer_sweep_data[sweep_idx].angle[max_idx];
            // Use only half the distance
            max_found /= 2;
            // Limit the movement further
            if (max_found > (float) ROAMER_MAX_MOVE_METERS) {
                max_found = (float) ROAMER_MAX_MOVE_METERS;
            }
            *distance = max_found;
        }

        rc = 0;
    }

    return rc;
}

static void drive_ctrl_cb(const dc_event_t *event)
{
    static bool running = false;

	LOG_DBG("Drive Ctrl CB %d", event->state);

    if (event->state == DC_STATE_FAILURE) {
        handle_event(EVENT_ERROR);
        running = false;
    } else if (!running) {
        if ((event->state == DC_STATE_TURNING) || (event->state == DC_STATE_MOVING) || (event->state == DC_STATE_SCANNING)) {
            handle_event(EVENT_MOVING);
            running = true;
        }
    } else if (running) {
        if ((event->state == DC_STATE_COMPLETE) || (event->state == DC_STATE_STOPPED)) {
            handle_event(EVENT_MOVE_COMPLETE);
            running = false;
        }
    }
}

static const char* get_event_name(roamer_app_event_t event)
{
    static const char* event_names[] = { "ENTER", "START", "STOP", "SCANNING", "SCAN-CMPL", "MOVING", "MOVE-CMPL", "ERROR", "UNK" };
    uint8_t idx = (uint8_t)event;

    if (idx > EVENT_NUM_OF) {
        idx = EVENT_NUM_OF;
    }

    return (event_names[idx]);
}

static const char* get_state_name(roamer_app_state_t state)
{
    static const char* state_names[] = { "WAIT", "SCAN", "MOVE", "CMPL", "ERROR", "UNK" };
    uint8_t idx = (uint8_t)state;

    if (idx > STATE_NUM_OF) {
        idx = STATE_NUM_OF;
    }

    return (state_names[idx]);
}

static void handle_event(roamer_app_event_t event)
{
    // Handle stop immediately in all cases
    if (event == EVENT_STOP) {
        if (roamer_app_state == STATE_PERFORM_SCAN) {
            mapper_abort_sweep();
            roamer_app_state = STATE_STOPPING;
        } else if (roamer_app_state == STATE_PERFORM_MOVE) {
            drive_ctrl_stop();
            roamer_app_state = STATE_STOPPING;
        } else {
            roamer_app_state = STATE_COMPLETE;
        }
    } else {
        roamer_app_state_t state = roamer_app_state;
        roamer_app_event_t new_event = event;

        do {
            // Delay a 250ms between operations
            usleep(250000);

            // Save the state, if it changed below
            roamer_app_state = state;

            LOG_DBG("Handling event %s from state %s", get_event_name(new_event), get_state_name(roamer_app_state));

            switch (roamer_app_state) {
            case STATE_WAIT_FOR_START:
                state = handle_wait_for_start(new_event);
                break;
            case STATE_PERFORM_SCAN:
                state = handle_scan(new_event);
                break;
            case STATE_PERFORM_MOVE:
                state = handle_move(new_event);
                break;
            case STATE_STOPPING:
                state = handle_stopping(new_event);
                break;
            case STATE_COMPLETE:
                state = handle_complete(new_event);
                break;
            case STATE_ERROR:
                state = handle_error(new_event);
                break;
            default:
                LOG_WRN("Invalid state %d", roamer_app_state);
                break;
            }

            LOG_DBG("New state %s", get_state_name(state));

            // All additional state transitions will use EVENT_ENTER_STATE
            new_event = EVENT_ENTER_STATE;

        } while (state != roamer_app_state);
    }
}

static roamer_app_state_t handle_complete(roamer_app_event_t event)
{
    roamer_app_state_t state = roamer_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        if (roamer_app_indicator_hndl >= 0) {
            rgb_led_set_color(roamer_app_indicator_hndl, &RGB_COLOR_YELLOW);
            rgb_led_set_blink(roamer_app_indicator_hndl, INDICATOR_BLINK_ON_MS, INDICATOR_BLINK_OFF_MS);
        }
        break;
    case EVENT_MOVING:
    case EVENT_MOVE_COMPLETE:
    case EVENT_SCANNING:
    case EVENT_SCAN_COMPLETE:
    case EVENT_START:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static roamer_app_state_t handle_error(roamer_app_event_t event)
{
    roamer_app_state_t state = roamer_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        if (roamer_app_indicator_hndl >= 0) {
            rgb_led_set_color(roamer_app_indicator_hndl, &RGB_COLOR_RED);
            rgb_led_set_blink(roamer_app_indicator_hndl, INDICATOR_BLINK_ON_MS, INDICATOR_BLINK_OFF_MS);
        }
        break;
    case EVENT_MOVING:
    case EVENT_MOVE_COMPLETE:
    case EVENT_SCANNING:
    case EVENT_SCAN_COMPLETE:
    case EVENT_START:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static roamer_app_state_t handle_move(roamer_app_event_t event)
{
    roamer_app_state_t state = roamer_app_state;
    float distance = 0.0f;
    int16_t angle = 0;

    switch (event) {
    case EVENT_ENTER_STATE:
        if (determine_move(&angle, &distance) != 0) {
            LOG_ERR("Get move params failed");
            state = STATE_ERROR;
        } else if (drive_ctrl_move(angle, distance) != 0) {
            LOG_ERR("Move failed");
            state = STATE_ERROR;
        } else {
            if (roamer_app_indicator_hndl >= 0) {
                rgb_led_set_color(roamer_app_indicator_hndl, &RGB_COLOR_GREEN);
                rgb_led_set_blink(roamer_app_indicator_hndl, INDICATOR_BLINK_ON_MS, INDICATOR_BLINK_OFF_MS);
            }
        }
        break;
    case EVENT_MOVING:
        // Nothing to do
        break;
    case EVENT_MOVE_COMPLETE:
        state = STATE_PERFORM_SCAN;
        break;
    case EVENT_SCANNING:
    case EVENT_SCAN_COMPLETE:
        // These can occur, but it is normal since the mapper is being used during moves
        break;
    case EVENT_START:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static roamer_app_state_t handle_scan(roamer_app_event_t event)
{
    roamer_app_state_t state = roamer_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        if (roamer_sweep_data_idx >= ROAMER_SWEEP_DATA_FRAMES) {
            LOG_ERR("No sweep frames available");
            state = STATE_ERROR;
        } else {
            mapper_start_sweep(MAP_TYPE_FULL, &roamer_sweep_data[roamer_sweep_data_idx]);
            roamer_sweep_data_idx ++;

            if (roamer_app_indicator_hndl >= 0) {
                rgb_led_set_color(roamer_app_indicator_hndl, &RGB_COLOR_BLUE);
                rgb_led_set_blink(roamer_app_indicator_hndl, INDICATOR_BLINK_ON_MS, INDICATOR_BLINK_OFF_MS);
            }
        }
        break;
    case EVENT_SCANNING:
        // Nothing to do
        break;
    case EVENT_SCAN_COMPLETE:
        if (roamer_sweep_data_idx >= ROAMER_SWEEP_DATA_FRAMES) {
            state = STATE_COMPLETE;
        } else {
            state = STATE_PERFORM_MOVE;
        }
        break;
    case EVENT_MOVING:
    case EVENT_MOVE_COMPLETE:
    case EVENT_START:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static roamer_app_state_t handle_stopping(roamer_app_event_t event)
{
    roamer_app_state_t state = roamer_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        if (roamer_app_indicator_hndl >= 0) {
            rgb_led_set_color(roamer_app_indicator_hndl, &RGB_COLOR_RED);
            rgb_led_set_blink(roamer_app_indicator_hndl, INDICATOR_BLINK_ON_MS, INDICATOR_BLINK_OFF_MS);
        }
        break;
    case EVENT_MOVE_COMPLETE:
    case EVENT_SCAN_COMPLETE:
        state = STATE_COMPLETE;
        break;
    case EVENT_MOVING:
    case EVENT_SCANNING:
    case EVENT_START:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static roamer_app_state_t handle_wait_for_start(roamer_app_event_t event)
{
    roamer_app_state_t state = roamer_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        // Reset the frame index
        roamer_sweep_data_idx = 0;

        // Just ensure we are waiting for a start event
        start_stop_auto_stop();

        if (roamer_app_indicator_hndl >= 0) {
            rgb_led_set_color(roamer_app_indicator_hndl, &RGB_OFF);
            rgb_led_set_blink(roamer_app_indicator_hndl, 0, 0);
        }
        break;
    case EVENT_START:
        // Always start with a scan
        state = STATE_PERFORM_SCAN;
        break;
    case EVENT_SCANNING:
    case EVENT_SCAN_COMPLETE:
    case EVENT_MOVING:
    case EVENT_MOVE_COMPLETE:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static void mapper_cb(const mapper_event_t *event)
{
	LOG_DBG("Mapper CB %d", event->state);

    if (event->state == MAP_STATE_RUNNING) {
        handle_event(EVENT_START);
    } else if (event->state == MAP_STATE_COMPLETE) {
        handle_event(EVENT_STOP);
    } else if (event->state == MAP_STATE_FAILURE) {
        handle_event(EVENT_ERROR);
    }
}

static void should_not_occur(roamer_app_event_t event)
{
    LOG_WRN("Should not occur: State:%s; Event:%s;", get_state_name(roamer_app_state), get_event_name(event));
}

static void start_stop_cb(const start_stop_event_t *event)
{
	LOG_DBG("Start/Stop CB %d", event->start_not_stop);

    if (event->start_not_stop == true) {
        handle_event(EVENT_START);
    } else {
        handle_event(EVENT_STOP);
    }
}
