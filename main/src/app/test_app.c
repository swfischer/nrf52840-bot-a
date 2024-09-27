/*****************************************************************************
 * test_app.c - source file for the tank track test application
 * 
 * This application will:
 * a) Wait for a start command (via start_stop.h)
 * b) Move forward 50cm
 * c) Turn CW 90 degrees
 * d) Move forward 10cm
 * e) Turn CCW 90 degrees
 * f) Move reverse 50cm
 * g) Stop and wait for the next start
 *
 *****************************************************************************/

#include "test_app.h"

#include <stdint.h>
#include <unistd.h>

//#include "lidar.h"
#include "logger.h"
#include "mapper.h"
//#include "panner.h"
#include "rgb_led.h"
#include "start_stop.h"
#include "tracks.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

#define INDICATOR_BLINK_PERIOD_MS (1000)
#define INDICATOR_BLINK_ON_MS (200)
#define INDICATOR_BLINK_OFF_MS (INDICATOR_BLINK_PERIOD_MS - INDICATOR_BLINK_ON_MS)

typedef enum {
    STATE_WAIT_FOR_START = 0,
    STATE_MOVE_FWD_10CM,
    STATE_MOVE_FWD_50CM,
    STATE_MOVE_REV_50CM,
    STATE_TURN_CCW_90DEG,
    STATE_TURN_CW_90DEG,
    STATE_NUM_OF // Must be last
} test_app_state_t;

typedef enum {
    EVENT_ENTER_STATE = 0,
    EVENT_START,
    EVENT_STOP,
    EVENT_MOVING,
    EVENT_MOVE_COMPLETE,
    EVENT_NUM_OF // Must be last
} test_app_event_t;

static test_app_state_t test_app_state = STATE_WAIT_FOR_START;
static test_app_state_t test_app_next_state[] =
{ STATE_MOVE_FWD_50CM
, STATE_TURN_CW_90DEG
, STATE_MOVE_FWD_10CM
, STATE_TURN_CCW_90DEG
, STATE_MOVE_REV_50CM
, STATE_WAIT_FOR_START
};
static uint8_t test_app_next_state_idx = 0;
static const uint8_t TEST_APP_NUM_NEXT_STATES = (sizeof(test_app_next_state) / sizeof(test_app_state_t));

static int test_app_indicator_hndl = 0;

static const char* get_event_name(test_app_event_t event);
static test_app_state_t get_next_state(void);
static const char* get_state_name(test_app_state_t state);
static void handle_event(test_app_event_t event);
static test_app_state_t handle_move_fwd_10cm(test_app_event_t event);
static test_app_state_t handle_move_fwd_50cm(test_app_event_t event);
static test_app_state_t handle_move_rev_50cm(test_app_event_t event);
static test_app_state_t handle_turn_ccw_90deg(test_app_event_t event);
static test_app_state_t handle_turn_cw_90deg(test_app_event_t event);
static test_app_state_t handle_wait_for_start(test_app_event_t event);
static void should_not_occur(test_app_event_t event);
static void start_stop_cb(const start_stop_event_t *event);
static void tracks_cb(const tracks_event_t *event);

/*******************************************************************************
 * @brief start_stop_init - initialize the test application.
 *
 * @details A routine used to initialize the test application.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int test_app_start(void)
{
    int rc = -1;

    // We don't fail if the RGB LED is not functional
    test_app_indicator_hndl = rgb_led_init();

(void)mapper_init();

	if (tracks_init() != 0) {
		LOG_ERR("Tracks init failed");
	} else if (tracks_reg_cb(tracks_cb) != 0) {
		LOG_ERR("Tracks cb reg failed");
	} else if (start_stop_init() != 0) {
		LOG_ERR("Start/stop init failed");
	} else if (start_stop_reg_cb(start_stop_cb) != 0) {
		LOG_ERR("Start/stop cb reg failed");
	} else {
        test_app_state = STATE_WAIT_FOR_START;
        handle_event(EVENT_ENTER_STATE);
        rc = 0;
	}

    return rc;
}

/*******************************************************************************
 * @brief test_app_exit - de-initialize the test application.
 *
 * @details A routine used to de-initialize the test application.
 *
 * @return None.
 ******************************************************************************/

void test_app_exit(void)
{
    start_stop_dereg_cb(start_stop_cb);
    start_stop_exit();
    tracks_dereg_cb(tracks_cb);
    tracks_exit();

    if (test_app_indicator_hndl >= 0) {
        rgb_led_exit(test_app_indicator_hndl);
    }
}

/*****************************************************************************/

static const char* get_event_name(test_app_event_t event)
{
    static const char* event_names[] = { "ENTER", "START", "STOP", "MOVING", "MOVE-CMPL", "UNK" };
    uint8_t idx = (uint8_t)event;

    if (idx > EVENT_NUM_OF) {
        idx = EVENT_NUM_OF;
    }

    return (event_names[idx]);
}

static test_app_state_t get_next_state(void)
{
    test_app_state_t state = STATE_WAIT_FOR_START;

    if (test_app_next_state_idx < TEST_APP_NUM_NEXT_STATES) {
        state = test_app_next_state[test_app_next_state_idx];
        test_app_next_state_idx++;
    }

    return state;
}

static const char* get_state_name(test_app_state_t state)
{
    static const char* state_names[] = { "WAIT", "FWD10", "FWD50", "REV50", "CCW90", "CW90", "UNK" };
    uint8_t idx = (uint8_t)state;

    if (idx > STATE_NUM_OF) {
        idx = STATE_NUM_OF;
    }

    return (state_names[idx]);
}

static void handle_event(test_app_event_t event)
{
    // Handle stop immediately in all cases
    if (event == EVENT_STOP) {
        tracks_stop();
    } else {
        test_app_state_t state = test_app_state;
        test_app_event_t new_event = event;

        do {
            // Delay a 250ms between operations
            usleep(250000);

            // Save the state, if it changed below
            test_app_state = state;

            LOG_DBG("Handling event %s from state %s", get_event_name(new_event), get_state_name(test_app_state));

            switch (test_app_state) {
            case STATE_WAIT_FOR_START:
                state = handle_wait_for_start(new_event);
                break;
            case STATE_MOVE_FWD_10CM:
                state = handle_move_fwd_10cm(new_event);
                break;
            case STATE_MOVE_FWD_50CM:
                state = handle_move_fwd_50cm(new_event);
                break;
            case STATE_MOVE_REV_50CM:
                state = handle_move_rev_50cm(new_event);
                break;
            case STATE_TURN_CCW_90DEG:
                state = handle_turn_ccw_90deg(new_event);
                break;
            case STATE_TURN_CW_90DEG:
                state = handle_turn_cw_90deg(new_event);
                break;
            default:
                LOG_WRN("Invalid state %d", test_app_state);
                break;
            }

            LOG_DBG("New state %s", get_state_name(state));

            // All additional state transitions will use EVENT_ENTER_STATE
            new_event = EVENT_ENTER_STATE;

        } while (state != test_app_state);
    }
}

static test_app_state_t handle_move_fwd_10cm(test_app_event_t event)
{
    test_app_state_t state = test_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        if (tracks_move(10) != 0) {
            LOG_ERR("Move failed");
            state = STATE_WAIT_FOR_START;
        } else {
            if (test_app_indicator_hndl >= 0) {
                rgb_led_set_color(test_app_indicator_hndl, &RGB_COLOR_GREEN);
                rgb_led_set_blink(test_app_indicator_hndl, INDICATOR_BLINK_ON_MS, INDICATOR_BLINK_OFF_MS);
            }
        }
        break;
    case EVENT_MOVING:
        // Nothing to do
        break;
    case EVENT_MOVE_COMPLETE:
        state = get_next_state();
        break;
    case EVENT_START:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static test_app_state_t handle_move_fwd_50cm(test_app_event_t event)
{
    test_app_state_t state = test_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        if (tracks_move(50) != 0) {
            LOG_ERR("Move failed");
            state = STATE_WAIT_FOR_START;
        } else {
            if (test_app_indicator_hndl >= 0) {
                rgb_led_set_color(test_app_indicator_hndl, &RGB_COLOR_GREEN);
                rgb_led_set_blink(test_app_indicator_hndl, INDICATOR_BLINK_ON_MS, INDICATOR_BLINK_OFF_MS);
            }
        }
        break;
    case EVENT_MOVING:
        // Nothing to do
        break;
    case EVENT_MOVE_COMPLETE:
        state = get_next_state();
        break;
    case EVENT_START:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static test_app_state_t handle_move_rev_50cm(test_app_event_t event)
{
    test_app_state_t state = test_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        if (tracks_move(-50) != 0) {
            LOG_ERR("Move failed");
            state = STATE_WAIT_FOR_START;
        } else {
            if (test_app_indicator_hndl >= 0) {
                rgb_led_set_color(test_app_indicator_hndl, &RGB_COLOR_GREEN);
                rgb_led_set_blink(test_app_indicator_hndl, INDICATOR_BLINK_ON_MS, INDICATOR_BLINK_OFF_MS);
            }
        }
        break;
    case EVENT_MOVING:
        // Nothing to do
        break;
    case EVENT_MOVE_COMPLETE:
        state = get_next_state();
        break;
    case EVENT_START:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static test_app_state_t handle_turn_ccw_90deg(test_app_event_t event)
{
    test_app_state_t state = test_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        if (tracks_pivot(-90) != 0) {
            LOG_ERR("Pivot failed");
            state = STATE_WAIT_FOR_START;
        } else {
            if (test_app_indicator_hndl >= 0) {
                rgb_led_set_color(test_app_indicator_hndl, &RGB_COLOR_BLUE);
                rgb_led_set_blink(test_app_indicator_hndl, INDICATOR_BLINK_ON_MS, INDICATOR_BLINK_OFF_MS);
            }
        }
        break;
    case EVENT_MOVING:
        // Nothing to do
        break;
    case EVENT_MOVE_COMPLETE:
        state = get_next_state();
        break;
    case EVENT_START:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static test_app_state_t handle_turn_cw_90deg(test_app_event_t event)
{
    test_app_state_t state = test_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        if (tracks_pivot(90) != 0) {
            LOG_ERR("Pivot failed");
            state = STATE_WAIT_FOR_START;
        } else {
            if (test_app_indicator_hndl >= 0) {
                rgb_led_set_color(test_app_indicator_hndl, &RGB_COLOR_BLUE);
                rgb_led_set_blink(test_app_indicator_hndl, INDICATOR_BLINK_ON_MS, INDICATOR_BLINK_OFF_MS);
            }
        }
        break;
    case EVENT_MOVING:
        // Nothing to do
        break;
    case EVENT_MOVE_COMPLETE:
        state = get_next_state();
        break;
    case EVENT_START:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static test_app_state_t handle_wait_for_start(test_app_event_t event)
{
    test_app_state_t state = test_app_state;

    switch (event) {
    case EVENT_ENTER_STATE:
        test_app_next_state_idx = 0;
        // Just ensure we are waiting for a start event
        start_stop_auto_stop();

        if (test_app_indicator_hndl >= 0) {
            rgb_led_set_color(test_app_indicator_hndl, &RGB_OFF);
            rgb_led_set_blink(test_app_indicator_hndl, 0, 0);
        }
        break;
    case EVENT_START:
        state = get_next_state();
        break;
    case EVENT_MOVING:
    case EVENT_MOVE_COMPLETE:
    case EVENT_STOP:
    default:
        should_not_occur(event);
        break;
    }

    return state;
}

static void should_not_occur(test_app_event_t event)
{
    LOG_WRN("Should not occur: State:%s; Event:%s;", get_state_name(test_app_state), get_event_name(event));
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

static void tracks_cb(const tracks_event_t *event)
{
	LOG_DBG("Tracks CB %d", event->moving);

    if (event->moving == true) {
        handle_event(EVENT_MOVING);
    } else {
        handle_event(EVENT_MOVE_COMPLETE);
    }
}
