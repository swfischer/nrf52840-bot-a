/*****************************************************************************
 * start_stop.c - source file for providing system start and stop indications
 *
 *****************************************************************************/

#include "start_stop.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "logger.h"
#include "os_al.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

#define BUTTON_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(BUTTON_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
#define LED_NODE DT_ALIAS(led1)
#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

#define LED_OFF (0)
#define LED_ON (1)

static const struct gpio_dt_spec ss_button = GPIO_DT_SPEC_GET_OR(BUTTON_NODE, gpios, {0});
static struct gpio_callback ss_button_cb_data;
static struct gpio_dt_spec ss_led = GPIO_DT_SPEC_GET_OR(LED_NODE, gpios, {0});

//! Flag used to know if the module has been initialized or not
static bool ss_initialized = false;

//! Worker used to perform the callbacks
static struct k_work ss_callback_worker;

//! The registered callback function and event data
static start_stop_cb_t ss_callback_func = NULL;
static start_stop_event_t ss_callback_event;

//! The current state (true for started and false for stopped)
static bool ss_curr_state = false;

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void callback_work_handler(struct k_work *work);
static int hw_init(void);
static void hw_exit(void);

/*******************************************************************************
 * @brief start_stop_init - initialize the module for use.
 *
 * @details A routine used to initialize the start/stop module for use.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int start_stop_init(void)
{
    int rc = -1;

    if (ss_initialized) {
        LOG_WRN("Already init'd");
    } else if (hw_init() != 0) {
        LOG_ERR("HW init failed");
    } else {
        k_work_init(&ss_callback_worker, callback_work_handler);
        ss_curr_state = false;
        ss_callback_func = NULL;
        ss_initialized = true;
        rc = 0;
    }

    return rc;
}

/*******************************************************************************
 * @brief start_stop_exit - de-initialize the module.
 *
 * @details A routine used to de-initialize the start/stop module.
 *
 * @return None.
 ******************************************************************************/

void start_stop_exit(void)
{
    if (ss_initialized) {
        if (k_work_is_pending(&ss_callback_worker)) {
            (void)k_work_cancel(&ss_callback_worker);
        }
        hw_exit();
        ss_initialized = false;
    }
}

/*******************************************************************************
 * @brief start_stop_reg_cb - register a callback function for event cb's.
 *
 * @details A routine used to register for event callbacks during regular
 * operation.  These callbacks should be handled like ISRs in that the
 * processing done within them must be short (non-blocking).
 *
 * @param cb - the callback function to register.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int start_stop_reg_cb(start_stop_cb_t cb)
{
    int rc = -1;

    if (!ss_initialized) {
        LOG_WRN("Not init'd");
    } else if (ss_callback_func != NULL) {
        LOG_WRN("Callback already registered");
    } else {
        ss_callback_func = cb;
        rc = 0;
    }

    return rc;
}

/*******************************************************************************
 * @brief start_stop_dereg_cb - de-register a previously registered callback.
 *
 * @details A routine used to de-register a callback function.
 *
 * @return None.
 ******************************************************************************/

void start_stop_dereg_cb(start_stop_cb_t cb)
{
    if (!ss_initialized) {
        LOG_WRN("Not init'd");
    } else if (ss_callback_func != cb) {
        LOG_WRN("Callback not registered");
    } else {
        ss_callback_func = NULL;
    }
}

/*******************************************************************************
 * @brief start_stop_auto_stop - allow reporting of an automatic stop event.
 *
 * @details A routine used to report that the started event has automatically
 * stopped, hence the next button press should report a start event.
 *
 * @return None.
 ******************************************************************************/

void start_stop_auto_stop(void)
{
    if (!ss_initialized) {
        LOG_WRN("Not init'd");
    } else {
        ss_curr_state = false;
        (void)gpio_pin_set_dt(&ss_led, LED_OFF);
    }
}

/*****************************************************************************/

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    (void)dev;

    // Just make sure our pin is interrupting
    if ((cb->pin_mask & pins) != 0) {
        k_work_submit(&ss_callback_worker);
    }
}

static void callback_work_handler(struct k_work *work)
{
    static uint32_t last_report_time = 0;

    // To debounce the button, we will not allow another transition to occur
    // within 100ms of the previous.

    //================================================================================
    // TODO: This method does not work well, need to rework and add proper debouncing.
    //================================================================================

    uint32_t now = os_al_get_ms32();

    if ((now - last_report_time) > 100) {
        if (ss_curr_state == true) {
            ss_curr_state = false;
            (void)gpio_pin_set_dt(&ss_led, LED_OFF);
        } else {
            ss_curr_state = true;
            (void)gpio_pin_set_dt(&ss_led, LED_ON);
        }

        if (ss_callback_func != NULL) {
            ss_callback_event.start_not_stop = ss_curr_state;
            ss_callback_func(&ss_callback_event);
        }
    }
}

static int hw_init(void)
{
    int rc = -1;

    if (!gpio_is_ready_dt(&ss_button)) {
        LOG_ERR("Button not ready");
	} else if (ss_led.port && !gpio_is_ready_dt(&ss_led)) {
        LOG_ERR("LED not ready");
	} else if (gpio_pin_configure_dt(&ss_button, GPIO_INPUT) != 0) {
        LOG_ERR("Button config failed");
	} else if (gpio_pin_configure_dt(&ss_led, GPIO_OUTPUT) != 0) {
        LOG_ERR("LED config failed");
	} else if (gpio_pin_interrupt_configure_dt(&ss_button, GPIO_INT_EDGE_TO_ACTIVE) != 0) {
        LOG_ERR("ISR config failed");
	} else {
		(void)gpio_pin_set_dt(&ss_led, LED_OFF);
    	gpio_init_callback(&ss_button_cb_data, button_pressed, BIT(ss_button.pin));
        if (gpio_add_callback(ss_button.port, &ss_button_cb_data) != 0) {
            LOG_ERR("Callback add failed");
        } else {
            rc = 0;
        }
    }

    return rc;
}

static void hw_exit(void)
{
    (void)gpio_remove_callback(ss_button.port, &ss_button_cb_data);
}
