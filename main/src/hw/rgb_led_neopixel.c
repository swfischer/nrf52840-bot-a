/*****************************************************************************
 * rgb_led_neopixel.c - source file for an RGB LED device module based on a
 * single Neopixel device
 * 
 * This source file is a wrapper for a Neopixel RGB LED device, allowing for
 * color changes, blinking, flashing, and brightness settings.
 *****************************************************************************/

#include "rgb_led.h"

#include <pthread.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include "logger.h"
#include "os_al.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

// Removed all compile time issue if the device does not exist

#define STRIP_NODE DT_ALIAS(led_strip)
//#if !DT_NODE_HAS_STATUS(STRIP_NODE, okay)
//#error "\"led_strip\" devicetree alias is not defined"
//#endif
#if DT_NODE_HAS_PROP(STRIP_NODE, chain_length)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)
#else
//#error "Unable to determine length of LED strip"
#define STRIP_NUM_PIXELS (0)
#endif

#define NON_TIMER_EXPIRY (false)
#define TIMER_EXPIRY (true)

static const struct device *const neopixel = DEVICE_DT_GET_OR_NULL(STRIP_NODE);

//! This mutex is used to serialize public function calls
static pthread_mutex_t rgb_api_lock = PTHREAD_MUTEX_INITIALIZER;

//! Worker used to handle timer expirations
static struct k_work rgb_expiry_worker;

typedef enum { TIMER_USE_NONE, TIMER_USE_BLINK, TIMER_USE_FLASH } timer_use_t;

typedef struct
{
    bool initialized;

    bool blink_state_on;
    uint16_t blink_on_ms;
    uint16_t blink_off_ms;
    uint8_t brightness;
    rgb_color_t color;

    timer_use_t timer_use;
    os_al_timer_t timer;
    uint32_t timer_remaining_ms;
    bool timer_expired;

} rgb_data_t;

#define RGB_MAX_NUMBER_OF_HANDLES (1)
static rgb_data_t rgb_hndl_data[RGB_MAX_NUMBER_OF_HANDLES] = {0};

static void expiry_work_handler(struct k_work *work);
static void hw_exit(int hndl);
static int hw_init(int hndl);
static int hw_setup(int hndl, bool expiry);
static int set_color_and_brightness(const rgb_color_t *color, uint8_t brightness);
static int set_flash(int hndl, const rgb_color_t *color, uint16_t ms);
static void timer_expired_cb(os_al_timer_t timer);

/*******************************************************************************
 * @brief rgb_led_init - initialize an RGB LED instance for use.
 *
 * @details A routine used to initialize an RGB LED instance for use.
 *
 * @return returns an instance handle on success, otherwise -1.
 ******************************************************************************/

int rgb_led_init(void)
{
    int tmp_hndl = 0; // Assume a handle of 0
    int hndl = -1;

    pthread_mutex_lock(&rgb_api_lock);

    if (rgb_hndl_data[tmp_hndl].initialized == true) {
        LOG_WRN("No resources available");
    } else if (hw_init(tmp_hndl) != 0) {
        LOG_ERR("Init failed");
    } else {
        rgb_data_t *data = &rgb_hndl_data[tmp_hndl];

        data->timer = os_al_timer_create(timer_expired_cb);
        if (data->timer < 0) {
            LOG_ERR("Timer creation failed");
        } else {
            k_work_init(&rgb_expiry_worker, expiry_work_handler);
            data->initialized = true;
            data->timer_use = TIMER_USE_NONE;
            data->timer_remaining_ms = 0;
            data->timer_expired = false;
            hndl = tmp_hndl;
        }
    }

    pthread_mutex_unlock(&rgb_api_lock);

    return hndl;
}

/*******************************************************************************
 * @brief dcme_exit - de-initialize the given RGB LED handle
 *
 * @details A routine used to revert the initialization of an RGB LED instance.
 *
 * @param hndl - the instance handle to close.
 *
 * @return none.
 ******************************************************************************/

void rgb_led_exit(int hndl)
{
    pthread_mutex_lock(&rgb_api_lock);

    if ((hndl < 0) || (hndl >= RGB_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (rgb_hndl_data[hndl].initialized == false) {
        LOG_WRN("Already exit'd");
    } else {
        hw_exit(hndl);
    }

    pthread_mutex_unlock(&rgb_api_lock);
}

/*******************************************************************************
 * @brief rgb_led_get_blink - retrieve the current RGB LED blink state
 *
 * @details A routine used to retrieve the current RGB LED blink state.
 *
 * @param hndl - the instance handle to use.
 * @param on_ms - pointer to return the "on" time into
 * @param off_ms - pointer to return the "off" time into
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int rgb_led_get_blink(int hndl, uint16_t *on_ms, uint16_t *off_ms)
{
    int rc = -1;

    pthread_mutex_lock(&rgb_api_lock);

    if ((hndl < 0) || (hndl >= RGB_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (rgb_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        rgb_data_t *data = &rgb_hndl_data[hndl];

        if (on_ms != NULL) {
            *on_ms = data->blink_on_ms;
        }
        if (off_ms != NULL) {
            *off_ms = data->blink_off_ms;
        }
        rc = 0;
    }

    pthread_mutex_unlock(&rgb_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief rgb_led_set_blink - set the current RGB LED blink state
 *
 * @details A routine used to set the current RGB LED blink state.  Note that
 * either or both of the on/off time values being zero will disable blinking.
 *
 * @param hndl - the instance handle to use.
 * @param on_ms - the "on" time to use in milliseconds
 * @param off_ms - the "off" time to use in milliseconds
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int rgb_led_set_blink(int hndl, uint16_t on_ms, uint16_t off_ms)
{
    int rc = -1;

    pthread_mutex_lock(&rgb_api_lock);

    if ((hndl < 0) || (hndl >= RGB_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (rgb_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        rgb_data_t *data = &rgb_hndl_data[hndl];

        if ((on_ms == 0) || (off_ms == 0)) {
            data->blink_on_ms = 0;
            data->blink_off_ms = 0;
        } else {
            data->blink_state_on = false;
            data->blink_on_ms = on_ms;
            data->blink_off_ms = off_ms;
        }

        rc = hw_setup(hndl, NON_TIMER_EXPIRY);
    }

    pthread_mutex_unlock(&rgb_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief rgb_led_get_brightness - retrieve the current RGB LED brightness value
 *
 * @details A routine used to retrieve the current RGB LED brightness value.
 *
 * @param hndl - the instance handle to use.
 * @param brightness - pointer to return the brightness value into
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int rgb_led_get_brightness(int hndl, uint8_t *brightness)
{
    int rc = -1;

    pthread_mutex_lock(&rgb_api_lock);

    if ((hndl < 0) || (hndl >= RGB_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (rgb_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        rgb_data_t *data = &rgb_hndl_data[hndl];

        if (brightness != NULL) {
            *brightness = data->brightness;
        }
        rc = 0;
    }

    pthread_mutex_unlock(&rgb_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief rgb_led_set_brightness - set the current RGB LED brightness value
 *
 * @details A routine used to set the current RGB LED brightness value.  Valid
 * brightness values are 1 (dimmest) to 100 (brightest).
 *
 * @param hndl - the instance handle to use.
 * @param brightness - the brightness value to set
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int rgb_led_set_brightness(int hndl, uint8_t brightness)
{
    int rc = -1;

    pthread_mutex_lock(&rgb_api_lock);

    if ((hndl < 0) || (hndl >= RGB_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (rgb_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else if ((brightness < RGB_LED_BRIGHTNESS_MIN) || (brightness > RGB_LED_BRIGHTNESS_MAX)) {
        LOG_WRN("Invalid param");
    } else {
        rgb_data_t *data = &rgb_hndl_data[hndl];

        data->brightness = brightness;

        rc = hw_setup(hndl, NON_TIMER_EXPIRY);
    }

    pthread_mutex_unlock(&rgb_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief rgb_led_get_color - retrieve the current RGB LED color value
 *
 * @details A routine used to retrieve the current RGB LED color value.
 *
 * @param hndl - the instance handle to use.
 * @param brightness - pointer to return the color value into
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int rgb_led_get_color(int hndl, rgb_color_t *color)
{
    int rc = -1;

    pthread_mutex_lock(&rgb_api_lock);

    if ((hndl < 0) || (hndl >= RGB_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (rgb_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else {
        rgb_data_t *data = &rgb_hndl_data[hndl];

        if (color != NULL) {
            memcpy(color, &data->color, sizeof(*color));
        }
        rc = 0;
    }

    pthread_mutex_unlock(&rgb_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief rgb_led_set_color - set the current RGB LED color value
 *
 * @details A routine used to set the current RGB LED color value.
 *
 * @param hndl - the instance handle to use.
 * @param color - the color value to set.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int rgb_led_set_color(int hndl, const rgb_color_t *color)
{
    int rc = -1;

    pthread_mutex_lock(&rgb_api_lock);

    if ((hndl < 0) || (hndl >= RGB_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (rgb_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else if (color == NULL) {
        LOG_WRN("Invalid param");
    } else {
        rgb_data_t *data = &rgb_hndl_data[hndl];

        memcpy(&data->color, color, sizeof(data->color));

        rc = hw_setup(hndl, NON_TIMER_EXPIRY);
    }

    pthread_mutex_unlock(&rgb_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief rgb_led_flash - flash the RGB LED with the given color value and time
 *
 * @details A routine used to flash the RGB LED with a given color value for a
 * given amount of time.  After the flash time is over, return to the previous
 * state.
 *
 * @param hndl - the instance handle to use.
 * @param color - the color value to flash.
 * @param ms - the time of the flash in milliseconds.
 *
 * @return Returns 0 on success, otherwise -1.
 ******************************************************************************/

int rgb_led_flash(int hndl, const rgb_color_t *color, uint16_t ms)
{
    int rc = -1;

    pthread_mutex_lock(&rgb_api_lock);

    if ((hndl < 0) || (hndl >= RGB_MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (rgb_hndl_data[hndl].initialized == false) {
        LOG_WRN("Not initialized");
    } else if (color == NULL) {
        LOG_WRN("Invalid param");
    } else {
        rc = set_flash(hndl, color, ms);
    }

    pthread_mutex_unlock(&rgb_api_lock);

    return rc;
}

/*****************************************************************************/

static void expiry_work_handler(struct k_work *work)
{
    rgb_data_t *data = NULL;
    int hndl = -1;

    // Handle all timer expirations
    for (int i = 0; i < RGB_MAX_NUMBER_OF_HANDLES; i++) {
        if (rgb_hndl_data[i].timer_expired == true) {
            hndl = i;
            data = &rgb_hndl_data[hndl];
            data->timer_expired = false;

            switch (data->timer_use) {
            case TIMER_USE_BLINK:
                hw_setup(hndl, TIMER_EXPIRY);
                break;
            case TIMER_USE_FLASH:
                // The flash has ended, so now setup the LED again for the existing settings
                hw_setup(hndl, TIMER_EXPIRY);
                break;
            case TIMER_USE_NONE:
            default:
                LOG_WRN("Unused timer expiry (%d)", hndl);
                break;
            }
        }
    }
}

static void hw_exit(int hndl)
{
    rgb_data_t *data = &rgb_hndl_data[hndl];

    data->blink_on_ms = 0;
    data->blink_off_ms = 0;
    data->brightness = 0;
    memcpy(&data->color, &RGB_OFF, sizeof(data->color));
    (void)hw_setup(hndl, NON_TIMER_EXPIRY);
}

static int hw_init(int hndl)
{
    int rc = -1;

    if (neopixel == NULL) {
		LOG_WRN("Neopixel does not exist");
	} else if (!device_is_ready(neopixel)) {
		LOG_WRN("Neopixel is not ready");
    } else if (STRIP_NUM_PIXELS != 1) {
        LOG_ERR("Invalid pixel length");
	} else {
        rgb_data_t *data = &rgb_hndl_data[hndl];

        data->blink_state_on = false;
        data->blink_on_ms = 0;
        data->blink_off_ms = 0;
        data->brightness = 50;
        memcpy(&data->color, &RGB_OFF, sizeof(data->color));
        rc = hw_setup(hndl, NON_TIMER_EXPIRY);
    }

    return rc;
}

static int hw_setup(int hndl, bool expiry)
{
    rgb_data_t *data = &rgb_hndl_data[hndl];
    uint32_t ms = 0;
    int rc = -1;

    if (data->blink_on_ms == 0) {
        // Not blinking, so make sure the timer is cancelled
        os_al_timer_cancel(data->timer);
        data->timer_use = TIMER_USE_NONE;
        data->timer_remaining_ms = 0;
        data->timer_expired = false;
    } else if ((data->timer_use == TIMER_USE_BLINK) && (data->blink_on_ms > 0)) {
        if (expiry) {
            // Continue the blinking on an expiration
            data->blink_state_on = (data->blink_state_on == true) ? false : true;
            ms = (data->blink_state_on == true) ? data->blink_on_ms : data->blink_off_ms;
        }
    } else if (data->timer_remaining_ms > 0) {
        // The blinking was pre-empted, so start from where we were
        ms = data->timer_remaining_ms;
        data->timer_remaining_ms = 0;
        data->timer_use = TIMER_USE_BLINK;
    } else {
        // Start blinking, always from the "on" state
        ms = data->blink_on_ms;
        data->timer_use = TIMER_USE_BLINK;
        data->blink_state_on = true;
    }

    if (data->timer_use == TIMER_USE_BLINK) {
        if (ms > 0) {
            data->timer_expired = false;
            int ret = os_al_timer_start(data->timer, ms, TIMER_TYPE_ONE_SHOT);
            if (ret != 0) {
                LOG_WRN("Timer start failed");
                data->timer_use = TIMER_USE_NONE;
            } else {
                if (data->blink_state_on == true) {
                    rc = set_color_and_brightness(&data->color, data->brightness);
                } else {
                    rc = set_color_and_brightness(&RGB_OFF, data->brightness);
                }
            }
        }
    } else {
        rc = set_color_and_brightness(&data->color, data->brightness);
    }

    return rc;
}

#define APPLY_BRIGHTNESS(c,b) (uint8_t)((((uint16_t)c) * b) / RGB_LED_BRIGHTNESS_MAX)

static int set_color_and_brightness(const rgb_color_t *color, uint8_t brightness)
{
    int rc = -1;

    struct led_rgb pixel_color;
    pixel_color.r = APPLY_BRIGHTNESS(color->r, brightness);
    pixel_color.g = APPLY_BRIGHTNESS(color->g, brightness);
    pixel_color.b = APPLY_BRIGHTNESS(color->b, brightness);

    int ret = led_strip_update_rgb(neopixel, &pixel_color, 1);
    if (ret != 0) {
        LOG_WRN("RGB update failed (%d)", ret);
    } else {
        rc = 0;
    }

    return rc;
}

static int set_flash(int hndl, const rgb_color_t *color, uint16_t ms)
{
    rgb_data_t *data = &rgb_hndl_data[hndl];
    int rc = -1;

    if (data->timer_use != TIMER_USE_NONE) {
        data->timer_remaining_ms = os_al_timer_remaining(data->timer);
        os_al_timer_cancel(data->timer);
    }

    if (set_color_and_brightness(color, data->brightness) != 0) {
        // Return an error
    } else if (os_al_timer_start(data->timer, ms, TIMER_TYPE_ONE_SHOT) != 0) {
        LOG_ERR("Timer start failed");
        hw_setup(hndl, NON_TIMER_EXPIRY); // Return to the previous settings
    } else {
        data->timer_use = TIMER_USE_FLASH;
        rc = 0;
    }

    return rc;
}

static void timer_expired_cb(os_al_timer_t timer)
{
    int cnt = 0;

    // Find and mark the given timer as having expired
    for (int i = 0; i < RGB_MAX_NUMBER_OF_HANDLES; i++) {
        if (timer == rgb_hndl_data[i].timer) {
            rgb_hndl_data[i].timer_expired = true;
            cnt ++;
        }
    }

    // If an expiration occurred, queue the expiration worker
    if (cnt > 0) {
        k_work_submit(&rgb_expiry_worker);
    }
}
