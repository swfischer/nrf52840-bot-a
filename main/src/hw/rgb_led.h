/*****************************************************************************
 * rgb_led.h - header file for an RGB LED device module
 * 
 * This header file is a wrapper for an RGB LED device, allowing for color
 * changes, blinking, flashing, and brightness settings.
 *****************************************************************************/

#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdint.h>

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const rgb_color_t RGB_COLOR_BLACK = RGB(0x00, 0x00, 0x00);
static const rgb_color_t RGB_COLOR_BLUE = RGB(0x00, 0x00, 0xff);
static const rgb_color_t RGB_COLOR_GREEN = RGB(0x00, 0xff, 0x00);
static const rgb_color_t RGB_COLOR_RED = RGB(0xff, 0x00, 0x00);
static const rgb_color_t RGB_COLOR_YELLOW = RGB(0xff, 0xbf, 0x00);
static const rgb_color_t RGB_COLOR_WHITE = RGB(0xff, 0xff, 0xff);

static const rgb_color_t RGB_OFF = RGB(0x00, 0x00, 0x00);

enum { RGB_LED_BRIGHTNESS_MIN = 1, RGB_LED_BRIGHTNESS_MAX = 100 };

// Returns an instance handle on success, otherwise -1
extern int rgb_led_init(void);
extern void rgb_led_exit(int hndl);

// Returns 0 on success, otherwise -1;
extern int rgb_led_get_blink(int hndl, uint16_t *on_ms, uint16_t *off_ms);
// Returns 0 on success, otherwise -1;
extern int rgb_led_set_blink(int hndl, uint16_t on_ms, uint16_t off_ms);
// Returns 0 on success, otherwise -1;
extern int rgb_led_get_brightness(int hndl, uint8_t *brightness);
// Returns 0 on success, otherwise -1;
extern int rgb_led_set_brightness(int hndl, uint8_t brightness);
// Returns 0 on success, otherwise -1;
extern int rgb_led_get_color(int hndl, rgb_color_t *color);
// Returns 0 on success, otherwise -1;
extern int rgb_led_set_color(int hndl, const rgb_color_t *color);
// Returns 0 on success, otherwise -1;
extern int rgb_led_flash(int hndl, const rgb_color_t *color, uint16_t ms);

#endif // RGB_LED_H