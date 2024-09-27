/*****************************************************************************
 * rgb_led_shell.c - The source to shell commands for the RGB LED module
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "rgb_led.h"
#include "shell_.h"

// Assuming the default RGB LED handle of 0
static int rgb_hndl = 0;

static int rgb_cmd_blink(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        if (argc < 3) {
            shell_print(sh, "Invalid command");
            break;
        }
    
        uint16_t on = strtoul(argv[1], NULL, 0);
        uint16_t off = strtoul(argv[2], NULL, 0);

        int ret = rgb_led_set_blink(rgb_hndl, on, off);
        if (ret != 0) {
            shell_print(sh, "Set blink failed");
            break;
        }

        shell_print(sh, "Success");
        rc = 0;

    } while (0);

    return rc;
}

static int rgb_cmd_bright(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        if (argc < 2) {
            shell_print(sh, "Invalid command");
            break;
        }
    
        uint8_t percent = strtoul(argv[1], NULL, 0);

        int ret = rgb_led_set_brightness(rgb_hndl, percent);
        if (ret != 0) {
            shell_print(sh, "Set brightness failed");
            break;
        }

        shell_print(sh, "Success");
        rc = 0;

    } while(0);

    return rc;
}

static int rgb_cmd_color(const struct shell *sh, size_t argc, char **argv)
{
    const rgb_color_t *clr = &RGB_OFF;
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        if (argc < 2) {
            shell_print(sh, "Invalid command");
            break;
        }
    
        if (strcmp(argv[1], "red") == 0) {
            clr = &RGB_COLOR_RED;
        } else if (strcmp(argv[1], "grn") == 0) {
            clr = &RGB_COLOR_GREEN;
        } else if (strcmp(argv[1], "blu") == 0) {
            clr = &RGB_COLOR_BLUE;
        } else if (strcmp(argv[1], "yel") == 0) {
            clr = &RGB_COLOR_YELLOW;
        } else if (strcmp(argv[1], "wht") == 0) {
            clr = &RGB_COLOR_WHITE;
        } else if (strcmp(argv[1], "blk") == 0) {
            clr = &RGB_COLOR_BLACK;
        } else {
            shell_print(sh, "Unknown color");
            break;
        }

        int ret = rgb_led_set_color(rgb_hndl, clr);
        if (ret != 0) {
            shell_print(sh, "Set color failed");
            break;
        }

        shell_print(sh, "Success");
        rc = 0;

    } while(0);

    return rc;
}

static int rgb_cmd_flash(const struct shell *sh, size_t argc, char **argv)
{
    const rgb_color_t *clr = &RGB_OFF;
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        if (argc < 3) {
            shell_print(sh, "Invalid command");
            break;
        }
    
        if (strcmp(argv[1], "red") == 0) {
            clr = &RGB_COLOR_RED;
        } else if (strcmp(argv[1], "grn") == 0) {
            clr = &RGB_COLOR_GREEN;
        } else if (strcmp(argv[1], "blu") == 0) {
            clr = &RGB_COLOR_BLUE;
        } else if (strcmp(argv[1], "yel") == 0) {
            clr = &RGB_COLOR_YELLOW;
        } else if (strcmp(argv[1], "wht") == 0) {
            clr = &RGB_COLOR_WHITE;
        } else if (strcmp(argv[1], "blk") == 0) {
            clr = &RGB_COLOR_BLACK;
        } else {
            shell_print(sh, "Unknown color");
            break;
        }

        uint16_t ms = strtoul(argv[2], NULL, 0);

        int ret = rgb_led_flash(rgb_hndl, clr, ms);
        if (ret != 0) {
            shell_print(sh, "Flash failed");
            break;
        }

        shell_print(sh, "Success");
        rc = 0;

    } while(0);

    return rc;
}

static int rgb_cmd_status(const struct shell *sh, size_t argc, char **argv)
{
    uint16_t on = 0;
    uint16_t off = 0;
    if (rgb_led_get_blink(rgb_hndl, &on, &off) != 0) {
        shell_print(sh, "Get blink failed");
    }
    shell_print(sh, "blink ......: %u/%u", on, off);

    uint8_t brightness = 0;
    if (rgb_led_get_brightness(rgb_hndl, &brightness) != 0) {
        shell_print(sh, "Get brightness failed");
    }
    shell_print(sh, "brightness .: %u", brightness);

    rgb_color_t color = {0};
    if (rgb_led_get_color(rgb_hndl, &color) != 0) {
        shell_print(sh, "Get color failed");
    }
    shell_print(sh, "color ......: %02x,%02x,%02x", color.r, color.g, color.b);

    return 0;
}

#define RGB_CMD_HELP_MSG "RGB LED Commands"
#define RGB_BLINK_SHELL_MSG "Set blink rate (0's for no blinking)"
#define RGB_BLINK_SHELL_DETAILS "Usage: rgb blink <on> <off>"
#define RGB_BRIGHT_SHELL_MSG "Set the brightness"
#define RGB_BRIGHT_SHELL_DETAILS "Usage: rgb bright <1-100>"
#define RGB_COLOR_SHELL_MSG "Set the color"
#define RGB_COLOR_SHELL_DETAILS "Usage: rgb color <red/grn/blu/yel/wht/blk>"
#define RGB_FLASH_SHELL_MSG "Flash a color"
#define RGB_FLASH_SHELL_DETAILS "Usage: rgb flash <red/grn/blu/yel/wht/blk> <ms>"
#define RGB_STATUS_SHELL_MSG "Display RGB LED status"
#define RGB_STATUS_SHELL_DETAILS "Usage: rgb status"

#if defined(ZEPHYR_BUILD)

#define SUB_SHELL_CMD_ARG(_cmd, _func, _help1, _help2) \
    SHELL_CMD_ARG(_cmd, NULL, _help1 "\n" _help2, _func, 1, 10)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_rgb,
	SUB_SHELL_CMD_ARG(blink, rgb_cmd_blink, RGB_BLINK_SHELL_MSG, RGB_BLINK_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(bright, rgb_cmd_bright, RGB_BRIGHT_SHELL_MSG, RGB_BRIGHT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(color, rgb_cmd_color, RGB_COLOR_SHELL_MSG, RGB_COLOR_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(flash, rgb_cmd_flash, RGB_FLASH_SHELL_MSG, RGB_FLASH_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(status, rgb_cmd_status, RGB_STATUS_SHELL_MSG, RGB_STATUS_SHELL_DETAILS),
	SHELL_SUBCMD_SET_END // Array terminator, must be last
);

SHELL_CMD_REGISTER(rgb, &sub_rgb, RGB_CMD_HELP_MSG, NULL);

#else // Non-Zephyr Build
#error "Non-Zephyr Shell is Not Supported"
#endif // Non-Zephyr Build
