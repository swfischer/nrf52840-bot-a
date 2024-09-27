/*****************************************************************************
 * tfluna_shell.c - The source to shell commands for TF-Luna LIDAR sensor
 *****************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "tfluna.h"
#include "shell_.h"

// Comment out the following line to remove the "init" and "exit" commands
//#define ALLOW_INIT_EXIT_CMDS (1)

#if defined(ZEPHYR_BUILD)
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#if defined(ALLOW_INIT_EXIT_CMDS)
static const struct device *i2c_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(i2c1));
#endif
#elif defined(MAC_BUILD)
#elif defined(WINDOWS_BUILD)
#endif

static int tfluna_cmds_hndl = -1;

static int tfluna_cmd_init(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (tfluna_cmds_hndl != -1) {
		shell_info(sh, "Driver already init'd");
    } else {
#if defined(ALLOW_INIT_EXIT_CMDS)
        tfluna_hardware_t hw;
        hw.i2c_dev = i2c_dev;
        hw.i2c_addr = TFLUNA_I2C_ADDR;

        int hndl = tfluna_init(&hw);
        if (hndl < 0) {
            shell_warn(sh, "Init failed");
        } else {
            shell_print(sh, "Success");
            tfluna_cmds_hndl = hndl;
            rc = 0;
        }
#else
        shell_info(sh, "Assuming FD = 0\n");
        tfluna_cmds_hndl = 0;
        rc = 0;
#endif
    }

    return rc;
}

static int tfluna_cmd_exit(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (tfluna_cmds_hndl == -1) {
		shell_info(sh, "Driver already exit'd");
    } else {
#if defined(ALLOW_INIT_EXIT_CMDS)
        tfluna_exit(tfluna_cmds_hndl);
#endif
        tfluna_cmds_hndl = -1;
        rc = 0;
    }

    return rc;
}

static int tfluna_cmd_get(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (tfluna_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else {
        uint16_t cm;
        uint16_t amplitude;

        if (tfluna_get(tfluna_cmds_hndl, &cm, &amplitude) != 0) {
		    shell_warn(sh, "Get failed");
        } else {
		    shell_print(sh, "Distance ..: %u cm", cm);
		    shell_print(sh, "Amplitude .: %u", amplitude);
            rc = 0;
        }
    }

    return rc;
}

static int tfluna_cmd_setcal(const struct shell *sh, size_t argc, char **argv)
{
    static const uint8_t ARGV_IDX = 1;
    static const uint8_t ARGV_CM = 2;
    static const uint8_t ARGV_FACTOR = 3;

    int rc = -1;

    if (tfluna_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else if (argc <= ARGV_FACTOR) {
		shell_warn(sh, "Not enough parameters");
    } else {
        uint8_t idx = (uint8_t) strtoul(argv[ARGV_IDX], NULL, 0);
        uint16_t cm = (uint16_t) strtoul(argv[ARGV_CM], NULL, 0);
        float factor = strtof(argv[ARGV_FACTOR], NULL);

        if (idx >= TFL_MAX_CAL_POINTS) {
		    shell_warn(sh, "Invalid cal index");
        } else if (tfluna_set_cal_point(tfluna_cmds_hndl, idx, cm, factor) != 0) {
		    shell_warn(sh, "Setcal failed");
        } else {
		    shell_print(sh, "Success");
            rc = 0;
        }
    }

    return rc;
}

static int tfluna_cmd_status(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (tfluna_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else {
        tfluna_status_t status;

        if (tfluna_get_status(tfluna_cmds_hndl, &status) != 0) {
		    shell_warn(sh, "Get status failed");
        } else {
		    shell_print(sh, "Distance ....: %u cm", status.cm);
		    shell_print(sh, "Amplitude ...: %u", status.amplitude);
		    shell_print(sh, "Tempurature .: %u.%u C", (status.temp / 100), (status.temp % 100));
		    shell_print(sh, "Timestamp ...: %u", status.ticks);
		    shell_print(sh, "Error .......: %u", status.error);
		    shell_print(sh, "Limit min ...: %u", status.limit_min);
		    shell_print(sh, "Limit max ...: %u", status.limit_max);
            for (int i = 0; i < TFL_MAX_CAL_POINTS; i++) {
                if (status.cal.cm[i] != TFL_UNUSED_CAL_CM) {
                    shell_print(sh, "Cal[%d] ......: %u, %d.%03d",i , status.cal.cm[i], (int)status.cal.factor[i], (((int)(status.cal.factor[i] * 1000)) % 1000));
                }
            }
            rc = 0;
        }
    }

    return rc;
}

static int tfluna_cmd_dbgrd(const struct shell *sh, size_t argc, char **argv)
{
    static const uint8_t ARGV_REG = 1;

    int rc = -1;

    if (tfluna_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else if (argc <= ARGV_REG) {
		shell_warn(sh, "Not enough parameters");
    } else {
        uint8_t reg = (uint8_t) strtoul(argv[ARGV_REG], NULL, 0);
        uint8_t data = 0;

        rc = tfluna_reg_read(tfluna_cmds_hndl, reg, &data);
        if (rc == 0) {
            shell_print(sh, "Success");
        } else {
            shell_print(sh, "Failure");
        }
        if (rc == 0) {
            shell_print(sh, "RET = %d (0x%X)", data, data);
        }
    }

    return rc;
}

static int tfluna_cmd_dbgwr(const struct shell *sh, size_t argc, char **argv)
{
    static const uint8_t ARGV_REG = 1;
    static const uint8_t ARGV_DATA = 2;

    int rc = -1;

    if (tfluna_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else if (argc <= ARGV_DATA) {
		shell_warn(sh, "Not enough parameters");
    } else {
        uint8_t reg = (uint8_t) strtoul(argv[ARGV_REG], NULL, 0);
        uint8_t data = (uint8_t) strtoul(argv[ARGV_DATA], NULL, 0);

        rc = tfluna_reg_write(tfluna_cmds_hndl, reg, data);
        if (rc == 0) {
            shell_print(sh, "Success");
        } else {
            shell_print(sh, "Failure");
        }
    }

    return rc;
}


#define TFL_CMD_HELP_MSG "TF-Luna LIDAR Sensor Commands"
#define TFL_INIT_SHELL_MSG "Initialize an instance of the TF-Luna driver"
#define TFL_INIT_SHELL_DETAILS "Usage: tfl init"
#define TFL_EXIT_SHELL_MSG "Close/exit the instance of the TF-Luna driver"
#define TFL_EXIT_SHELL_DETAILS "Usage: tfl exit"
#define TFL_GET_SHELL_MSG "Retrieve a distance measurement"
#define TFL_GET_SHELL_DETAILS "Usage: tfl get"
#define TFL_SETCAL_SHELL_MSG "Set a calibration point"
#define TFL_SETCAL_SHELL_DETAILS "Usage: tfl <idx> <cm> <factor>"
#define TFL_STATUS_SHELL_MSG "Retrieves current sensor status"
#define TFL_STATUS_SHELL_DETAILS "Usage: tfl status"
#define TFL_DBGRD_SHELL_MSG "Perform a debug register read"
#define TFL_DBGRD_SHELL_DETAILS "Usage: tfl dbgrd <reg>"
#define TFL_DBGWR_SHELL_MSG "Perform a debug register write"
#define TFL_DBGWR_SHELL_DETAILS "Usage: tfl dbgwr <reg> <data>"

#if defined(ZEPHYR_BUILD)

#define SUB_SHELL_CMD_ARG(_cmd, _func, _help1, _help2) \
    SHELL_CMD_ARG(_cmd, NULL, _help1 "\n" _help2, _func, 1, 10)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tfl,
    SUB_SHELL_CMD_ARG(init, tfluna_cmd_init, TFL_INIT_SHELL_MSG, TFL_INIT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(exit, tfluna_cmd_exit, TFL_EXIT_SHELL_MSG, TFL_EXIT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(get, tfluna_cmd_get, TFL_GET_SHELL_MSG, TFL_GET_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(setcal, tfluna_cmd_setcal, TFL_SETCAL_SHELL_MSG, TFL_SETCAL_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(status, tfluna_cmd_status, TFL_STATUS_SHELL_MSG, TFL_STATUS_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(dbgrd, tfluna_cmd_dbgrd, TFL_DBGRD_SHELL_MSG, TFL_DBGRD_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(dbgwr, tfluna_cmd_dbgwr, TFL_DBGWR_SHELL_MSG, TFL_DBGWR_SHELL_DETAILS),
	SHELL_SUBCMD_SET_END // Array terminator, must be last
);

SHELL_CMD_REGISTER(tfl, &sub_tfl, TFL_CMD_HELP_MSG, NULL);

#else // Non-Zephyr Build

static void tfl_shell_help(void)
{
    shell_print(NULL, TFL_CMD_HELP_MSG);
    shell_print(NULL, "  init     : " TFL_INIT_SHELL_MSG);
    shell_print(NULL, "             " TFL_INIT_SHELL_DETAILS);
    shell_print(NULL, "  exit     : " TFL_EXIT_SHELL_MSG);
    shell_print(NULL, "             " TFL_EXIT_SHELL_DETAILS);
    shell_print(NULL, "  get      : " TFL_GET_SHELL_MSG);
    shell_print(NULL, "             " TFL_GET_SHELL_DETAILS);
    shell_print(NULL, "  setcal   : " TFL_SETCAL_SHELL_MSG);
    shell_print(NULL, "             " TFL_SETCAL_SHELL_DETAILS);
    shell_print(NULL, "  status   : " TFL_STATUS_SHELL_MSG);
    shell_print(NULL, "             " TFL_STATUS_SHELL_DETAILS);
    shell_print(NULL, "  dbgrd    : " TFL_DBGRD_SHELL_MSG);
    shell_print(NULL, "             " TFL_DBGRD_SHELL_DETAILS);
    shell_print(NULL, "  dbgwr    : " TFL_DBGWR_SHELL_MSG);
    shell_print(NULL, "             " TFL_DBGWR_SHELL_DETAILS);
}

#define HANDLE_CMD(_name, _func) \
    else if (strcmp(argv[0], _name) == 0) { \
        (void) _func(sh, argc, argv); \
        rc = 0; \
    }

static int tfl_shell_cmd_handler(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -ENOEXEC;

    if (argc == 0) {
        tfl_shell_help();
    }
    HANDLE_CMD("init", tfluna_cmd_init)
    HANDLE_CMD("exit", tfluna_cmd_exit)
    HANDLE_CMD("get", tfluna_cmd_get)
    HANDLE_CMD("setcal", tfluna_cmd_setcal)
    HANDLE_CMD("status", tfluna_cmd_status)
    HANDLE_CMD("dbgrd", tfluna_cmd_dbgrd)
    HANDLE_CMD("dbgwr", tfluna_cmd_dbgwr)

    return rc;
}

void tfluna_reg_shell_cmds(void)
{
    shell_nz_reg_cmd("tfl", tfl_shell_cmd_handler, TFL_CMD_HELP_MSG, tfl_shell_help);
}

#endif // Non-Zephyr Build
