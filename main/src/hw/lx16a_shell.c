/*****************************************************************************
 * lx16a_shell.c - The source to shell commands for LX-16A Serial Bus Servos
 *****************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lx16a.h"
#include "shell_.h"

// Comment out the following line to remove the "init" and "exit" commands
//#define ALLOW_INIT_EXIT_CMDS (1)

#if defined(ZEPHYR_BUILD)
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#if defined(ALLOW_INIT_EXIT_CMDS)
static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
#endif
#elif defined(MAC_BUILD)
#if defined(ALLOW_INIT_EXIT_CMDS)
static const char uart_dev[] = "/dev/cu.usbserial-0001";
#endif
#elif defined(WINDOWS_BUILD)
#if defined(ALLOW_INIT_EXIT_CMDS)
static const char uart_dev[] = "COM1";
#endif
#endif

#define UART_BAUD_RATE (115200)

static int lx16a_cmds_hndl = -1;

static uint8_t get_read_param_cnt(uint8_t cmd);

static int lx16a_cmd_init(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (lx16a_cmds_hndl != -1) {
		shell_info(sh, "Driver already init'd");
    } else {
#if defined(ALLOW_INIT_EXIT_CMDS)
        lx16a_hardware_t hw;
        hw.uart = uart_dev;
        hw.baud = UART_BAUD_RATE;

        int hndl = lx16a_init(&hw);
        if (hndl < 0) {
            shell_warn(sh, "Init failed");
        } else {
            shell_print(sh, "Success");
            lx16a_cmds_hndl = hndl;
            rc = 0;
        }
#else
        shell_info(sh, "Assuming FD = 0\n");
        lx16a_cmds_hndl = 0;
        rc = 0;
#endif
    }

    return rc;
}

static int lx16a_cmd_exit(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (lx16a_cmds_hndl == -1) {
		shell_info(sh, "Driver already exited");
    } else {
#if defined(ALLOW_INIT_EXIT_CMDS)
        lx16a_exit(lx16a_cmds_hndl);
#endif
        lx16a_cmds_hndl = -1;
        rc = 0;
    }

    return rc;
}

static int lx16a_cmd_en(const struct shell *sh, size_t argc, char **argv)
{
    static const uint8_t ARGV_EN = 1;

    int rc = -1;

    if (lx16a_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else if (argc <= ARGV_EN) {
		shell_warn(sh, "Wrong parameter count");
    } else {
        uint8_t en = (uint8_t) strtoul(argv[ARGV_EN], NULL, 0);

        int ret = lx16a_set_all_enable_states(lx16a_cmds_hndl, (en != 0));
        if (ret != 0) {
            shell_print(sh, "Failure");
        } else {
            shell_print(sh, "Success");
        }
    }

    return rc;
}

static int lx16a_cmd_get_angle(const struct shell *sh, size_t argc, char **argv)
{
    static const uint8_t ARGV_ID = 1;

    int rc = -1;

    if (lx16a_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else if (argc <= ARGV_ID) {
		shell_warn(sh, "Wrong parameter count");
    } else {
        uint8_t id = (uint8_t) strtoul(argv[ARGV_ID], NULL, 0);

        float angle = lx16a_get_angle(lx16a_cmds_hndl, id);
        if (angle < 0.1f) {
            shell_print(sh, "Failure");
        } else {
            shell_print(sh, "Success");
            shell_print(sh, "Angle: %d", (int)angle);
            rc = 0;
        }
    }

    return rc;
}

static int lx16a_cmd_set_angle(const struct shell *sh, size_t argc, char **argv)
{
    static const uint8_t ARGV_ID = 1;
    static const uint8_t ARGV_ANGLE = 2;
    static const uint8_t ARGV_MS = 3;

    int rc = -1;

    if (lx16a_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else if (argc <= ARGV_ANGLE) {
		shell_warn(sh, "Wrong parameter count");
    } else {
        uint8_t id = (uint8_t) strtoul(argv[ARGV_ID], NULL, 0);
        uint8_t angle = strtof(argv[ARGV_ANGLE], NULL);
        uint16_t ms = 1000;
        
        if (argc > ARGV_MS) {
            ms = (uint16_t) strtoul(argv[ARGV_MS], NULL, 0);
        }

        rc = lx16a_set_angle(lx16a_cmds_hndl, id, angle, ms);
        if (rc != 0) {
            shell_print(sh, "Failure");
        } else {
            shell_print(sh, "Success");
        }
    }

    return rc;
}

static int lx16a_cmd_status(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (lx16a_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else {
        lx16a_stats_t stats;
        int ret = lx16a_get_stats(lx16a_cmds_hndl, &stats);
        if (ret != 0) {
		    shell_print(sh, "Stats call failed");
        } else {
            shell_print(sh, "lost .........: %u", stats.lost);
            shell_print(sh, "rd_retries ...: %u", stats.rd_retries);
            shell_print(sh, "rd_timeouts ..: %u", stats.rd_timeouts);
            shell_print(sh, "rd_invalids ..: %u", stats.rd_invalids);
            shell_print(sh, "rd_fails .....: %u", stats.rd_fails);
            shell_print(sh, "wr_retries ...: %u", stats.wr_retries);
            shell_print(sh, "wr_fails .....: %u", stats.wr_fails);
            rc = 0;
        }
    }

    return rc;
}

static int lx16a_cmd_dbgrd(const struct shell *sh, size_t argc, char **argv)
{
    static const uint8_t ARGV_ID = 1;
    static const uint8_t ARGV_CMD = 2;

    int rc = -1;

    if (lx16a_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else if (argc <= ARGV_CMD) {
		shell_warn(sh, "Not enough parameters");
    } else {
        uint8_t id = (uint8_t) strtoul(argv[ARGV_ID], NULL, 0);
        uint8_t cmd = (uint8_t) strtoul(argv[ARGV_CMD], NULL, 0);
        uint8_t params[MAX_CMD_PARAM_CNT] = {0};
        uint8_t paramCnt = get_read_param_cnt(cmd);

        rc = lx16a_cmd_read(lx16a_cmds_hndl, id, cmd, params, paramCnt);
        if (rc == 0) {
            shell_print(sh, "Success");
        } else {
            shell_print(sh, "Failure");
        }
        if (rc == 0) {
            for (int i = 0; i < paramCnt; i++) {
                shell_print(sh, "RET[%d] = %d (0x%X)", i, params[i], params[i]);
            }
        }
    }

    return rc;
}

static int lx16a_cmd_dbgwr(const struct shell *sh, size_t argc, char **argv)
{
    static const uint8_t ARGV_ID = 1;
    static const uint8_t ARGV_CMD = 2;
    static const uint8_t ARGV_PARM0 = 3;
    static const uint8_t ARGV_PARM1 = 4;
    static const uint8_t ARGV_PARM2 = 5;
    static const uint8_t ARGV_PARM3 = 6;

    int rc = -1;

    if (lx16a_cmds_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else if (argc <= ARGV_CMD) {
		shell_warn(sh, "Not enough parameters");
    } else {
        uint8_t id = (uint8_t) strtoul(argv[ARGV_ID], NULL, 0);
        uint8_t cmd = (uint8_t) strtoul(argv[ARGV_CMD], NULL, 0);
        uint8_t params[MAX_CMD_PARAM_CNT] = {0};
        uint8_t paramCnt = 0;

        if (argc > ARGV_PARM0) {
            params[0] = (uint8_t) strtoul(argv[ARGV_PARM0], NULL, 0);
            paramCnt ++;
        }
        if (argc > ARGV_PARM1) {
            params[1] = (uint8_t) strtoul(argv[ARGV_PARM1], NULL, 0);
            paramCnt ++;
        }
        if (argc > ARGV_PARM2) {
            params[2] = (uint8_t) strtoul(argv[ARGV_PARM2], NULL, 0);
            paramCnt ++;
        }
        if (argc > ARGV_PARM3) {
            params[3] = (uint8_t) strtoul(argv[ARGV_PARM3], NULL, 0);
            paramCnt ++;
        }

        rc = lx16a_cmd_write(lx16a_cmds_hndl, id, cmd, params, paramCnt);
        if (rc == 0) {
            shell_print(sh, "Success");
        } else {
            shell_print(sh, "Failure");
        }
    }

    return rc;
}

static uint8_t get_read_param_cnt(uint8_t cmd)
{
    static const uint8_t read_param_cnt_table[][2] = {
    { LX16A_CMD_MOVE_TIME_READ, 4},
    { LX16A_CMD_MOVE_TIME_WAIT_READ, 4},
    { LX16A_CMD_ID_READ, 1},
    { LX16A_CMD_ANGLE_OFFSET_READ, 1},
    { LX16A_CMD_ANGLE_LIMIT_READ, 4},
    { LX16A_CMD_VIN_LIMIT_READ, 4},
    { LX16A_CMD_TEMP_MAX_LIMIT_READ, 1},
    { LX16A_CMD_TEMP_READ, 1},
    { LX16A_CMD_VIN_READ, 2},
    { LX16A_CMD_POS_READ, 2},
    { LX16A_CMD_SERVO_OR_MOTOR_MODE_READ, 4},
    { LX16A_CMD_LOAD_OR_UNLOAD_READ, 1},
    { LX16A_CMD_LED_CTRL_READ, 1},
    { LX16A_CMD_LED_ERROR_READ, 1},
    { 0xFF, 0xFF }
    };

    uint8_t paramCnt = 0;
    int i = 0;

    while (read_param_cnt_table[i][0] != 0xFF) {
        if (read_param_cnt_table[i][0] == cmd) {
            paramCnt = read_param_cnt_table[i][1];
            break;
        }
        i ++;
    }

    return paramCnt;
}

#define LX16A_CMD_HELP_MSG "LX16A Servo Commands"
#define LX16A_INIT_SHELL_MSG "Initialize an instance of the LX-16A driver"
#define LX16A_INIT_SHELL_DETAILS "Usage: lx init"
#define LX16A_EXIT_SHELL_MSG "Close/exit the instance of the LX-16A driver"
#define LX16A_EXIT_SHELL_DETAILS "Usage: lx exit"
#define LX16A_EN_SHELL_MSG "Enable (1) or disable (0) all servos on the default link"
#define LX16A_EN_SHELL_DETAILS "Usage: lx en <1|0>"
#define LX16A_GET_SHELL_MSG "Get servo angle for servo <id>"
#define LX16A_GET_SHELL_DETAILS "Usage: lx get <id>"
#define LX16A_SET_SHELL_MSG "Set servo angle for servo <id> to <ang> in [ms] milliseconds"
#define LX16A_SET_SHELL_DETAILS "Usage: lx set <id> <ang> [ms]"
#define LX16A_STATUS_SHELL_MSG "Get servo status"
#define LX16A_STATUS_SHELL_DETAILS "Usage: lx status"
#define LX16A_DBGRD_SHELL_MSG "Perform a debug read command"
#define LX16A_DBGRD_SHELL_DETAILS "Usage: lx dbgrd <id> <cmd>"
#define LX16A_DBGWR_SHELL_MSG "Perform a debug write command"
#define LX16A_DBGWR_SHELL_DETAILS "Usage: lx dbgwr <id> <cmd> [params..]"

#if defined(ZEPHYR_BUILD)

#define SUB_SHELL_CMD_ARG(_cmd, _func, _help1, _help2) \
    SHELL_CMD_ARG(_cmd, NULL, _help1 "\n" _help2, _func, 1, 10)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_lx16a,
    SUB_SHELL_CMD_ARG(init, lx16a_cmd_init, LX16A_INIT_SHELL_MSG, LX16A_INIT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(exit, lx16a_cmd_exit, LX16A_EXIT_SHELL_MSG, LX16A_EXIT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(en, lx16a_cmd_en, LX16A_EN_SHELL_MSG, LX16A_EN_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(get, lx16a_cmd_get_angle, LX16A_GET_SHELL_MSG, LX16A_GET_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(set, lx16a_cmd_set_angle, LX16A_SET_SHELL_MSG, LX16A_SET_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(status, lx16a_cmd_status, LX16A_STATUS_SHELL_MSG, LX16A_STATUS_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(dbgrd, lx16a_cmd_dbgrd, LX16A_DBGRD_SHELL_MSG, LX16A_DBGRD_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(dbgwr, lx16a_cmd_dbgwr, LX16A_DBGWR_SHELL_MSG, LX16A_DBGWR_SHELL_DETAILS),
	SHELL_SUBCMD_SET_END // Array terminator, must be last
);

SHELL_CMD_REGISTER(lx, &sub_lx16a, LX16A_CMD_HELP_MSG, NULL);

#else // Non-Zephyr Build

static void lx16a_shell_help(void)
{
    shell_print(NULL, LX16A_CMD_HELP_MSG);
    shell_print(NULL, "  init     : " LX16A_INIT_SHELL_MSG);
    shell_print(NULL, "             " LX16A_INIT_SHELL_DETAILS);
    shell_print(NULL, "  exit     : " LX16A_EXIT_SHELL_MSG);
    shell_print(NULL, "             " LX16A_EXIT_SHELL_DETAILS);
    shell_print(NULL, "  en       : " LX16A_EN_SHELL_MSG);
    shell_print(NULL, "             " LX16A_EN_SHELL_DETAILS);
    shell_print(NULL, "  get      : " LX16A_GET_SHELL_MSG);
    shell_print(NULL, "             " LX16A_GET_SHELL_DETAILS);
    shell_print(NULL, "  set      : " LX16A_SET_SHELL_MSG);
    shell_print(NULL, "             " LX16A_SET_SHELL_DETAILS);
    shell_print(NULL, "  status   : " LX16A_STATUS_SHELL_MSG);
    shell_print(NULL, "             " LX16A_STATUS_SHELL_DETAILS);
    shell_print(NULL, "  dbgrd    : " LX16A_DBGRD_SHELL_MSG);
    shell_print(NULL, "             " LX16A_DBGRD_SHELL_DETAILS);
    shell_print(NULL, "  dbgwr    : " LX16A_DBGWR_SHELL_MSG);
    shell_print(NULL, "             " LX16A_DBGWR_SHELL_DETAILS);
}

#define HANDLE_CMD(_name, _func) \
    else if (strcmp(argv[0], _name) == 0) { \
        (void) _func(sh, argc, argv); \
        rc = 0; \
    }

static int lx16a_shell_cmd_handler(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -ENOEXEC;

    if (argc == 0) {
        lx16a_shell_help();
    }
    HANDLE_CMD("init", lx16a_cmd_init)
    HANDLE_CMD("exit", lx16a_cmd_exit)
    HANDLE_CMD("en", lx16a_cmd_en)
    HANDLE_CMD("get", lx16a_cmd_get_angle)
    HANDLE_CMD("set", lx16a_cmd_set_angle)
    HANDLE_CMD("status", lx16a_cmd_status)
    HANDLE_CMD("dbgrd", lx16a_cmd_dbgrd)
    HANDLE_CMD("dbgwr", lx16a_cmd_dbgwr)

    return rc;
}

void lx16a_reg_shell_cmds(void)
{
    shell_nz_reg_cmd("lx", lx16a_shell_cmd_handler, LX16A_CMD_HELP_MSG, lx16a_shell_help);
}

#endif // Non-Zephyr Build
