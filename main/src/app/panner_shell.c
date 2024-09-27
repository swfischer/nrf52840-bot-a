/*****************************************************************************
 * panner_shell.c - The source to shell commands for the Panner module.
 *****************************************************************************/

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "panner.h"
#include "shell_.h"

// Comment out the following line to remove the "init" and "exit" commands
//#define ALLOW_INIT_EXIT_CMDS (1)

static bool panner_initialized = false;

static int panner_cmd_init(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (panner_initialized) {
		shell_info(sh, "Already init'd");
    } else {
#if defined(ALLOW_INIT_EXIT_CMDS)
        int ret = panner_init();
        if (ret < 0) {
            shell_warn(sh, "Init failed");
        } else {
            shell_print(sh, "Success");
            panner_initialized = true;
            rc = 0;
        }
#else
        shell_info(sh, "Assuming its init'd\n");
        panner_initialized = true;
        rc = 0;
#endif
    }

    return rc;
}

static int panner_cmd_exit(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (!panner_initialized) {
		shell_info(sh, "Driver already exited");
    } else {
#if defined(ALLOW_INIT_EXIT_CMDS)
        panner_exit();
#endif
        panner_initialized = false;
        rc = 0;
    }

    return rc;
}

static int panner_cmd_en(const struct shell *sh, size_t argc, char **argv)
{
    static const uint8_t ARGV_EN = 1;

    int rc = -1;

    if (!panner_initialized) {
		shell_warn(sh, "Driver not init'd");
    } else if (argc <= ARGV_EN) {
		shell_warn(sh, "Wrong parameter count");
    } else {
        uint8_t en = (uint8_t) strtoul(argv[ARGV_EN], NULL, 0);

        int ret = panner_enable((en != 0));
        if (ret != 0) {
            shell_print(sh, "Failure");
        } else {
            shell_print(sh, "Success");
        }
    }

    return rc;
}

static int panner_cmd_get_position(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (!panner_initialized) {
		shell_warn(sh, "Driver not init'd");
    } else {
        int16_t pos = 0;

        rc = panner_get_position(&pos);
        if (rc != 0) {
            shell_print(sh, "Failure");
        } else {
            shell_print(sh, "Success");
            shell_print(sh, "Position: %d", pos);
        }
    }

    return rc;
}

static int panner_cmd_set_position(const struct shell *sh, size_t argc, char **argv)
{
    static const uint8_t ARGV_POSITION = 1;

    int rc = -1;

    if (!panner_initialized) {
		shell_warn(sh, "Driver not init'd");
    } else if (argc <= ARGV_POSITION) {
		shell_warn(sh, "Wrong parameter count");
    } else {
        int16_t pos = strtol(argv[ARGV_POSITION], NULL, 0);

        rc = panner_set_position(pos);
        if (rc != 0) {
            shell_print(sh, "Failure");
        } else {
            shell_print(sh, "Success");
        }
    }

    return rc;
}

#define PANNER_CMD_HELP_MSG "Panner Commands"
#define PANNER_INIT_SHELL_MSG "Initialize the panner module"
#define PANNER_INIT_SHELL_DETAILS "Usage: pan init"
#define PANNER_EXIT_SHELL_MSG "Close/exit the panner module"
#define PANNER_EXIT_SHELL_DETAILS "Usage: pan exit"
#define PANNER_EN_SHELL_MSG "Enable (1) or disable (0) the panner servo(s)"
#define PANNER_EN_SHELL_DETAILS "Usage: pan en <1|0>"
#define PANNER_GET_SHELL_MSG "Get panner position"
#define PANNER_GET_SHELL_DETAILS "Usage: pan get"
#define PANNER_SET_SHELL_MSG "Set panner position to <pos>"
#define PANNER_SET_SHELL_DETAILS "Usage: pan set <ang>"

#if defined(ZEPHYR_BUILD)

#define SUB_SHELL_CMD_ARG(_cmd, _func, _help1, _help2) \
    SHELL_CMD_ARG(_cmd, NULL, _help1 "\n" _help2, _func, 1, 10)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_panner,
    SUB_SHELL_CMD_ARG(init, panner_cmd_init, PANNER_INIT_SHELL_MSG, PANNER_INIT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(exit, panner_cmd_exit, PANNER_EXIT_SHELL_MSG, PANNER_EXIT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(en, panner_cmd_en, PANNER_EN_SHELL_MSG, PANNER_EN_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(get, panner_cmd_get_position, PANNER_GET_SHELL_MSG, PANNER_GET_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(set, panner_cmd_set_position, PANNER_SET_SHELL_MSG, PANNER_SET_SHELL_DETAILS),
	SHELL_SUBCMD_SET_END // Array terminator, must be last
);

SHELL_CMD_REGISTER(pan, &sub_panner, PANNER_CMD_HELP_MSG, NULL);

#else // Non-Zephyr Build

static void panner_shell_help(void)
{
    shell_print(NULL, PANNER_CMD_HELP_MSG);
    shell_print(NULL, "  init     : " PANNER_INIT_SHELL_MSG);
    shell_print(NULL, "             " PANNER_INIT_SHELL_DETAILS);
    shell_print(NULL, "  exit     : " PANNER_EXIT_SHELL_MSG);
    shell_print(NULL, "             " PANNER_EXIT_SHELL_DETAILS);
    shell_print(NULL, "  en       : " PANNER_EN_SHELL_MSG);
    shell_print(NULL, "             " PANNER_EN_SHELL_DETAILS);
    shell_print(NULL, "  get      : " PANNER_GET_SHELL_MSG);
    shell_print(NULL, "             " PANNER_GET_SHELL_DETAILS);
    shell_print(NULL, "  set      : " PANNER_SET_SHELL_MSG);
    shell_print(NULL, "             " PANNER_SET_SHELL_DETAILS);
}

#define HANDLE_CMD(_name, _func) \
    else if (strcmp(argv[0], _name) == 0) { \
        (void) _func(sh, argc, argv); \
        rc = 0; \
    }

static int panner_shell_cmd_handler(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -ENOEXEC;

    if (argc == 0) {
        panner_shell_help();
    }
    HANDLE_CMD("init", panner_cmd_init)
    HANDLE_CMD("exit", panner_cmd_exit)
    HANDLE_CMD("en", panner_cmd_en)
    HANDLE_CMD("get", panner_cmd_get_position)
    HANDLE_CMD("set", panner_cmd_set_position)

    return rc;
}

void panner_reg_shell_cmds(void)
{
    shell_nz_reg_cmd("pan", panner_shell_cmd_handler, PANNER_CMD_HELP_MSG, panner_shell_help);
}

#endif // Non-Zephyr Build
