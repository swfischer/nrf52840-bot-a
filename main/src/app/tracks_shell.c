/*****************************************************************************
 * tracks_shell.c - The source to shell commands for the tracks module
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "dcme.h"
#include "tracks.h"
#include "shell_.h"

static int trk_cmd_move(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        if (argc < 2) {
            shell_print(sh, "Invalid command");
            break;
        }
    
        int32_t cm = strtol(argv[1], NULL, 0);

        int ret = tracks_move(cm);
        if (ret != 0) {
            shell_print(sh, "Move failed");
            break;
        }

        shell_print(sh, "Success");
        rc = 0;

    } while (0);

    return rc;
}

static int trk_cmd_pivot(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        if (argc < 2) {
            shell_print(sh, "Invalid command");
            break;
        }
    
        int32_t degrees = strtol(argv[1], NULL, 0);

        int ret = tracks_pivot(degrees);
        if (ret != 0) {
            shell_print(sh, "Pivot failed");
            break;
        }

        shell_print(sh, "Success");
        rc = 0;

    } while(0);

    return rc;
}

static int trk_cmd_status(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "stopped ..........: %s", (tracks_is_stopped()) ? "yes" : "no");
    shell_print(sh, "system error .....: %d", tracks_error_state());
    shell_print(sh, "curr/last move ...: %d cm", tracks_last_move_cm());
    shell_print(sh, "curr/last pivot ..: %d deg", tracks_last_pivot_deg());

    return 0;
}

static int trk_cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
    tracks_stop();
    shell_print(sh, "Success");

    return 0;
}

#define TRK_CMD_HELP_MSG "Tracks Commands"
#define TRK_MOVE_SHELL_MSG "Start forward/reverse movement"
#define TRK_MOVE_SHELL_DETAILS "Usage: trk move <centimeter>"
#define TRK_PIVOT_SHELL_MSG "Start Pivot CW or CCW"
#define TRK_PIVOT_SHELL_DETAILS "Usage: trk pivot <degrees>"
#define TRK_STATUS_SHELL_MSG "Get tracks status"
#define TRK_STATUS_SHELL_DETAILS "Usage: trk status"
#define TRK_STOP_SHELL_MSG "Stop all movement"
#define TRK_STOP_SHELL_DETAILS "Usage: trk stop"

#if defined(ZEPHYR_BUILD)

#define SUB_SHELL_CMD_ARG(_cmd, _func, _help1, _help2) \
    SHELL_CMD_ARG(_cmd, NULL, _help1 "\n" _help2, _func, 1, 10)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_trk,
	SUB_SHELL_CMD_ARG(move, trk_cmd_move, TRK_MOVE_SHELL_MSG, TRK_MOVE_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(pivot, trk_cmd_pivot, TRK_PIVOT_SHELL_MSG, TRK_PIVOT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(status, trk_cmd_status, TRK_STATUS_SHELL_MSG, TRK_STATUS_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(stop, trk_cmd_stop, TRK_STOP_SHELL_MSG, TRK_STOP_SHELL_DETAILS),
	SHELL_SUBCMD_SET_END // Array terminator, must be last
);

SHELL_CMD_REGISTER(trk, &sub_trk, TRK_CMD_HELP_MSG, NULL);

#else // Non-Zephyr Build
#error "Non-Zephyr Shell is Not Supported"
#endif // Non-Zephyr Build
