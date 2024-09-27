/*****************************************************************************
 * shutdown_shell.c - The source to shell commands for application shutdown
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "shell_.h"
#include "sign_of_life.h"

static int shutdown_now_cmd(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        shell_print(sh, "Shutting down");

        sign_of_life_end();

        shell_stop(sh);

        // Weirdly the shell still keeps operating, so just loop forever here
        while (1) {
            sleep(5);
        }

    } while (0);

    return rc;
}

#define DT_CMD_HELP_MSG "Shutdown Command"
#define DT_FWD_SHUTDOWN_MSG "Shutdown the application"
#define DT_FWD_SHUTDOWN_DETAILS "Usage: shutdown now"

#if defined(ZEPHYR_BUILD)

#define SUB_SHELL_CMD_ARG(_cmd, _func, _help1, _help2) \
    SHELL_CMD_ARG(_cmd, NULL, _help1 "\n" _help2, _func, 1, 10)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_shutdown,
	SUB_SHELL_CMD_ARG(now, shutdown_now_cmd, DT_FWD_SHUTDOWN_MSG, DT_FWD_SHUTDOWN_DETAILS),
	SHELL_SUBCMD_SET_END // Array terminator, must be last
);

SHELL_CMD_REGISTER(shutdown, &sub_shutdown, DT_CMD_HELP_MSG, NULL);

#else // Non-Zephyr Build
#error "Non-Zephyr Shell is Not Supported"
#endif // Non-Zephyr Build
