/*****************************************************************************
 * shell_.h - The interface to a common shell for Zephyr, MacOS, Windows
 * 
 * The point of this interface is to provide a common shell structure which
 * will run in Zephyr and non-Zephyr builds, like for MacOS or Windows.  This
 * structure follows the Zephyr style of APIs.
 * 
 * The "_" in the name "shell_.h" is just a means of differeniating it from
 * the Zephyr shell.h file.
 *****************************************************************************/

#ifndef SHELL__H
#define SHELL__H

#include <stdbool.h>
#include <string.h>

#if defined(ZEPHYR_BUILD)
#include <zephyr/shell/shell.h>
#else // Non-Zephyr Build
#define SHELL_ERROR (0)
#define SHELL_WARNING (1)
#define SHELL_INFO (2)
#define SHELL_NORMAL (3)

#define ENOEXEC (1)

// A structure to match the shell structure in Zephyr
struct shell {
    int not_used;
};

// Return 0 on success, otherwise -ENOEXEC
typedef int (*shell_cmd_handler) (const struct shell *sh, size_t argc, char **argv);

typedef void (*shell_cmd_help) (void);

#define shell_info(_sh, ...)  shell_nz_printf(_sh, SHELL_INFO, __VA_ARGS__)
#define shell_print(_sh, ...) shell_nz_printf(_sh, SHELL_NORMAL, __VA_ARGS__)
#define shell_warn(_sh, ...)  shell_nz_printf(_sh, SHELL_WARNING, __VA_ARGS__)
#define shell_error(_sh, ...) shell_nz_printf(_sh, SHELL_ERROR, __VA_ARGS__)

// Does not return until the shell is exited
extern void shell_nz_start(void);
// Call to register a command with the shell
extern void shell_nz_reg_cmd(const char *cmd, shell_cmd_handler handler, const char *help_msg, shell_cmd_help help_func);
// Shell print function
extern void shell_nz_printf(const struct shell *sh, int level, ...);
#endif // Non-Zephyr Build

#endif // SHELL__H
