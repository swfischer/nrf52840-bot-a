/*****************************************************************************
 * mapper_shell.c - The source to shell commands for the mapper application.
 *****************************************************************************/

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mapper.h"
#include "shell_.h"

static sweep_data_t mapper_sweep_data;

static int mapper_cmd_start(const struct shell *sh, size_t argc, char **argv)
{
    mapper_start_sweep(MAP_TYPE_FULL, &mapper_sweep_data);

    return 0;
}

static int mapper_cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
    uint16_t cnt = mapper_sweep_data.count;

    printf("Data Points ........: %u\n", cnt);
    for (int idx = 0; idx < cnt; idx++) {
        float meters = mapper_sweep_data.meters[idx];
        printf("Point[%d]: %d, %d.%03d\n", idx, mapper_sweep_data.angle[idx], (int)meters, (((int)(meters * 1000)) % 1000));
    }

    return 0;
}

#define MAPPER_CMD_HELP_MSG "Mapper Commands"
#define MAPPER_START_SHELL_MSG "Start a sweep"
#define MAPPER_START_SHELL_DETAILS "Usage: map start"
#define MAPPER_DUMP_SHELL_MSG "Dump data for the last sweep"
#define MAPPER_DUMP_SHELL_DETAILS "Usage: map dump"

#if defined(ZEPHYR_BUILD)

#define SUB_SHELL_CMD_ARG(_cmd, _func, _help1, _help2) \
    SHELL_CMD_ARG(_cmd, NULL, _help1 "\n" _help2, _func, 1, 10)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_mapper,
	SUB_SHELL_CMD_ARG(start, mapper_cmd_start, MAPPER_START_SHELL_MSG, MAPPER_START_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(dump, mapper_cmd_dump, MAPPER_DUMP_SHELL_MSG, MAPPER_DUMP_SHELL_DETAILS),
	SHELL_SUBCMD_SET_END // Array terminator, must be last
);

SHELL_CMD_REGISTER(map, &sub_mapper, MAPPER_CMD_HELP_MSG, NULL);

#else // Non-Zephyr Build

static void mapper_shell_help(void)
{
    shell_print(NULL, MAPPER_CMD_HELP_MSG);
    shell_print(NULL, "  start    : " MAPPER_START_SHELL_MSG);
    shell_print(NULL, "             " MAPPER_START_SHELL_DETAILS);
    shell_print(NULL, "  dump     : " MAPPER_DUMP_SHELL_MSG);
    shell_print(NULL, "             " MAPPER_DUMP_SHELL_DETAILS);
}

#define HANDLE_CMD(_name, _func) \
    else if (strcmp(argv[0], _name) == 0) { \
        (void) _func(sh, argc, argv); \
        rc = 0; \
    }

static int mapper_shell_cmd_handler(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -ENOEXEC;

    if (argc == 0) {
        mapper_shell_help();
    }
    HANDLE_CMD("start", mapper_cmd_start)
    HANDLE_CMD("dump", mapper_cmd_dump)

    return rc;
}

void mapper_reg_shell_cmds(void)
{
    shell_nz_reg_cmd("map", mapper_shell_cmd_handler, MAPPER_CMD_HELP_MSG, mapper_shell_help);
}

#endif // Non-Zephyr Build
