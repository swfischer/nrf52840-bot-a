/*****************************************************************************
 * drivetrain_shell.c - The source to shell commands for the drivetrain module
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "dcme.h"
#include "drivetrain.h"
#include "shell_.h"

#define DT_SHELL_MOTION_DURATION (3)

static int dcme_hndl_left = -1;
static int dcme_hndl_right = -1;

static int dt_cmd_fwd(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        if (argc < 2) {
            shell_print(sh, "Invalid command");
            break;
        }
    
        uint32_t percent = strtoul(argv[1], NULL, 0);

        if (percent > 100) {
            shell_print(sh, "Invalid speed");
            break;
        }

        int16_t min_ecps;
        int16_t max_ecps;
        drivetrain_get_motion_limits(&min_ecps, &max_ecps);
        int16_t speed = (max_ecps * percent) / 100;

        int ret = drivetrain_set_motion(speed, speed);
        if (ret != 0) {
            shell_print(sh, "Forward motion failed");
            break;
        }

        sleep(DT_SHELL_MOTION_DURATION);

        drivetrain_stop();

        shell_print(sh, "Success");
        rc = 0;

    } while (0);

    return rc;
}

static int dt_cmd_left(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        if (argc < 2) {
            shell_print(sh, "Invalid command");
            break;
        }
    
        uint32_t percent = strtoul(argv[1], NULL, 0);

        if (percent > 100) {
            shell_print(sh, "Invalid speed");
            break;
        }

        int16_t min_ecps;
        int16_t max_ecps;
        drivetrain_get_motion_limits(&min_ecps, &max_ecps);
        int16_t speed = (max_ecps * percent) / 100;

        int ret = drivetrain_set_motion(-speed, speed);
        if (ret != 0) {
            shell_print(sh, "Left motion failed");
            break;
        }

        sleep(DT_SHELL_MOTION_DURATION);

        drivetrain_stop();

        shell_print(sh, "Success");
        rc = 0;

    } while(0);

    return rc;
}

static int dt_cmd_rev(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        if (argc < 2) {
            shell_print(sh, "Invalid command");
            break;
        }
    
        uint32_t percent = strtoul(argv[1], NULL, 0);

        if (percent > 100) {
            shell_print(sh, "Invalid speed");
            break;
        }

        int16_t min_ecps;
        int16_t max_ecps;
        drivetrain_get_motion_limits(&min_ecps, &max_ecps);
        int16_t speed = (max_ecps * percent) / 100;

        int ret = drivetrain_set_motion(-speed, -speed);
        if (ret != 0) {
            shell_print(sh, "Reverse motion failed");
            break;
        }

        sleep(DT_SHELL_MOTION_DURATION);

        drivetrain_stop();

        shell_print(sh, "Success");
        rc = 0;

    } while(0);

    return rc;
}

static int dt_cmd_right(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    do { // Using do/while(0) as a quick escape mechanism

        if (argc < 2) {
            shell_print(sh, "Invalid command");
            break;
        }
    
        uint32_t percent = strtoul(argv[1], NULL, 0);

        if (percent > 100) {
            shell_print(sh, "Invalid speed");
            break;
        }

        int16_t min_ecps;
        int16_t max_ecps;
        drivetrain_get_motion_limits(&min_ecps, &max_ecps);
        int16_t speed = (max_ecps * percent) / 100;

        int ret = drivetrain_set_motion(speed, -speed);
        if (ret != 0) {
            shell_print(sh, "Right motion failed");
            break;
        }

        sleep(DT_SHELL_MOTION_DURATION);

        drivetrain_stop();

        shell_print(sh, "Success");
        rc = 0;

    } while(0);

    return rc;
}

static int dt_cmd_status(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (dcme_hndl_left == -1) {
        dcme_hndl_left = drivetrain_get_dcme_handle(true);
        if (dcme_hndl_left == -1) {
            shell_warn(sh, "Left handle get failed");
        }
    }

    if (dcme_hndl_right == -1) {
        dcme_hndl_right = drivetrain_get_dcme_handle(false);
        if (dcme_hndl_right == -1) {
            shell_warn(sh, "Right handle get failed");
        }
    }

    if ((dcme_hndl_left >= 0) && (dcme_hndl_right >= 0)) {
        dcme_status_t left_status;
        dcme_status_t right_status;

        if (dcme_get_status(dcme_hndl_left, &left_status) != 0) {
            shell_warn(sh, "Left status get failed");
        } else if (dcme_get_status(dcme_hndl_right, &right_status) != 0) {
            shell_warn(sh, "Right status get failed");
        } else {
            int16_t min_ecps;
            int16_t max_ecps;
            drivetrain_get_motion_limits(&min_ecps, &max_ecps);
            shell_print(sh, "ecps min ..........: %d", min_ecps);
            shell_print(sh, "ecps max ..........: %d", max_ecps);
            shell_print(sh, "Left:");
            shell_print(sh, "state .............: %u", left_status.state);
            float t = left_status.exp_enc_cnt_per_refresh;
            shell_print(sh, "exp_enc_cnt_per_ ..: %d.%d", (int)t, ((int)(t * 1000)) % 1000);
            shell_print(sh, "enc_cnt ...........: %u", left_status.enc_cnt);
            shell_print(sh, "enc_cnt_capture ...: %u", left_status.enc_cnt_capture);
            shell_print(sh, "enc_cnt_delta .....: %d", left_status.enc_cnt_delta);
            shell_print(sh, "enc_cnt @ 0 spd ...: %d (delta = %d)",
                            left_status.enc_cnt_at_spd_zero,
                            (left_status.enc_cnt - left_status.enc_cnt_at_spd_zero));
            shell_print(sh, "enc_cnt_failures ..: %u", left_status.enc_cnt_failures);
            shell_print(sh, "enc_isr_failures ..: %u", left_status.enc_isr_failures);
            t = left_status.pid_set_point;
            shell_print(sh, "pid_set_point .....: %d.%d", (int)t, ((int)(t * 1000)) % 1000);
            t = left_status.pid_output;
            shell_print(sh, "pid_output ........: %d.%d", (int)t, ((int)(t * 1000)) % 1000);
            shell_print(sh, "pid_failures ......: %u", left_status.pid_failures);
            shell_print(sh, "pwm_failures ......: %u", left_status.pwm_failures);
            shell_print(sh, "Right:");
            shell_print(sh, "state .............: %u", right_status.state);
            t = right_status.exp_enc_cnt_per_refresh;
            shell_print(sh, "exp_enc_cnt_per_ ..: %d.%d", (int)t, ((int)(t * 1000)) % 1000);
            shell_print(sh, "enc_cnt ...........: %u", right_status.enc_cnt);
            shell_print(sh, "enc_cnt_capture ...: %u", right_status.enc_cnt_capture);
            shell_print(sh, "enc_cnt_delta .....: %d", right_status.enc_cnt_delta);
            shell_print(sh, "enc_cnt @ 0 spd ...: %d (delta = %d)",
                            right_status.enc_cnt_at_spd_zero,
                            (right_status.enc_cnt - right_status.enc_cnt_at_spd_zero));
            shell_print(sh, "enc_cnt_failures ..: %u", right_status.enc_cnt_failures);
            shell_print(sh, "enc_isr_failures ..: %u", right_status.enc_isr_failures);
            t = right_status.pid_set_point;
            shell_print(sh, "pid_set_point .....: %d.%d", (int)t, ((int)(t * 1000)) % 1000);
            t = right_status.pid_output;
            shell_print(sh, "pid_output ........: %d.%d", (int)t, ((int)(t * 1000)) % 1000);
            shell_print(sh, "pid_failures ......: %u", right_status.pid_failures);
            shell_print(sh, "pwm_failures ......: %u", right_status.pwm_failures);
            rc = 0;
        }
    }

    return rc;
}

static int dt_cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
    drivetrain_stop();
    shell_print(sh, "Success");

    return 0;
}

static int dt_cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
    if (dcme_hndl_left == -1) {
        dcme_hndl_left = drivetrain_get_dcme_handle(true);
        if (dcme_hndl_left == -1) {
            shell_warn(sh, "Left handle get failed");
        }
    }

    if (dcme_hndl_right == -1) {
        dcme_hndl_right = drivetrain_get_dcme_handle(false);
        if (dcme_hndl_right == -1) {
            shell_warn(sh, "Right handle get failed");
        }
    }

    if ((dcme_hndl_left >= 0) && (dcme_hndl_right >= 0)) {
        dcme_dbg_data_t *left_data;
        dcme_dbg_data_t *right_data;

        if (dcme_get_dbg_data(dcme_hndl_left, &left_data) != 0) {
            shell_warn(sh, "Left get data failed");
        } else if (dcme_get_dbg_data(dcme_hndl_right, &right_data) != 0) {
            shell_warn(sh, "Right get data failed");
        } else {
            uint16_t cnt = (left_data->cnt > right_data->cnt) ? right_data->cnt : left_data->cnt;
            uint16_t l = (left_data->cnt < DCME_DBG_DATA_SIZE) ? 0 : left_data->head;
            uint16_t r = (right_data->cnt < DCME_DBG_DATA_SIZE) ? 0 : right_data->head;

            shell_print(sh, "ts, ramp_cnt, left_enc_cnt, left_enc_delta, left_pid_setpt, left_pid_out, right_enc_cnt, right_enc_delta, right_pid_setpt, right_pid_out");
            for (uint16_t i = 0; i < cnt; i++) {
                shell_print(sh, "%u, %u, %u, %u, %d.%d, %d.%d, %u, %u, %d.%d, %d.%d",
                            left_data->ts[l],
                            left_data->ramp_cnt[l],
                            left_data->enc_cnt[l], 
                            left_data->enc_delta[l], 
                            (int)left_data->pid_setpt[l],
                            ((int)(left_data->pid_setpt[l] * 1000)) % 1000,
                            (int)left_data->pid_out[l],
                            ((int)(left_data->pid_out[l] * 1000)) % 1000,
                            right_data->enc_cnt[r],
                            right_data->enc_delta[r],
                            (int)right_data->pid_setpt[r],
                            ((int)(right_data->pid_setpt[l] * 1000)) % 1000,
                            (int)right_data->pid_out[r],
                            ((int)(right_data->pid_out[l] * 1000)) % 1000);
                l++;
                r++;
                if (l >= DCME_DBG_DATA_SIZE) {
                    l = 0;
                }
                if (r >= DCME_DBG_DATA_SIZE) {
                    r = 0;
                }
            }
        }
    }

    return 0;
}

#define DT_CMD_HELP_MSG "Drivetrain Commands"
#define DT_FWD_SHELL_MSG "Start forward movement"
#define DT_FWD_SHELL_DETAILS "Usage: dt fwd <percent>"
#define DT_LEFT_SHELL_MSG "Start left pivot movement"
#define DT_LEFT_SHELL_DETAILS "Usage: dt left <percent>"
#define DT_REV_SHELL_MSG "Start reverse movement"
#define DT_REV_SHELL_DETAILS "Usage: dt rev <percent>"
#define DT_RIGHT_SHELL_MSG "Start right pivot movement"
#define DT_RIGHT_SHELL_DETAILS "Usage: dt right <percent>"
#define DT_STATUS_SHELL_MSG "Get drivetrain status"
#define DT_STATUS_SHELL_DETAILS "Usage: dt status"
#define DT_STOP_SHELL_MSG "Stop all movement"
#define DT_STOP_SHELL_DETAILS "Usage: dt stop"
#define DT_DUMP_SHELL_MSG "Dump debug motor movement details (if available)"
#define DT_DUMP_SHELL_DETAILS "Usage: dt dump"

#if defined(ZEPHYR_BUILD)

#define SUB_SHELL_CMD_ARG(_cmd, _func, _help1, _help2) \
    SHELL_CMD_ARG(_cmd, NULL, _help1 "\n" _help2, _func, 1, 10)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dt,
	SUB_SHELL_CMD_ARG(fwd, dt_cmd_fwd, DT_FWD_SHELL_MSG, DT_FWD_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(left, dt_cmd_left, DT_LEFT_SHELL_MSG, DT_LEFT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(rev, dt_cmd_rev, DT_REV_SHELL_MSG, DT_REV_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(right, dt_cmd_right, DT_RIGHT_SHELL_MSG, DT_RIGHT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(status, dt_cmd_status, DT_STATUS_SHELL_MSG, DT_STATUS_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(stop, dt_cmd_stop, DT_STOP_SHELL_MSG, DT_STOP_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(dump, dt_cmd_dump, DT_DUMP_SHELL_MSG, DT_DUMP_SHELL_DETAILS),
	SHELL_SUBCMD_SET_END // Array terminator, must be last
);

SHELL_CMD_REGISTER(dt, &sub_dt, DT_CMD_HELP_MSG, NULL);

#else // Non-Zephyr Build
#error "Non-Zephyr Shell is Not Supported"
#endif // Non-Zephyr Build
