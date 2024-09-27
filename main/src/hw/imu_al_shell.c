/*****************************************************************************
 * imu_al_shell.c - The source to shell commands for IMU sensors
 *****************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>

#include "imu_al.h"
#include "os_al.h"
#include "shell_.h"

// Comment out the following line to remove the "init" and "exit" commands
//#define ALLOW_INIT_EXIT_CMDS (1)

#if defined(ZEPHYR_BUILD)
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#if defined(ALLOW_INIT_EXIT_CMDS)
static const struct device *const ism330dhcx = DEVICE_DT_GET_ONE(st_ism330dhcx);
#endif
#elif defined(MAC_BUILD)
// TBD
#elif defined(WINDOWS_BUILD)
// TBD
#endif

static int imu_al_hndl = -1;

//! Timer to perform gyro test
static os_al_timer_t gyro_test_cycle_timer_hndl = -1;
//! Worker used to handle timer expirations
static struct k_work gyro_test_expiry_worker;
static int32_t gyro_test_remaining_test_ms = 0;
static uint16_t gyro_test_cycle_ms = 0;

static void expiry_work_handler(struct k_work *work);
#define FLOAT_TO_CHAR_BUF_SIZE (10)
#define FLOAT_TO_CHAR_DECIMAL_PTS (3)
#define FLOAT_TO_CHAR_MAGNATUDE (1000)
static char* float_to_char(float x, char *p);
static void timer_expired_cb(os_al_timer_t timer);

static int imu_al_cmd_init(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (imu_al_hndl != -1) {
		shell_info(sh, "Driver already init'd");
    } else {
#if defined(ALLOW_INIT_EXIT_CMDS)
        imu_al_hw_t hw;
        hw.imu_dev = ism330dhcx;

        int hndl = imu_al_init(&hw);
        if (hndl < 0) {
            shell_warn(sh, "Init failed");
        } else {
            shell_print(sh, "Success");
            imu_al_hndl = hndl;
            rc = 0;
        }
#else
        shell_info(sh, "Assuming FD = 0\n");
        imu_al_hndl = 0;
        rc = 0;
#endif
    }

    return rc;
}

static int imu_al_cmd_exit(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (imu_al_hndl == -1) {
		shell_info(sh, "Driver already exited");
    } else {
#if defined(ALLOW_INIT_EXIT_CMDS)
        imu_al_exit(imu_al_hndl);
#endif
        imu_al_hndl = -1;
        rc = 0;
    }

    return rc;
}

static int imu_al_cmd_gyro_test(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (imu_al_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else {
        uint16_t secs = strtoul(argv[1], NULL, 0);
        uint16_t cycle_ms = strtoul(argv[2], NULL, 0);

        if (gyro_test_cycle_timer_hndl == -1) {
            gyro_test_cycle_timer_hndl = os_al_timer_create(timer_expired_cb);
            if (gyro_test_cycle_timer_hndl >= 0) {
                k_work_init(&gyro_test_expiry_worker, expiry_work_handler);
            }
        }

        if (gyro_test_cycle_timer_hndl >= 0) {
            gyro_test_remaining_test_ms = secs * 1000;
            gyro_test_cycle_ms = cycle_ms;
            rc = os_al_timer_start(gyro_test_cycle_timer_hndl, cycle_ms, TIMER_TYPE_PERIODIC);
        }
    }

    return rc;
}

static int imu_al_cmd_status(const struct shell *sh, size_t argc, char **argv)
{
    int rc = -1;

    if (imu_al_hndl == -1) {
		shell_warn(sh, "Driver not init'd");
    } else {
        imu_accel_t accel;
        imu_gyro_t gyro;
        imu_gyro_int_t gyro_int;

        if (imu_al_read(imu_al_hndl, &accel, &gyro) != 0) {
		    shell_print(sh, "Read call failed");
        } else if (imu_al_get_gyro_int(imu_al_hndl, &gyro_int) != 0) {
		    shell_print(sh, "Get call failed");
        } else {
            char buf_x[FLOAT_TO_CHAR_BUF_SIZE];
            char buf_y[FLOAT_TO_CHAR_BUF_SIZE];
            char buf_z[FLOAT_TO_CHAR_BUF_SIZE];
            shell_print(sh, "Accel X/Y/Z ....: %s, %s, %s"
                , float_to_char(accel.x_mpss, buf_x)
                , float_to_char(accel.y_mpss, buf_y)
                , float_to_char(accel.z_mpss, buf_z));
            shell_print(sh, "Gyro X/Y/Z .....: %s, %s, %s"
                , float_to_char(gyro.x_dps, buf_x)
                , float_to_char(gyro.y_dps, buf_y)
                , float_to_char(gyro.z_dps, buf_z));
            shell_print(sh, "Gyro Int X/Y/Z .: %s, %s, %s"
                , float_to_char(gyro_int.x_deg, buf_x)
                , float_to_char(gyro_int.y_deg, buf_y)
                , float_to_char(gyro_int.z_deg, buf_z));
            rc = 0;
        }
    }

    return rc;
}

static void expiry_work_handler(struct k_work *work)
{
    static uint32_t log_ms = 0;
    imu_gyro_t gyro;
    imu_gyro_int_t gyro_int;

    int ret = imu_al_gyro_read(imu_al_hndl, &gyro, &gyro_int);
    if (ret == 0) {
        log_ms += gyro_test_cycle_ms;
        if (log_ms > 500) {
            log_ms = 0;
            char buf_x[FLOAT_TO_CHAR_BUF_SIZE];
            char buf_y[FLOAT_TO_CHAR_BUF_SIZE];
            char buf_z[FLOAT_TO_CHAR_BUF_SIZE];
            printf("[%u] %s, %s, %s\n"
                , gyro_test_remaining_test_ms
                , float_to_char(gyro_int.x_deg, buf_x)
                , float_to_char(gyro_int.y_deg, buf_y)
                , float_to_char(gyro_int.z_deg, buf_z));
        }
    }
}

// Here's a version optimized for embedded systems that doesn't require any
// stdio or memset, and has low memory footprint. You're responsible for
// passing a char buffer initialized with zeros (with pointer p) where you want
// to store your string, and defining FLOAT_TO_CHAR_BUF_SIZE when you make said buffer
// (so the returned string will be null terminated).

static char* float_to_char(float x, char *p)
{
    uint16_t decimals;  // variable to store the decimals
    int units;  // variable to store the units (part to left of decimal place)
    char *s = p + FLOAT_TO_CHAR_BUF_SIZE - 1; // go to end of buffer

    *--s = 0; // Terminate the buffer

    if (x < 0) { // take care of negative numbers
        decimals = (int)(x * -FLOAT_TO_CHAR_MAGNATUDE) % FLOAT_TO_CHAR_MAGNATUDE; // make 1000 for 3 decimals etc.
        units = (int)(-1 * x);
    } else { // positive numbers
        decimals = (int)(x * FLOAT_TO_CHAR_MAGNATUDE) % FLOAT_TO_CHAR_MAGNATUDE;
        units = (int)x;
    }

    for (int i = 0; i < FLOAT_TO_CHAR_DECIMAL_PTS; i++) {
        *--s = (decimals % 10) + '0';
        decimals /= 10;
    }
    *--s = '.';

    do {
        *--s = (units % 10) + '0';
        units /= 10;
    } while (units > 0);

    if (x < 0) {
        *--s = '-'; // unary minus sign for negative numbers
    }

    return s;
}

static void timer_expired_cb(os_al_timer_t timer)
{
    k_work_submit(&gyro_test_expiry_worker);
    gyro_test_remaining_test_ms -= gyro_test_cycle_ms;
    if (gyro_test_remaining_test_ms <= 0) {
        os_al_timer_cancel(gyro_test_cycle_timer_hndl);
    }
}

#define IMU_AL_CMD_HELP_MSG "IMU Commands"
#define IMU_AL_INIT_SHELL_MSG "Initialize an instance of the IMU"
#define IMU_AL_INIT_SHELL_DETAILS "Usage: imu init"
#define IMU_AL_EXIT_SHELL_MSG "Close/exit the instance of the IMU"
#define IMU_AL_EXIT_SHELL_DETAILS "Usage: imu exit"
#define IMU_AL_GYRO_SHELL_MSG "Run Gyro integration test"
#define IMU_AL_GYRO_SHELL_DETAILS "Usage: imu gyro <secs> <cycle_ms>"
#define IMU_AL_STATUS_SHELL_MSG "Get IMU status"
#define IMU_AL_STATUS_SHELL_DETAILS "Usage: imu status"

#if defined(ZEPHYR_BUILD)

#define SUB_SHELL_CMD_ARG(_cmd, _func, _help1, _help2) \
    SHELL_CMD_ARG(_cmd, NULL, _help1 "\n" _help2, _func, 1, 10)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu,
    SUB_SHELL_CMD_ARG(init, imu_al_cmd_init, IMU_AL_INIT_SHELL_MSG, IMU_AL_INIT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(exit, imu_al_cmd_exit, IMU_AL_EXIT_SHELL_MSG, IMU_AL_EXIT_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(gyro, imu_al_cmd_gyro_test, IMU_AL_GYRO_SHELL_MSG, IMU_AL_GYRO_SHELL_DETAILS),
	SUB_SHELL_CMD_ARG(status, imu_al_cmd_status, IMU_AL_STATUS_SHELL_MSG, IMU_AL_STATUS_SHELL_DETAILS),
	SHELL_SUBCMD_SET_END // Array terminator, must be last
);

SHELL_CMD_REGISTER(imu, &sub_imu, IMU_AL_CMD_HELP_MSG, NULL);

#else // Non-Zephyr Build

// TBD

#endif // Non-Zephyr Build
