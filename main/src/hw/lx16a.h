/*****************************************************************************
 * lx16a.h - The interface to communicate with LX-16A Serial Bus Servos
 * 
 * The point of this interface is to modularize the LX-16A servo code and
 * present an easy to use interface to the application.
 *****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#if defined(ZEPHYR_BUILD)
#include <zephyr/device.h>
#endif

#ifndef LX16A_H
#define LX16A_H

#define GET_UINT16_LSB(x) (x & 0x00FF)
#define GET_UINT16_MSB(x) ((x >> 8) & 0x00FF)
#define MAKE_UINT16(msb,lsb) ((((uint16_t)msb) << 8) + lsb) 

// The servo ID for broadcasting to all connected servos
#define LX16A_ID_BROADCAST (0xFE)

// Servo command list
#define LX16A_CMD_MOVE_TIME_WRITE (1)
#define LX16A_CMD_MOVE_TIME_READ (2)
#define LX16A_CMD_MOVE_TIME_WAIT_WRITE (7)
#define LX16A_CMD_MOVE_TIME_WAIT_READ (8)
#define LX16A_CMD_MOVE_START (11)
#define LX16A_CMD_MOVE_STOP (12)
#define LX16A_CMD_ID_WRITE (13)
#define LX16A_CMD_ID_READ (14)
#define LX16A_CMD_ANGLE_OFFSET_ADJUST (17)
#define LX16A_CMD_ANGLE_OFFSET_WRITE (18)
#define LX16A_CMD_ANGLE_OFFSET_READ (19)
#define LX16A_CMD_ANGLE_LIMIT_WRITE (20)
#define LX16A_CMD_ANGLE_LIMIT_READ (21)
#define LX16A_CMD_VIN_LIMIT_WRITE (22)
#define LX16A_CMD_VIN_LIMIT_READ (23)
#define LX16A_CMD_TEMP_MAX_LIMIT_WRITE (24)
#define LX16A_CMD_TEMP_MAX_LIMIT_READ (25)
#define LX16A_CMD_TEMP_READ (26)
#define LX16A_CMD_VIN_READ (27)
#define LX16A_CMD_POS_READ (28)
#define LX16A_CMD_SERVO_OR_MOTOR_MODE_WRITE (29)
#define LX16A_CMD_SERVO_OR_MOTOR_MODE_READ (30)
#define LX16A_CMD_LOAD_OR_UNLOAD_WRITE (31)
#define LX16A_CMD_LOAD_OR_UNLOAD_READ (32)
#define LX16A_CMD_LED_CTRL_WRITE (33)
#define LX16A_CMD_LED_CTRL_READ (34)
#define LX16A_CMD_LED_ERROR_WRITE (35)
#define LX16A_CMD_LED_ERROR_READ (36)

// The maximum number of parameters per command is 4
#define MAX_CMD_PARAM_CNT (4)

// Set/Get angle range
static const float LX16A_ANGLE_RANGE_DEG_MIN = 0.0f;
static const float LX16A_ANGLE_RANGE_DEG_MAX = 240.0f;
// Set/Get angle timing range
static const uint16_t LX16A_ANGLE_TIMING_RANGE_MS_MIN = 10;
static const uint16_t LX16A_ANGLE_TIMING_RANGE_MS_MAX = 30000;

// A structure intended to abtract the initialzation data as it changes based
// on the platform being compiled for.
typedef struct {
    uint32_t baud;
#if defined(ZEPHYR_BUILD)
    const struct device *uart;
#elif defined(MAC_BUILD)
    const char *uart;
#elif defined(WINDOWS_BUILD)
    const char *uart;
#endif
} lx16a_hardware_t;

// Returns an instance handle for the created driver on success, otherwise -1
extern int lx16a_init(const lx16a_hardware_t *hw);
extern void lx16a_exit(int hndl);

// Enable or disable all servos
// Return 0 on success, otherwise -1
extern int lx16a_set_all_enable_states(int hndl, bool enable);

// Get the current angle of a given servo (angle range from 0.0 to 240.0)
// Return the angle on success, otherwise < -0.1
extern float lx16a_get_angle(int hndl, uint8_t id);
// Set the angle of a given servo (angle range from 0.0 to 240.0)
// Return 0 on success, otherwise -1
extern int lx16a_set_angle(int hndl, uint8_t id, float angle, uint16_t time_ms);

// Returns 0 on success and statistics via passed pointers, otherwise -1
typedef struct {
    uint32_t lost; // Bytes lost (overwritten) in the UART driver
    uint32_t rd_retries; // Number of read retries that have occurred
    uint32_t rd_timeouts; // Number of read timeouts (waiting on data) that have occurred
    uint32_t rd_invalids; // Number of received packets that were invalid
    uint32_t rd_fails; // Number of times the read call returned a failure
    uint32_t wr_retries; // Number of write retries that have occurred
    uint32_t wr_fails; // Number of times the write call returned a failure
} lx16a_stats_t;
extern int lx16a_get_stats(int hndl, lx16a_stats_t *stats);

// These are really for debug and test purposes
// Returns 0 on success (and read parameters via params), otherwise -1
extern int lx16a_cmd_read(int hndl, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t paramCnt);
// Return 0 on success, otherwise -1
extern int lx16a_cmd_write(int hndl, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t paramCnt);

#endif // LX16A_H
