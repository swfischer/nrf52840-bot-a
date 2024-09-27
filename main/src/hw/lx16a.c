/*****************************************************************************
 * lx16a.c - The source to communicate with LX-16A serial bus servos
 * 
 * The point of this interface is to modularize the LX-16A servo code and
 * present an easy to use interface to the application.
 *****************************************************************************/

#include "lx16a.h"

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "logger.h"
#include "uart_al.h"

// Uncomment to output read/write packets
//#define DISPLAY_READ_WRITE_PACKETS

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

#define PACKET_HEADER_SIZE (6) // 6 is two 0x55, plus id, cmd, length, and checksum bytes
#define HEADER_MARKER (0x55)

//static const uint16_t TRANSLATED_ANGLE_RANGE_MIN = 0;
static const uint16_t TRANSLATED_ANGLE_RANGE_MAX = 1000;
static const float ANGLE_FACTOR = (LX16A_ANGLE_RANGE_DEG_MAX / TRANSLATED_ANGLE_RANGE_MAX);

//! This mutex is used to serialize public function calls
static pthread_mutex_t lx16a_api_lock = PTHREAD_MUTEX_INITIALIZER;

typedef struct
{
    bool initialized;
    bool enabled;
    uint8_t retries;
    int uart_hndl;

    lx16a_stats_t stats;

} lx16a_data_t;

#define MAX_NUMBER_OF_HANDLES (1)
static lx16a_data_t lx16a_hndl_data[MAX_NUMBER_OF_HANDLES] = {0};

// Returns 0 on success (and read parameters via params), otherwise -1
static int cmd_read(int hndl, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_cnt);
// Returns 0 on success, otherwise -1 
static int cmd_read_no_retry(int hndl, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_cnt);
// Returns 0 on success, otherwise -1 
static int cmd_write(int hndl, uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_cnt);
// Returns 0 on success, otherwise -1 
static int cmd_write_no_retry(int hndl, uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_cnt);
// Returns 0 on success, otherwise -1 
static int set_all_enable_states(int hndl, bool enable);
// Returns 0 on success, otherwise -1
static int setup_uart_device(const lx16a_hardware_t *hw, uart_al_device_t *dev);
// Returns true if valid receive msg
static bool verify_rcvd_pkt(uint8_t *buf, uint8_t id, uint8_t cmd, uint8_t param_cnt);

/*****************************************************************************
 * @brief lx16a_init - open an instance of the LX-16A driver
 * 
 * @details Performs initialization of an instance of the LX-16A driver
 *
 * @return Returns a file descriptor on success, otherwise -1
 *****************************************************************************/

int lx16a_init(const lx16a_hardware_t *hw)
{
    uart_al_device_t dev;
    int hndl = -1;
    int tmp_hndl = 0; // Default to the first and only handle
    int uart_hndl = -1;

    pthread_mutex_lock(&lx16a_api_lock);

    if (lx16a_hndl_data[tmp_hndl].initialized != false) {
        LOG_WRN("No resources available");
    } else if (setup_uart_device(hw, &dev) != 0) {
        LOG_WRN("Device setup failed");
    } else if ((uart_hndl = uart_al_init(&dev)) < 0) {
        LOG_WRN("Uart init failed");
    } else {
        hndl = tmp_hndl;
        lx16a_hndl_data[hndl].initialized = true;
        lx16a_hndl_data[hndl].enabled = false;
        lx16a_hndl_data[hndl].retries = 1;
        lx16a_hndl_data[hndl].uart_hndl = uart_hndl;
        lx16a_hndl_data[hndl].stats.rd_retries = 0;
        lx16a_hndl_data[hndl].stats.rd_timeouts = 0;
        lx16a_hndl_data[hndl].stats.rd_invalids = 0;
        lx16a_hndl_data[hndl].stats.rd_fails = 0;
        lx16a_hndl_data[hndl].stats.wr_retries = 0;
        lx16a_hndl_data[hndl].stats.wr_fails = 0;
    }

    pthread_mutex_unlock(&lx16a_api_lock);

    return hndl;
}

/*******************************************************************************
 * @brief lx16a_exit - close/exit an open instance of the LX-16A driver
 *
 * @details Performs de-initialization of an instance of the LX-16A driver
 *
 * @return None
 ******************************************************************************/

void lx16a_exit(int hndl)
{
    pthread_mutex_lock(&lx16a_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else {
        // Disable all servos on this link
        (void) set_all_enable_states(hndl, false);
        // Shutdown the uart
        uart_al_exit(lx16a_hndl_data[hndl].uart_hndl);
        // Mark as exit'd
        lx16a_hndl_data[hndl].initialized = false;
    }

    pthread_mutex_unlock(&lx16a_api_lock);
}

/*******************************************************************************
 * @brief lx16a_set_all_enable_states - enables or disables all servos on the
 * link related to the file descriptor
 *
 * @details Powers on (enables) or off (disables) all servos on the link
 * related to the file descriptor
 *
 * @return Returns 0 on success, otherwise -1
 ******************************************************************************/

int lx16a_set_all_enable_states(int hndl, bool enable)
{
    int rc = -1;

    pthread_mutex_lock(&lx16a_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else {
        rc = set_all_enable_states(hndl, enable);
    }

    pthread_mutex_unlock(&lx16a_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief lx16a_get_angle - retrieve the current servo angle
 *
 * @details Retrieves the current angle of the servo represented by the "id" on
 * the link related to the file descriptor
 *
 * @return Returns the retrieved angle on success, otherwise a value < -0.1
 ******************************************************************************/

float lx16a_get_angle(int hndl, uint8_t id)
{
    float angle = -1.0f;

    pthread_mutex_lock(&lx16a_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else {
        static const uint8_t POS_RD_PARAM_CNT = 2;
        uint8_t params[POS_RD_PARAM_CNT];
        if (cmd_read(hndl, id, LX16A_CMD_POS_READ, params, POS_RD_PARAM_CNT) != 0) {
            // Already printed error, just return
        } else {
            uint16_t trans_angle = MAKE_UINT16(params[1], params[0]);
            angle = ANGLE_FACTOR * trans_angle;
        }
    }

    pthread_mutex_unlock(&lx16a_api_lock);

    return angle;
}

/*******************************************************************************
 * @brief lx16a_set_angle - set the servo angle
 *
 * @details Sets the angle of the servo represented by the "id" on the link
 * related to the file descriptor.  The movement will be perform in
 * approximately "time_ms" milliseconds.  
 *
 * @return Returns 0 on success, otherwise -1
 ******************************************************************************/

int lx16a_set_angle(int hndl, uint8_t id, float angle, uint16_t time_ms)
{
    int rc = -1;

    pthread_mutex_lock(&lx16a_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else {
        static const uint8_t ANG_SET_PARAM_CNT = 4;
        uint8_t params[ANG_SET_PARAM_CNT];
        float adj_angle = angle;
        uint16_t adj_ms = time_ms;

        // Apply the angle limits
        if (adj_angle < LX16A_ANGLE_RANGE_DEG_MIN) {
            adj_angle = LX16A_ANGLE_RANGE_DEG_MIN;
        } else if (adj_angle > LX16A_ANGLE_RANGE_DEG_MAX) {
            adj_angle = LX16A_ANGLE_RANGE_DEG_MAX;
        }

        // Apply the timing limits
        if (adj_ms < LX16A_ANGLE_TIMING_RANGE_MS_MIN) {
            adj_ms = LX16A_ANGLE_TIMING_RANGE_MS_MIN;
        } else if (adj_ms > LX16A_ANGLE_TIMING_RANGE_MS_MAX) {
            adj_ms = LX16A_ANGLE_TIMING_RANGE_MS_MAX;
        }

        uint16_t trans_angle = adj_angle / ANGLE_FACTOR;

        params[0] = GET_UINT16_LSB(trans_angle);
        params[1] = GET_UINT16_MSB(trans_angle);
        params[2] = GET_UINT16_LSB(adj_ms);
        params[3] = GET_UINT16_MSB(adj_ms);

        rc = cmd_write(hndl, id, LX16A_CMD_MOVE_TIME_WRITE, params, ANG_SET_PARAM_CNT);
    }

    pthread_mutex_unlock(&lx16a_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief lx16a_get_stats - test/debug function to read driver statistics
 *
 * @details Retrieve driver statistics  
 *
 * @return Returns 0 on success and statistics via passed pointers, otherwise -1
 ******************************************************************************/

int lx16a_get_stats(int hndl, lx16a_stats_t *stats)
{
    int rc = -1;

    pthread_mutex_lock(&lx16a_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else if (stats == NULL) {
        LOG_WRN("Invalid params");
    } else {
        stats->lost = uart_al_get_lost_byte_count(lx16a_hndl_data[hndl].uart_hndl);
        stats->rd_retries = lx16a_hndl_data[hndl].stats.rd_retries;
        stats->rd_timeouts = lx16a_hndl_data[hndl].stats.rd_timeouts;
        stats->rd_invalids = lx16a_hndl_data[hndl].stats.rd_invalids;
        stats->rd_fails = lx16a_hndl_data[hndl].stats.rd_fails;
        stats->wr_retries = lx16a_hndl_data[hndl].stats.wr_retries;
        stats->wr_fails = lx16a_hndl_data[hndl].stats.wr_fails;
        rc = 0;
    }

    pthread_mutex_unlock(&lx16a_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief lx16a_cmd_read - test/debug function to perform a read command
 *
 * @details Perform a read command to the servo link for test/debug purposes  
 *
 * @return Returns 0 on success (and read parameters via params), otherwise -1
 ******************************************************************************/

int lx16a_cmd_read(int hndl, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t paramCnt)
{
    int rc = -1;

    pthread_mutex_lock(&lx16a_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else {
        rc = cmd_read(hndl, id, cmd, params, paramCnt);
    }

    pthread_mutex_unlock(&lx16a_api_lock);

    return rc;
}

/*******************************************************************************
 * @brief lx16a_cmd_write - test/debug function to perform a write command
 *
 * @details Perform a write command to the servo link for test/debug purposes  
 *
 * @return Returns 0 on success, otherwise -1
 ******************************************************************************/

int lx16a_cmd_write(int hndl, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t paramCnt)
{
    int rc = -1;

    pthread_mutex_lock(&lx16a_api_lock);

    if (hndl < 0 && hndl >= MAX_NUMBER_OF_HANDLES) {
        LOG_WRN("Invalid hndl");
    } else {
        rc = cmd_write(hndl, id, cmd, params, paramCnt);
    }

    pthread_mutex_unlock(&lx16a_api_lock);

    return rc;
}

/*****************************************************************************/

// Returns 0 on success (and read parameters via params), otherwise -1
static int cmd_read(int hndl, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_cnt) {
    uint8_t retries = lx16a_hndl_data[hndl].retries;
    int rc = -1;

    if (retries == 0) {
        rc = cmd_read_no_retry(hndl, id, cmd, params, param_cnt);
    } else {
        for (int i = 0; i <= retries; i++) { // Using "<=" since we try once and then should have "retries" number of retries
            rc = cmd_read_no_retry(hndl, id, cmd, params, param_cnt);
            if (rc == 0)
                break;
            if (i != 0) { // The first attempt is not a retry
                lx16a_hndl_data[hndl].stats.rd_retries ++;
            }
            usleep(2000); // Delay between tries (delay time is arbitrary)
        }
    }

    if (rc != 0) {
        lx16a_hndl_data[hndl].stats.rd_fails ++;
    }

    return rc;
}

// Returns 0 on success, otherwise -1 
static int cmd_read_no_retry(int hndl, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_cnt)
{
    int rc = -1;

    if ((param_cnt > MAX_CMD_PARAM_CNT) || ((param_cnt > 0) && (params == NULL))) {
        LOG_WRN("Invalid params");
    } else {
        // Issue the write command 
        if ((cmd_write_no_retry(hndl, id, cmd, NULL, 0)) == 0) {
            static const uint32_t MAX_WAIT_US = 100000; // Arbitrary maximum
            static const uint32_t LOOP_DELAY_US = 10000;
            static const uint32_t MAX_LOOPS = MAX_WAIT_US / LOOP_DELAY_US;
            uint8_t buf[PACKET_HEADER_SIZE + MAX_CMD_PARAM_CNT];
            uint8_t pkt_size = PACKET_HEADER_SIZE + param_cnt;
            uint8_t buf_cnt = 0;
            uint32_t loops = 0;

            do {
                int rd_size = uart_al_read(lx16a_hndl_data[hndl].uart_hndl, &buf[buf_cnt], (pkt_size - buf_cnt));
                if (rd_size > 0) {
                    buf_cnt += rd_size;
                    if (buf_cnt >= pkt_size) {
                        // Shouldn't happen, but truncate any overruns
                        if (buf_cnt > pkt_size) {
                            buf_cnt = pkt_size;
                        }
                        break;
                    }
                }

                usleep(LOOP_DELAY_US);
                loops ++;
            } while (loops < MAX_LOOPS);

#if defined(DISPLAY_READ_WRITE_PACKETS)
            for (int i = 0; i < buf_cnt; i++) { LOG_DBG("rd-pkt[%d]:%02x", i, buf[i]); }
#endif

            if (loops >= MAX_LOOPS) {
                lx16a_hndl_data[hndl].stats.rd_timeouts ++;
                LOG_INF("Read timeout (%u/%u)", pkt_size, buf_cnt);
            } else if (!verify_rcvd_pkt(buf, id, cmd, param_cnt)) {
                lx16a_hndl_data[hndl].stats.rd_invalids ++;
                LOG_INF("Invalid rcvd pkt");
            } else {
                for (int i = 0; i < param_cnt; i++) {
                    params[i] = buf[5 + i];
                }
                rc = 0;
            }
        }
    }

    return rc;
}

// Returns 0 on success, otherwise -1 
static int cmd_write(int hndl, uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_cnt) {
    uint8_t retries = lx16a_hndl_data[hndl].retries;
    int rc = -1;

    if (retries == 0) {
        rc = cmd_write_no_retry(hndl, id, cmd, params, param_cnt);
    } else {
        for (int i = 0; i <= retries; i++) { // Using "<=" since we try once and then should have "retries" number of retries
            rc = cmd_write_no_retry(hndl, id, cmd, params, param_cnt);
            if (rc == 0)
                break;
            lx16a_hndl_data[hndl].stats.wr_retries ++;
            usleep(2000); // Delay between tries (delay time is arbitrary)
        }
    }

    if (rc != 0) {
        lx16a_hndl_data[hndl].stats.wr_fails ++;
    }

    return rc;
}

// Returns 0 on success, otherwise -1 
static int cmd_write_no_retry(int hndl, uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_cnt)
{
    int rc = -1;

    if ((param_cnt > MAX_CMD_PARAM_CNT) || ((param_cnt > 0) && (params == NULL))) {
        LOG_INF("Invalid params");
    } else {
        uint8_t buf[PACKET_HEADER_SIZE + MAX_CMD_PARAM_CNT];
        uint8_t pkt_size = PACKET_HEADER_SIZE + param_cnt;
        uint8_t chksum = 0;

        // Flush the uart input so left-over data does not mess up our operation
        (void) uart_al_flush(lx16a_hndl_data[hndl].uart_hndl);

        // Build the data packet
        buf[0] = HEADER_MARKER;
        buf[1] = HEADER_MARKER;
        buf[2] = id;
        buf[3] = 3 + param_cnt; // 3 is length, cmd, and checksum bytes
        buf[4] = cmd;
        for (int i = 0; i < param_cnt; i ++) {
            buf[5 + i] = params[i];
        }
        for (int i = 2; i < pkt_size - 1; i ++) {
            chksum += buf[i];
        }
        buf[pkt_size - 1] = ~chksum;

#if defined(DISPLAY_READ_WRITE_PACKETS)
        for (int x = 0; x < pkt_size; x++) { LOG_DBG("wr-pkt[%d]:%02x", x, buf[x]); }
#endif

        rc = uart_al_write(lx16a_hndl_data[hndl].uart_hndl, buf, pkt_size);
        if (rc != 0) {
            LOG_INF("Uart write failed");
        }
    }

    return rc;
}

// Returns 0 on success, otherwise -1 
static int set_all_enable_states(int hndl, bool enable)
{
    int rc = -1;
	uint8_t params[1];

    params[0] = (enable) ? 1 : 0;
	rc = cmd_write_no_retry(hndl, LX16A_ID_BROADCAST, LX16A_CMD_LOAD_OR_UNLOAD_WRITE, params, 1);

    return rc;
}

static int setup_uart_device(const lx16a_hardware_t *hw, uart_al_device_t *dev)
{
    int rc = -1;

    if (hw == NULL) {
        LOG_INF("Invalid params");
    } else {
#if defined(ZEPHYR_BUILD)
        dev->uart = hw->uart;
        // Baud rate is set in the device tree
        rc = 0;
#elif defined(MAC_BUILD)
        dev->uart = hw->uart;
        dev->baud = hw->baud;
        rc = 0;
#elif defined(WINDOWS_BUILD)
        dev->uart = hw->uart;
        dev->baud = hw->baud;
        rc = 0;
#endif
    }

    return rc;
}

static bool verify_rcvd_pkt(uint8_t *buf, uint8_t id, uint8_t cmd, uint8_t param_cnt)
{
    bool valid = false;

    if (buf[0] != HEADER_MARKER) {
        LOG_INF("Invalid buf[0] (%02x)", buf[0]);
    } else if (buf[1] != HEADER_MARKER) {
        LOG_INF("Invalid buf[1] (%02x)", buf[1]);
    } else if (buf[2] != id) {
        LOG_INF("Invalid buf[2] (%02x)", buf[2]);
    } else if (buf[4] != cmd) {
        LOG_INF("Invalid buf[4] (%02x)", buf[4]);
    } else if (buf[3] != (3 + param_cnt)) { // 3 is length, cmd, and checksum bytes
        LOG_INF("Invalid buf[3] (%02x)", buf[3]);
    } else {
        uint8_t pkt_size = PACKET_HEADER_SIZE + param_cnt;
        uint8_t chksum = 0;

        for (int i = 2; i < pkt_size - 1; i ++) {
            chksum += buf[i];
        }
        chksum = ~chksum;

        if (chksum != buf[pkt_size - 1]) {
            LOG_INF("Invalid chksum (%02x/%02x)", chksum, buf[pkt_size - 1]);
        } else {
            valid = true;
        }
    }

    return valid;
}
