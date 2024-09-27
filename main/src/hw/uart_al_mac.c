/*****************************************************************************
 * uart_al_mac.c - The abstraction layer (al) source to a UART.
 * 
 * The point of this abstraction layer is to modularize the code wrapping the
 * OS UART driver to project an easy to use interface and allow multiple
 * implementations based on the OS the code is running under.
 * 
 * It is assumed that the user of this interface will not call public
 * functions while another public function is running
 *****************************************************************************/

#include "uart_al.h"

#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include "logger.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

typedef struct
{
    bool initialized;
    int uart_hndl;

} uart_al_data;

#define MAX_NUMBER_OF_HANDLES (1)
static uart_al_data uart_al_hndl_data[MAX_NUMBER_OF_HANDLES] = {0};

/*******************************************************************************
 * @brief uart_al_init - initialize the UART AL for use.
 *
 * @details A routine used to initialize the given UART at the given baud rate
 * and prepare the UART for use.
 *
 * @param uart - a device denoting the UART to initialize.
 *
 * @return returns an instance handle on success, otherwise -1.
 ******************************************************************************/

int uart_al_init(const uart_al_device_t *device)
{
    int hndl = -1;
    int tmp_hndl = 0; // Default to the first and only handle

    if (device == NULL) {
        LOG_WRN("Invalid params");
    } else if (uart_al_hndl_data[tmp_hndl].initialized != false) {
        LOG_INF("Already initialized");
    } else {
        do { // Using do/while(0) as a quick escape mechanism

            struct termios options;
            int uart_hndl = open(device->uart, O_RDWR | O_NOCTTY);
            if (uart_hndl == -1) {
                LOG_WRN("Open failed");
                break;
            }
            // Flush away any bytes previously read or written.
            if (tcflush(uart_hndl, TCIOFLUSH) != 0) {
                LOG_WRN("Flush failed");
                close(uart_hndl);
                break;
            }
            // Get the current configuration of the serial port.
            if (tcgetattr(uart_hndl, &options) != 0) {
                LOG_WRN("Get config failed");
                close(uart_hndl);
                break;
            } 
        
            // Turn off any options that might interfere with our ability to send and
            // receive raw binary bytes.
            options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
            options.c_oflag &= ~(ONLCR | OCRNL);
            options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

            // Set up timeouts: Calls to read() will return as soon as there is
            // at least one byte available or when 100 ms has passed.
            options.c_cc[VTIME] = 1;
            options.c_cc[VMIN] = 0;

            // This code only supports certain standard baud rates. Supporting
            // non-standard baud rates should be possible but takes more work.
            switch (device->baud) {
                case 4800:
                    cfsetospeed(&options, B4800);
                    break;
                case 9600:
                    cfsetospeed(&options, B9600);
                    break;
                case 19200:
                    cfsetospeed(&options, B19200);
                    break;
                case 38400:
                    cfsetospeed(&options, B38400);
                    break;
                case 115200:
                default:
                    cfsetospeed(&options, B115200);
                    break;
            }
            cfsetispeed(&options, cfgetospeed(&options));

            if (tcsetattr(uart_hndl, TCSANOW, &options) != 0) {
                LOG_WRN("Set config failed");
                close(uart_hndl);
            } else {
                uart_al_hndl_data[tmp_hndl].initialized = true;
                uart_al_hndl_data[tmp_hndl].uart_hndl = uart_hndl;
                hndl = tmp_hndl;
            }
        } while (0);
    }

    return hndl;
}

/*******************************************************************************
 * @brief uart_al_exit - de-initialize the UART
 *
 * @details A routine used to revert the initialization of the UART.
 *
 * @param hndl - the handle to close.
 *
 * @return none.
 ******************************************************************************/

void uart_al_exit(int hndl)
{
    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (uart_al_hndl_data[hndl].initialized == true) {
        close(uart_al_hndl_data[hndl].uart_hndl);
        uart_al_hndl_data[hndl].initialized = false;
    }
}

/*******************************************************************************
 * @brief uart_al_read - read data from the UART
 *
 * @details A routine used to read a number of bytes from the UART and return
 * the bytes via the passed in buffer pointer.
 *
 * @param hndl - the handle of the UART to read from.
 * @param buffer - a pointer to the buffer to return the read data in.
 * @param size - the number of bytes to read.
 *
 * @return return the number of bytes read, otherwise -1.
 ******************************************************************************/

int uart_al_read(int hndl, uint8_t *buffer, uint16_t size)
{
    int recvd = -1;

    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (uart_al_hndl_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
    } else if ((buffer == NULL) || (size == 0U)) {
        // Just return an error
    } else {
        recvd = 0;
        while (recvd < size) {
            ssize_t ret = read(uart_al_hndl_data[hndl].uart_hndl, &buffer[recvd], size - recvd);
            if (ret < 0) {
                LOG_WRN("Uart read failed");
                recvd = -1;
                break;
            }
            if (ret == 0) {
                // Timeout
                break;
            }
            recvd += ret;
        }
    }

    return recvd;
}

/*******************************************************************************
 * @brief uart_al_write - write data to the UART
 *
 * @details A routine used to write a number of bytes from the passed buffer to
 * the UART.
 *
 * @param hndl - the handle of the UART to write to.
 * @param buffer - a pointer to the buffer containing the data to write.
 * @param size - the number of bytes to write.
 *
 * @return returns 0 on success, otherwise -1
 ******************************************************************************/

int uart_al_write(int hndl, const uint8_t *buffer, uint16_t size)
{
    int rc = -1;

    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (uart_al_hndl_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
    } else if ((buffer == NULL) || (size == 0U)) {
        // Just return an error
    } else {
        ssize_t result = write(uart_al_hndl_data[hndl].uart_hndl, buffer, size);
        if (result != (ssize_t)size) {
            LOG_WRN("Uart write failed");
            rc = -1;
        } else {
            rc = 0;
        }
    }

    return rc;
}

/*******************************************************************************
 * @brief uart_al_flush - flush the input data stream
 *
 * @details A routine to flush any existing data from the input stream.
 *
 * @param hndl - the handle of the UART to flush.
 * 
 * @return Returns 0 on success, otherwise -1
 ******************************************************************************/

int uart_al_flush(int hndl)
{
    int rc = -1;

    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (uart_al_hndl_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
    } else if (tcflush(uart_al_hndl_data[hndl].uart_hndl, TCIOFLUSH) != 0) {
        LOG_WRN("Flush failed");
    } else {
        rc = 0;
    }

    return rc;
}

/*******************************************************************************
 * @brief uart_al_get_lost_byte_count - retrieve the count of bytes lost due to
 * overflow events
 *
 * @details A routine used to retrieve the count of bytes lost due to overflow
 * events.
 *
 * @return returns the number of bytes that have been overwritten (lost) due to
 * buffer overflows.
 ******************************************************************************/

uint32_t uart_al_get_lost_byte_count(int hndl)
{
    return 0;
}

/*******************************************************************************/
