/*****************************************************************************
 * uart_al_windows.c - The abstraction layer (al) source to a UART.
 * 
 * The point of this abstraction layer is to modularize the code wrapping the
 * OS UART driver to project an easy to use interface and allow multiple
 * implementations based on the OS the code is running under.
 * 
 * It is assumed that the user of this interface will not call public
 * functions while another public function is running
 *****************************************************************************/

#include "uart_al.h"

#include <stdint.h>
#include <stdio.h>
#include <windows.h>

#include "logger.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

typedef struct
{
    bool initialized;
    HANDLE port;

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

int uart_al_init(const struct device *uart_dev)
{
    int hndl = -1;
    int tmp_hndl = 0; // Default to the first and only handle

    if (uart_al_hndl_data[tmpfd].initialized != false) {
        LOG_INF("Already initialized");
    } else { 
        do { // Using do/while(0) as a quick escape mechanism
            HANDLE port = CreateFileA(device, (GENERIC_READ | GENERIC_WRITE), 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
            if (port == INVALID_HANDLE_VALUE)
            {
                LOG_WRN("Invalid port handle");
                break;
            }

            // Flush away any bytes previously read or written.
            BOOL success = FlushFileBuffers(port);
            if (!success)
            {
                LOG_WRN("Port flush failed");
                CloseHandle(port);
                break;
            }

            // Configure read and write operations to time out after 100 ms.
            COMMTIMEOUTS timeouts = {0};
            timeouts.ReadIntervalTimeout = 0;
            timeouts.ReadTotalTimeoutConstant = 100;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 100;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            
            success = SetCommTimeouts(port, &timeouts);
            if (!success)
            {
                LOG_WRN("Set timeouts failed");
                CloseHandle(port);
                break;
            }
            
            // Set the baud rate and other options.
            DCB state = {0};
            state.DCBlength = sizeof(DCB);
            state.BaudRate = baud_rate;
            state.ByteSize = 8;
            state.Parity = NOPARITY;
            state.StopBits = ONESTOPBIT;
            success = SetCommState(port, &state);
            if (!success)
            {
                LOG_WRN("Set options failed");
                CloseHandle(port);
                break;
            }

            uart_al_hndl_data[tmp_hndl].initialized = true;
            uart_al_hndl_data[tmp_hndl].port = port;
            hndl = tmp_hndl;

        } while (0)
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
        CloseHandle(uart_al_hndl_data[hndl].port);
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
        DWORD received_cnt;
        BOOL success = ReadFile(uart_al_hndl_data[hndl].port, buffer, size, &received_cnt, NULL);
        if (success)
        {
            recvd = received_cnt;
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
        DWORD written_cnt;
        BOOL success = WriteFile(uart_al_hndl_data[hndl].port, buffer, size, &written_cnt, NULL);
        if (success && (written_cnt == size))
        {
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
    } else {
        BOOL success = FlushFileBuffers(uart_al_hndl_data[hndl].port);
        if (success) {
            rc = 0;
        }
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
