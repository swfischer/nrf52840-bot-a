/*****************************************************************************
 * uart_al_zephyr.c - The abstraction layer (al) source to a UART.
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
#include <unistd.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "logger.h"

LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

#define UART_AL_BUFFER_SIZE (4U * 1024U)

typedef struct
{
    int hndl; // Need a fixed location for the uart_irq_callback_user_data_set call
    bool initialized;
    uint8_t buf[UART_AL_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint32_t lost;

    const struct device *uart_dev;

} uart_al_data;

#define MAX_NUMBER_OF_HANDLES (1)
static uart_al_data uart_al_hndl_data[MAX_NUMBER_OF_HANDLES] = {0};

static void serial_isr(const struct device *dev, void *user_data);

/*******************************************************************************
 * @brief uart_al_init - initialize the UART AL for use.
 *
 * @details A routine used to initialize the given UART at the given baud rate
 * and prepare the UART for use.
 *
 * @param device - a device denoting the UART to initialize.
 *
 * @return returns an instance handle on success, otherwise -1.
 ******************************************************************************/

int uart_al_init(const uart_al_device_t *device)
{
    // The baud rate is set in the device tree

    int hndl = -1;
    int tmp_hndl = 0; // Default to the first and only handle

    if (device == NULL) {
        LOG_WRN("Invalid params");
    } else if (uart_al_hndl_data[tmp_hndl].initialized != false) {
        LOG_INF("Already initialized");
    } else { 
        uart_al_data *hndl_data = &uart_al_hndl_data[tmp_hndl];

        hndl_data->hndl = tmp_hndl;
        hndl_data->head = 0;
        hndl_data->tail = 0;
        hndl_data->lost = 0;
        hndl_data->uart_dev = device->uart;

        if (!device_is_ready(hndl_data->uart_dev)) {
            LOG_WRN("UART device not found");
        } else if (uart_irq_callback_user_data_set(hndl_data->uart_dev, serial_isr, &hndl_data->hndl) != 0) {
            LOG_WRN("Callback set failed");
        } else {
            hndl_data->initialized = true;
            uart_irq_rx_enable(hndl_data->uart_dev);
            hndl = tmp_hndl;
        }
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
        uart_irq_rx_disable(uart_al_hndl_data[hndl].uart_dev);
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
        uart_al_data *hndl_data = &uart_al_hndl_data[hndl];

        // Disable interrupts so that the head and tail indexes don't change
        // while retrieving data from the buffer
        uart_irq_rx_disable(hndl_data->uart_dev);

        recvd = 0;
        while (recvd < size) {
            // Check if there is data available
            if (hndl_data->tail == hndl_data->head) {
                break;
            } else {
                buffer[recvd] = hndl_data->buf[hndl_data->tail];
                hndl_data->tail = (hndl_data->tail + 1U) % UART_AL_BUFFER_SIZE;
                recvd++;
            }
        }

        uart_irq_rx_enable(hndl_data->uart_dev);
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
        uart_al_data *hndl_data = &uart_al_hndl_data[hndl];

        for (size_t i = 0; i < size; i++) {
            uart_poll_out(hndl_data->uart_dev, buffer[i]);
        }
        rc = 0;
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
        // Disable interrupts so that the head and tail indexes don't change
        // while flushing
        uart_irq_rx_disable(uart_al_hndl_data[hndl].uart_dev);

        uart_al_hndl_data[hndl].tail = 0;
        uart_al_hndl_data[hndl].head = 0;
        rc = 0;

        uart_irq_rx_enable(uart_al_hndl_data[hndl].uart_dev);
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
    uint32_t cnt = 0;

    if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
        LOG_WRN("Invalid hndl");
    } else if (uart_al_hndl_data[hndl].initialized == false) {
        LOG_WRN("Un-init'd hndl");
    } else {
        cnt = uart_al_hndl_data[hndl].lost;
    }

    return cnt;
}

/*******************************************************************************/

static void serial_isr(const struct device *dev, void *user_data)
{
    // IMPORTANT: This function is an ISR should must be fast

    (void)dev; // TODO: Investigate why we are not using the passed in device

    if (user_data == NULL) {
        // Do nothing since we don't know which file descriptor
    } else {
        int hndl = *((int*)user_data);
        uint8_t c;

        if ((hndl < 0) || (hndl >= MAX_NUMBER_OF_HANDLES)) {
            // Invalid hndl
        } else if (uart_al_hndl_data[hndl].initialized == false) {
            // Un-init'd hndl
        } else if (!uart_irq_update(uart_al_hndl_data[hndl].uart_dev)) {
            // This should not occur
        } else {
            uart_al_data *hndl_data = &uart_al_hndl_data[hndl];

            while (uart_irq_rx_ready(hndl_data->uart_dev) != 0) {
                uart_fifo_read(hndl_data->uart_dev, &c, 1);

                // We will overwrite old data for new incoming data
                hndl_data->buf[hndl_data->head] = c;
                hndl_data->head = (hndl_data->head + 1U) % UART_AL_BUFFER_SIZE;
                // If the tail now equals the head, we have an overflow condition and we need to move the tail
                if (hndl_data->tail == hndl_data->head) {
                    hndl_data->tail = (hndl_data->tail + 1U) % UART_AL_BUFFER_SIZE;
                    hndl_data->lost++;  // Count the overflow
                }
            }
        }
    }
}
