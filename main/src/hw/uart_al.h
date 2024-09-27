/*****************************************************************************
 * uart_al.h - An abstraction layer (al) interface to a UART.
 * 
 * The point of this abstraction layer is to modularize the code wrapping the
 * OS UART driver to project an easy to use interface and allow multiple
 * implementations based on the OS the code is running under.
 *****************************************************************************/

#ifndef UART_AL_H
#define UART_AL_H

#include <stddef.h>
#include <stdint.h>
#if defined(ZEPHYR_BUILD)
#include <zephyr/device.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

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
} uart_al_device_t;

// Returns an instance handle on success, otherwise -1
extern int uart_al_init(const uart_al_device_t *device);
extern void uart_al_exit(int hndl);
// Returns the number of bytes read on success, otherwise -1
extern int uart_al_read(int hndl, uint8_t *buffer, uint16_t size);
// Returns 0 on success, otherwise -1
extern int uart_al_write(int hndl, const uint8_t *buffer, uint16_t size);
// Returns 0 on success, otherwise -1
extern int uart_al_flush(int hndl);
// Returns the number of bytes that have been lost due to buffer overflows
extern uint32_t uart_al_get_lost_byte_count(int hndl);

#ifdef __cplusplus
}
#endif

#endif  // UART_AL_H
