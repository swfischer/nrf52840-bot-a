/*****************************************************************************
 * os_al.c - The source to some OS abstracted functions
 * 
 * The point of this interface is to provide some OS agnostic functions to
 * make coding easier.
 *****************************************************************************/

#include "os_al.h"

#if defined(ZEPHYR_BUILD) || defined(MAC_BUILD)
#include <time.h>
#endif

uint32_t os_al_get_ms32(void)
{
    uint32_t ms = 0;

#if defined(ZEPHYR_BUILD) || defined(MAC_BUILD)
    struct timespec now = {0, 0};
    if (clock_gettime(CLOCK_MONOTONIC, &now) == 0) {
        ms = ((uint32_t)(now.tv_sec * 1000U)) + ((uint32_t)(now.tv_nsec / 1000000));
    }
#endif

    return ms;
}
