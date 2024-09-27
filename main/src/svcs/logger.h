/*****************************************************************************
 * logger.h - The interface to logging infrastructure
 * 
 * The point of this interface is to support Zephyr style logging macros in
 * Zephyr builds as well as MacOS and Windows builds.
 *****************************************************************************/

#include <stdint.h>

#ifndef LOGGER_H
#define LOGGER_H

// Add to one file: LOG_MODULE_REGISTER(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);
// Add to all other files: LOG_MODULE_DECLARE(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

#if defined(ZEPHYR_BUILD)
#include <zephyr/logging/log.h>
#define APP_MODULE_LOGGER_NAME bota
#define APP_MODULE_LOGGER_LEVEL LOG_LEVEL_DBG
#else
#include <stdarg.h>

#define LOG_LEVEL_NONE 0U
#define LOG_LEVEL_ERR  1U
#define LOG_LEVEL_WRN  2U
#define LOG_LEVEL_INF  3U
#define LOG_LEVEL_DBG  4U

#define LOG_MODULE_REGISTER(_name, _level) LOG_MODULE_DECLARE(_name,_level)
#define LOG_MODULE_DECLARE(_name, _level) \
static const char logger_module_name[] = _name; \
static const uint8_t logger_module_level = _level 

#define LOG_ERR(...)    Z_LOG(LOG_LEVEL_ERR, __VA_ARGS__)
#define LOG_WRN(...)    Z_LOG(LOG_LEVEL_WRN, __VA_ARGS__)
#define LOG_INF(...)    Z_LOG(LOG_LEVEL_INF, __VA_ARGS__)
#define LOG_DBG(...)    Z_LOG(LOG_LEVEL_DBG, __VA_ARGS__)

#define Z_LOG(_level, ...) \
	logger_nz_print(_level, __FILE__, logger_module_name, logger_module_level, __VA_ARGS__)

#define APP_MODULE_LOGGER_NAME "bot"
#define APP_MODULE_LOGGER_LEVEL LOG_LEVEL_DBG

extern void logger_nz_print(uint8_t print_level, const char *file, const char *module, int min_level, ...);

#endif

#endif // LOGGER_H
