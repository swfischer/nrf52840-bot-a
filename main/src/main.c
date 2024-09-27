/*****************************************************************************
 * main.c - the main function for this application
 *****************************************************************************/

#include <unistd.h>

#include "logger.h"
#include "roamer_app.h"
#include "sign_of_life.h"

LOG_MODULE_REGISTER(APP_MODULE_LOGGER_NAME, APP_MODULE_LOGGER_LEVEL);

int main(void)
{
	LOG_INF("Starting Bot-A application");

	if (roamer_app_start() != 0) {
		LOG_ERR("App start failed");
	} else {
		LOG_DBG("App start success");
	}

	// Continue to use this task as SOL indicator
	// Note, this call does not return unless sign_of_life_end() is called
	sign_of_life_start();

	roamer_app_exit();

	// Spin in a loop forever, just to be sure what the system is doing
	while (1) {
		sleep(5);
	}

	return 0;
}
