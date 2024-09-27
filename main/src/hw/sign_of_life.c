/*****************************************************************************
 * sign_of_life.c - source file for the sign-of-life feature
 *****************************************************************************/

#include <pthread.h>
#include <stdint.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define CYCLE_TIME_MS   (1500)
#define ON_TIME_MS      (200)
#define OFF_TIME_MS     (CYCLE_TIME_MS - ON_TIME_MS)

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

//! This mutex is used with the LED toggle condition
static pthread_mutex_t sol_toggle_lock = PTHREAD_MUTEX_INITIALIZER;
//! This condition is used for timing toggle events
static pthread_cond_t sol_toggle_cond = PTHREAD_COND_INITIALIZER;
//! The LED to toggle
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
//! Flag to exit the LED toggle loop
static bool sol_exit = false;

static void get_delay_ts(struct timespec *ts, uint16_t delay_ms);

void sign_of_life_start(void)
{
    struct timespec delay_ts = {0};
    int delay_ms = OFF_TIME_MS;

    pthread_mutex_lock(&sol_toggle_lock);

	if (!gpio_is_ready_dt(&led)) {
        // Just return
	} else if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) {
        // Just return
	} else {
        sol_exit = false;

        do
        {
            (void) gpio_pin_set_dt(&led, (delay_ms == ON_TIME_MS));

            get_delay_ts(&delay_ts, delay_ms);
            pthread_cond_timedwait(&sol_toggle_cond, &sol_toggle_lock, &delay_ts);

            delay_ms = (delay_ms == OFF_TIME_MS) ? ON_TIME_MS : OFF_TIME_MS;

        } while (!sol_exit);
    }

    pthread_mutex_unlock(&sol_toggle_lock);
}

void sign_of_life_end(void)
{
    // Set the exit flag
    sol_exit = true;
    // Signal to wake up the current delay
    pthread_mutex_lock(&sol_toggle_lock);
    pthread_cond_signal(&sol_toggle_cond);
    pthread_mutex_unlock(&sol_toggle_lock);
}

static void get_delay_ts(struct timespec *ts, uint16_t delay_ms)
{
    // Get the current time
    clock_gettime(CLOCK_MONOTONIC, ts);

    // Determine the adjustments
    uint32_t delay = delay_ms;  // Used to keep type size the same in the next few lines
    uint32_t sec = (delay / 1000U);
    uint32_t ms = delay - (sec * 1000U);

    // Deal with nsec overflow
    long NSEC_IN_SEC = 1000 * 1000 * 1000;
    long nsec = ts->tv_nsec + (((long)ms) * 1000 * 1000);
    if (nsec > NSEC_IN_SEC) {
        nsec -= NSEC_IN_SEC;
        sec++;
    }

    // Add delay to ts
    ts->tv_sec += sec;
    ts->tv_nsec = nsec;
}
