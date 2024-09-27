/*****************************************************************************
 * test_app.h - header file for the tank track test application
 * 
 * This application will:
 * a) Wait for a start command (via start_stop.h)
 * b) Move forward 50cm
 * c) Turn CW 90 degrees
 * d) Move forward 10cm
 * e) Turn CCW 90 degrees
 * f) Move reverse 50cm
 * g) Stop and wait for the next start
 *
 *****************************************************************************/

#ifndef TEST_APP_H
#define TEST_APP_H

// Return 0 on success, otherwise -1
extern int test_app_start(void);
extern void test_app_exit(void);

#endif // TEST_APP_H