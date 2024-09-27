# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

if (NOT build)
    set(build zephyr)
endif()

if (build STREQUAL zephyr)
    # Re-direct the directory where the 'boards' directory is found from $ZEPHYR_BASE to this directory.
    list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

    if (NOT BOARD)
        # Set default board
        set(BOARD nrf52840dk_x/nrf52840)
    endif()

    find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
endif()

project(lidar-test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror -Wall -std=c99")

list(APPEND COMMON_INCLUDES src)
list(APPEND COMMON_INCLUDES src/app)
list(APPEND COMMON_INCLUDES src/hw)
list(APPEND COMMON_INCLUDES src/svcs)

list(APPEND COMMON_SOURCES src/main.c)
list(APPEND COMMON_SOURCES src/app/drive_ctrl.c)
list(APPEND COMMON_SOURCES src/app/lidar.c)
list(APPEND COMMON_SOURCES src/app/mapper.c)
list(APPEND COMMON_SOURCES src/app/mapper_shell.c)
list(APPEND COMMON_SOURCES src/app/panner.c)
list(APPEND COMMON_SOURCES src/app/panner_shell.c)
list(APPEND COMMON_SOURCES src/app/roamer_app.c)
list(APPEND COMMON_SOURCES src/app/tracks.c)
list(APPEND COMMON_SOURCES src/app/tracks_shell.c)
list(APPEND COMMON_SOURCES src/hw/dcme.c)
list(APPEND COMMON_SOURCES src/hw/drivetrain.c)
list(APPEND COMMON_SOURCES src/hw/drivetrain_shell.c)
list(APPEND COMMON_SOURCES src/hw/i2c_al_zephyr.c)
list(APPEND COMMON_SOURCES src/hw/imu_al_zephyr_ism330dhcx.c)
list(APPEND COMMON_SOURCES src/hw/imu_al_shell.c)
list(APPEND COMMON_SOURCES src/hw/lx16a.c)
list(APPEND COMMON_SOURCES src/hw/lx16a_shell.c)
list(APPEND COMMON_SOURCES src/hw/pid.c)
list(APPEND COMMON_SOURCES src/hw/rgb_led_neopixel.c)
list(APPEND COMMON_SOURCES src/hw/rgb_led_shell.c)
list(APPEND COMMON_SOURCES src/hw/sign_of_life.c)
list(APPEND COMMON_SOURCES src/hw/start_stop.c)
list(APPEND COMMON_SOURCES src/hw/tfluna.c)
list(APPEND COMMON_SOURCES src/hw/tfluna_shell.c)
list(APPEND COMMON_SOURCES src/hw/uart_al_zephyr.c)
list(APPEND COMMON_SOURCES src/svcs/os_al.c)
list(APPEND COMMON_SOURCES src/svcs/os_al_timer.c)
list(APPEND COMMON_SOURCES src/svcs/shutdown_shell.c)

if(build STREQUAL zephyr)
    add_compile_definitions(ZEPHYR_BUILD)
    target_include_directories(app PRIVATE ${COMMON_INCLUDES})
    target_sources(app PRIVATE ${COMMON_SOURCES})
endif()
