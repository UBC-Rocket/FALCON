# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)

set(ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../../..)

# Compatible boards for this app
set(COMPATIBLE_BOARDS
    ubcrocket_fc_2526_r1
)

if(NOT BOARD IN_LIST COMPATIBLE_BOARDS)
    message(WARNING 
        "Board '${BOARD}' is not listed as compatible with this application.\n"
        "Compatible boards: ${COMPATIBLE_BOARDS}\n"
        "Build may fail or behave unexpectedly.\n"
        "To add compatibility for this board, add it to the COMPATIBLE_BOARDS list in CMakeLists.txt.")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cloudburst)

target_sources(app PRIVATE src/main.c src/data.c src/sensors/imu_thread.c src/logger_thread.c src/sensors/baro_thread.c src/state_machine.c)
