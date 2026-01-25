# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)

set(ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../../..)

# Compatible boards for this app
set(COMPATIBLE_BOARDS
    ubcrocket_fc_2526_r1
    native_sim/native/64
    native_sim
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

target_sources(app PRIVATE
  src/main.c
  src/data.c
  src/sensors/imu_thread.c
  src/logger_thread.c
  src/sensors/baro_thread.c
  src/state_machine/state_machine.c
  src/state_machine/state_machine_common.c
  src/state_machine/states/standby.c
  src/state_machine/states/ascent.c
  src/state_machine/states/mach_lock.c
  src/state_machine/states/drogue_descent.c
  src/state_machine/states/main_descent.c
  src/state_machine/states/landed.c
  src/pyro/pyro_thread.c
)

target_include_directories(app PRIVATE
  src
  src/state_machine
)

if(DEFINED DATA_FILE)
  zephyr_compile_definitions(DATA_FILE="${DATA_FILE}")
endif()