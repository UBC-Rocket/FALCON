# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cloudburst_integration_test)

# Include nanopb for protobuf support
list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)

# Generate protobuf sources from falcon-protos
set(FALCON_PROTOS_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../../modules/falcon-protos)
zephyr_nanopb_sources(app ${FALCON_PROTOS_DIR}/HelloWorldPacket.proto)
zephyr_nanopb_sources(app ${FALCON_PROTOS_DIR}/TelemetryPacket.proto)

# LwGPS NMEA parser library
set(LWGPS_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../../modules/lwgps)

# Add the main application sources needed for integration testing
target_sources(app PRIVATE 
    src/main.c
    ../../src/data.c
    ../../src/state_machine/state_machine.c
    ../../src/state_machine/state_machine_common.c
    ../../src/state_machine/states/standby.c
    ../../src/state_machine/states/ascent.c
    ../../src/state_machine/states/mach_lock.c
    ../../src/state_machine/states/drogue_descent.c
    ../../src/state_machine/states/main_descent.c
    ../../src/state_machine/states/landed.c
    ../../src/pyro/pyro_thread.c
    ../../src/gps/gps_thread.c
    ../../src/radio/radio_thread.c
    ${LWGPS_DIR}/lwgps/src/lwgps/lwgps.c

)

target_include_directories(app PRIVATE
    ../../src
    ../../src/state_machine
)
