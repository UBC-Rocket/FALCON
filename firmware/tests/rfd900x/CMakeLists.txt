# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rfd900x_test)

# nanopb-generated GroundCommand.pb.h (RfdConfig), same as the firmware build
list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)
set(FALCON_PROTOS_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../../modules/falcon-protos)
zephyr_nanopb_sources(app ${FALCON_PROTOS_DIR}/GroundCommand.proto)

target_sources(app PRIVATE
  ../../src/radio/rfd900x_at.c
  src/main.c
)

target_include_directories(app PRIVATE
  ../../src
  ../../src/radio
)

# Shrink the AT-session timing so the timeout-path tests run fast; the
# uplink_config.h defaults stay in force for the real firmware build.
target_compile_definitions(app PRIVATE
  RFD_AT_GUARD_TIME_MS=20
  RFD_AT_ENTER_TIMEOUT_MS=80
  RFD_AT_RESPONSE_TIMEOUT_MS=60
)
