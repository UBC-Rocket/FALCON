# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(state_machine_test)

target_sources(app PRIVATE
  ../../src/state_machine/state_machine.c
  ../../src/state_machine/state_machine_common.c
  ../../src/state_machine/states/standby.c
  ../../src/state_machine/states/ascent.c
  ../../src/state_machine/states/mach_lock.c
  ../../src/state_machine/states/drogue_descent.c
  ../../src/state_machine/states/main_descent.c
  ../../src/state_machine/states/landed.c
  ../../src/data.c
  src/main.c
)

target_include_directories(app PRIVATE
  ../../src
  ../../src/state_machine
)
