# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)

# Any board with an LED named led0 should be supported (this includes most dev boards and our custom boards)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blinky_test)

target_sources(app PRIVATE src/main.c)
