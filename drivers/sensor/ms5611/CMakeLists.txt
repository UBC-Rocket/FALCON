# Copyright (c) 2019 Thomas Schmid <tom@lfence.de>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(ms5611.c ms5611_i2c.c ms5611_spi.c)
