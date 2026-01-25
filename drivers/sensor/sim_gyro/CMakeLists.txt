zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SIM_GYRO sim_gyro.c)