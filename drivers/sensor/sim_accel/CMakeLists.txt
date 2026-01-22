zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SIM_ACCEL sim_accel.c)