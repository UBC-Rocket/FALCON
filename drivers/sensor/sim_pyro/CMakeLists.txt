zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SIM_PYRO sim_pyro.c)