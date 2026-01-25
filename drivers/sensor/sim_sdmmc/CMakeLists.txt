zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SIM_SDMMC sim_sdmmc.c)