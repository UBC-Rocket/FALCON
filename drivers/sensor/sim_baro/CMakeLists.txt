zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SIM_BARO sim_baro.c)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../sim_common)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/../sim_common/sim_csv.c)