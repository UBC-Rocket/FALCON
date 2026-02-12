#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "data.h"

LOG_MODULE_REGISTER(data, LOG_LEVEL_INF);

// Global instances
struct imu_data g_imu_data;
struct baro_data g_baro_data;
struct state_data g_state_data;
struct gps_data g_gps_data;

// Mutexes for thread safety
K_MUTEX_DEFINE(imu_mutex);
K_MUTEX_DEFINE(baro_mutex);
K_MUTEX_DEFINE(state_mutex);
K_MUTEX_DEFINE(gps_mutex);

// Setter functions
void set_imu_data(const struct imu_data *src)
{
    k_mutex_lock(&imu_mutex, K_FOREVER);
    g_imu_data = *src;
    k_mutex_unlock(&imu_mutex);
}

void set_baro_data(const struct baro_data *src)
{
    k_mutex_lock(&baro_mutex, K_FOREVER);
    g_baro_data = *src;
    k_mutex_unlock(&baro_mutex);
}

// Getter functions
void get_imu_data(struct imu_data *dst)
{
    k_mutex_lock(&imu_mutex, K_FOREVER);
    *dst = g_imu_data;
    k_mutex_unlock(&imu_mutex);
}

void get_baro_data(struct baro_data *dst)
{
    k_mutex_lock(&baro_mutex, K_FOREVER);
    *dst = g_baro_data;
    k_mutex_unlock(&baro_mutex);
}

void set_state_data(const struct state_data *src)
{
    k_mutex_lock(&state_mutex, K_FOREVER);
    g_state_data = *src;
    k_mutex_unlock(&state_mutex);
}

void get_state_data(struct state_data *dst)
{
    k_mutex_lock(&state_mutex, K_FOREVER);
    *dst = g_state_data;
    k_mutex_unlock(&state_mutex);
}

void set_gps_data(const struct gps_data *src)
{
    k_mutex_lock(&gps_mutex, K_FOREVER);
    g_gps_data = *src;
    k_mutex_unlock(&gps_mutex);
}

void get_gps_data(struct gps_data *dst)
{
    k_mutex_lock(&gps_mutex, K_FOREVER);
    *dst = g_gps_data;
    k_mutex_unlock(&gps_mutex);
}
