#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "data.h"

LOG_MODULE_REGISTER(data, LOG_LEVEL_INF);

// Global instances
struct imu_data g_imu_data;
struct baro_data g_baro_data;

// Mutexes for thread safety
K_MUTEX_DEFINE(imu_mutex);
K_MUTEX_DEFINE(baro_mutex);

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
