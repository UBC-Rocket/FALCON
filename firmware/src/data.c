#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "data.h"

LOG_MODULE_REGISTER(data, LOG_LEVEL_INF);

// Global instances
struct imu_data g_imu_data;
struct baro_data g_baro_data;
struct state_data g_state_data;
struct pyro_data g_pyro_data;

// Mutexes for thread safety
K_MUTEX_DEFINE(imu_mutex);
K_MUTEX_DEFINE(baro_mutex);
K_MUTEX_DEFINE(state_mutex);
K_MUTEX_DEFINE(pyro_data_mutex);

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

void set_pyro_data(const struct pyro_data *src)
{
    k_mutex_lock(&pyro_data_mutex, K_FOREVER);

    // Log changes to pyro flags
    if (src->drogue_fire_requested != g_pyro_data.drogue_fire_requested) {
        LOG_INF("Pyro: drogue_fire_requested -> %d", src->drogue_fire_requested);
    }
    if (src->main_fire_requested != g_pyro_data.main_fire_requested) {
        LOG_INF("Pyro: main_fire_requested -> %d", src->main_fire_requested);
    }
    if (src->drogue_fire_ack != g_pyro_data.drogue_fire_ack) {
        LOG_INF("Pyro: drogue_fire_ack -> %d", src->drogue_fire_ack);
    }
    if (src->main_fire_ack != g_pyro_data.main_fire_ack) {
        LOG_INF("Pyro: main_fire_ack -> %d", src->main_fire_ack);
    }
    if (src->drogue_fired != g_pyro_data.drogue_fired) {
        LOG_INF("Pyro: drogue_fired -> %d", src->drogue_fired);
    }
    if (src->main_fired != g_pyro_data.main_fired) {
        LOG_INF("Pyro: main_fired -> %d", src->main_fired);
    }
    if (src->drogue_fail != g_pyro_data.drogue_fail) {
        LOG_ERR("Pyro: drogue_fail -> %d", src->drogue_fail);
    }
    if (src->main_fail != g_pyro_data.main_fail) {
        LOG_ERR("Pyro: main_fail -> %d", src->main_fail);
    }
    if (src->drogue_cont_ok != g_pyro_data.drogue_cont_ok) {
        LOG_INF("Pyro: drogue_cont_ok -> %d", src->drogue_cont_ok);
    }
    if (src->main_cont_ok != g_pyro_data.main_cont_ok) {
        LOG_INF("Pyro: main_cont_ok -> %d", src->main_cont_ok);
    }

    g_pyro_data = *src;
    k_mutex_unlock(&pyro_data_mutex);
}

void get_pyro_data(struct pyro_data *dst)
{
    k_mutex_lock(&pyro_data_mutex, K_FOREVER);
    *dst = g_pyro_data;
    k_mutex_unlock(&pyro_data_mutex);
}
