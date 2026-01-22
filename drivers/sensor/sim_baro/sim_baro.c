#define DT_DRV_COMPAT zephyr_sim_baro

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>

struct sim_baro_data {
    float altitude_m;
    float velocity_mps;
    float pressure_pa;
    float temperature_c;
};

static int sim_baro_sample_fetch(const struct device *dev,
                                 enum sensor_channel chan)
{
    struct sim_baro_data *data = dev->data;

    static int64_t last_ms;
    int64_t now = k_uptime_get();
    float dt = (now - last_ms) / 1000.0f;
    last_ms = now;

    if (dt <= 0) dt = 0.02f;

    /* Example: simple vertical motion */
    data->velocity_mps += 0.1f * dt;      // acceleration
    data->altitude_m   += data->velocity_mps * dt;

    /* Add noise */
    float noise = ((int32_t)sys_rand32_get() % 1000) / 1000.0f - 0.5f;
    data->altitude_m += noise * 0.05f;

    /* Convert altitude → pressure (ISA approx) */
    data->pressure_pa = 101325.0f *
        powf(1.0f - (data->altitude_m / 44330.0f), 5.255f);

    data->temperature_c = 20.0f;

    return 0;
}

static int sim_baro_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val)
{
    struct sim_baro_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_PRESS:
        sensor_value_from_float(val, data->pressure_pa);
        return 0;
    case SENSOR_CHAN_ALTITUDE:
        sensor_value_from_float(val, data->altitude_m);
        return 0;
    case SENSOR_CHAN_AMBIENT_TEMP:
        sensor_value_from_float(val, data->temperature_c);
        return 0;
    default:
        return -ENOTSUP;
    }
}

static const struct sensor_driver_api sim_baro_api = {
    .sample_fetch = sim_baro_sample_fetch,
    .channel_get  = sim_baro_channel_get,
};

static struct sim_baro_data sim_baro_0;
static struct sim_baro_data sim_baro_1;

DEVICE_DT_INST_DEFINE(0, NULL, NULL,
                      &sim_baro_0, NULL,
                      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
                      &sim_baro_api);

DEVICE_DT_INST_DEFINE(1, NULL, NULL,
                      &sim_baro_1, NULL,
                      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
                      &sim_baro_api);

#define SIM_BARO_INIT(inst)                                 \
    static struct sim_baro_data sim_baro_data_##inst;       \
    DEVICE_DT_INST_DEFINE(inst,                             \
                          NULL,                             \
                          NULL,                             \
                          &sim_baro_data_##inst,            \
                          NULL, POST_KERNEL,                \
                          CONFIG_SENSOR_INIT_PRIORITY,      \
                          &sim_baro_api);

DT_INST_FOREACH_STATUS_OKAY(SIM_BARO_INIT)                      