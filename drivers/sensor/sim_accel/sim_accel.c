#define DT_DRV_COMPAT zephyr_sim_accel

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>

struct sim_accel_data {
    float accel_x;
    float accel_y;
    float accel_z;
};

static int sim_accel_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct sim_accel_data *data = dev->data;

    /* Simulate gravity + small noise */
    float noise_x = ((int32_t)sys_rand32_get() % 1000) / 10000.0f - 0.05f;
    float noise_y = ((int32_t)sys_rand32_get() % 1000) / 10000.0f - 0.05f;
    float noise_z = ((int32_t)sys_rand32_get() % 1000) / 10000.0f - 0.05f;

    data->accel_x = 0.0f + noise_x;
    data->accel_y = 0.0f + noise_y;
    data->accel_z = -9.81f + noise_z; // Gravity

    return 0;
}

static int sim_accel_channel_get(const struct device *dev, enum sensor_channel chan,
                                 struct sensor_value *val)
{
    struct sim_accel_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
        sensor_value_from_float(val, data->accel_x);
        return 0;
    case SENSOR_CHAN_ACCEL_Y:
        sensor_value_from_float(val, data->accel_y);
        return 0;
    case SENSOR_CHAN_ACCEL_Z:
        sensor_value_from_float(val, data->accel_z);
        return 0;
    case SENSOR_CHAN_ACCEL_XYZ:
        sensor_value_from_float(&val[0], data->accel_x);
        sensor_value_from_float(&val[1], data->accel_y);
        sensor_value_from_float(&val[2], data->accel_z);
        return 0;
    default:
        return -ENOTSUP;
    }
}

static const struct sensor_driver_api sim_accel_api = {
    .sample_fetch = sim_accel_sample_fetch,
    .channel_get = sim_accel_channel_get,
};

#define SIM_ACCEL_INIT(inst)                                                           \
    static struct sim_accel_data sim_accel_data_##inst;                                \
    DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &sim_accel_data_##inst, NULL, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &sim_accel_api);

DT_INST_FOREACH_STATUS_OKAY(SIM_ACCEL_INIT)
