#define DT_DRV_COMPAT zephyr_sim_gyro

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>

struct sim_gyro_data {
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

static int sim_gyro_sample_fetch(const struct device *dev,
                                  enum sensor_channel chan)
{
    struct sim_gyro_data *data = dev->data;

    /* Simulate small random rotation rates */
    float noise_x = ((int32_t)sys_rand32_get() % 1000) / 50000.0f - 0.01f;
    float noise_y = ((int32_t)sys_rand32_get() % 1000) / 50000.0f - 0.01f;
    float noise_z = ((int32_t)sys_rand32_get() % 1000) / 50000.0f - 0.01f;

    data->gyro_x = noise_x;
    data->gyro_y = noise_y;
    data->gyro_z = noise_z;

    return 0;
}

static int sim_gyro_channel_get(const struct device *dev,
                                 enum sensor_channel chan,
                                 struct sensor_value *val)
{
    struct sim_gyro_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_GYRO_X:
        sensor_value_from_float(val, data->gyro_x);
        return 0;
    case SENSOR_CHAN_GYRO_Y:
        sensor_value_from_float(val, data->gyro_y);
        return 0;
    case SENSOR_CHAN_GYRO_Z:
        sensor_value_from_float(val, data->gyro_z);
        return 0;
    case SENSOR_CHAN_GYRO_XYZ:
        sensor_value_from_float(&val[0], data->gyro_x);
        sensor_value_from_float(&val[1], data->gyro_y);
        sensor_value_from_float(&val[2], data->gyro_z);
        return 0;
    default:
        return -ENOTSUP;
    }
}

static const struct sensor_driver_api sim_gyro_api = {
    .sample_fetch = sim_gyro_sample_fetch,
    .channel_get  = sim_gyro_channel_get,
};

#define SIM_GYRO_INIT(inst)                                 \
    static struct sim_gyro_data sim_gyro_data_##inst;       \
    DEVICE_DT_INST_DEFINE(inst,                             \
                          NULL,                             \
                          NULL,                             \
                          &sim_gyro_data_##inst,            \
                          NULL,                             \
                          POST_KERNEL,                      \
                          CONFIG_SENSOR_INIT_PRIORITY,      \
                          &sim_gyro_api);

DT_INST_FOREACH_STATUS_OKAY(SIM_GYRO_INIT)