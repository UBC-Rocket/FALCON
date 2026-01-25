#define DT_DRV_COMPAT zephyr_sim_baro

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <math.h>

#define DATA_FILE ""

struct sim_baro_data {
    float altitude_m;
    float velocity_mps;
    float pressure_hpa;      // Changed: now in hPa to match baro_thread expectations
    float temperature_c;
    int64_t last_ms;
};

static int sim_baro_sample_fetch(const struct device *dev,
                                 enum sensor_channel chan)
{
    struct sim_baro_data *data = dev->data;

    int64_t now = k_uptime_get();
    float dt = (now - data->last_ms) / 1000.0f;
    
    // Initialize on first call
    if (data->last_ms == 0) {
        data->last_ms = now;
        return 0;
    }
    
    data->last_ms = now;

    if (dt <= 0) dt = 0.02f;

    if (DATA_FILE != "") {

    } else {

      /* Example: simple vertical motion */
      data->velocity_mps += 0.1f * dt;
      data->altitude_m   += data->velocity_mps * dt;

      /* Add noise */
      float noise = ((int32_t)sys_rand32_get() % 1000) / 1000.0f - 0.5f;
      data->altitude_m += noise * 0.05f;

      /* Convert altitude → pressure (ISA approx), result in hPa */
      float pressure_pa = 101325.0f *
          powf(1.0f - (data->altitude_m / 44330.0f), 5.255f);
      
      data->pressure_hpa = pressure_pa / 100.0f;  // Convert Pa to hPa

      data->temperature_c = 20.0f;

    }

    return 0;
}

static int sim_baro_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val)
{
    struct sim_baro_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_PRESS:
        // Return pressure in hPa (baro_thread converts to Pa)
        sensor_value_from_float(val, data->pressure_hpa);
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

#define SIM_BARO_INIT(inst)                                 \
    static struct sim_baro_data sim_baro_data_##inst = {    \
        .last_ms = 0,                                       \
    };                                                      \
    DEVICE_DT_INST_DEFINE(inst,                             \
                          NULL,                             \
                          NULL,                             \
                          &sim_baro_data_##inst,            \
                          NULL,                             \
                          POST_KERNEL,                      \
                          CONFIG_SENSOR_INIT_PRIORITY,      \
                          &sim_baro_api);

DT_INST_FOREACH_STATUS_OKAY(SIM_BARO_INIT)