#define DT_DRV_COMPAT zephyr_sim_baro

#include "sim_csv.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(sim_baro, LOG_LEVEL_INF);

#ifndef DATA_FILE
#define DATA_FILE ""
#endif

struct sim_baro_data {
    float altitude_m;
    float velocity_mps;
    float pressure_hpa;
    float temperature_c;
    int64_t last_ms;
    
    struct sim_csv_context csv_ctx;
};


static void baro_copy_to_sensor(void *sensor_data, const struct sim_csv_row *csv_row)
{
    struct sim_baro_data *data = (struct sim_baro_data *)sensor_data;
    
    data->pressure_hpa = csv_row->fields[CSV_COL_AIR_PRESSURE];
    data->temperature_c = csv_row->fields[CSV_COL_AIR_TEMP];
    data->altitude_m = csv_row->fields[CSV_COL_ALTITUDE];
}

static void baro_interpolate(void *sensor_data, 
                             const struct sim_csv_row *row_curr,
                             const struct sim_csv_row *row_next, 
                             float alpha)
{
    struct sim_baro_data *data = (struct sim_baro_data *)sensor_data;
    
    data->pressure_hpa = row_curr->fields[CSV_COL_AIR_PRESSURE] + 
                         alpha * (row_next->fields[CSV_COL_AIR_PRESSURE] - 
                                 row_curr->fields[CSV_COL_AIR_PRESSURE]);
    
    data->temperature_c = row_curr->fields[CSV_COL_AIR_TEMP] + 
                          alpha * (row_next->fields[CSV_COL_AIR_TEMP] - 
                                  row_curr->fields[CSV_COL_AIR_TEMP]);
    
    data->altitude_m = row_curr->fields[CSV_COL_ALTITUDE] + 
                       alpha * (row_next->fields[CSV_COL_ALTITUDE] - 
                               row_curr->fields[CSV_COL_ALTITUDE]);
}

static void baro_log_first_row(const struct sim_csv_row *row)
{
    LOG_INF("First data point: t=%.3f s, p=%.2f hPa, alt=%.2f m, T=%.2f C",
            (double)row->fields[CSV_COL_TIMESTAMP],
            (double)row->fields[CSV_COL_AIR_PRESSURE],
            (double)row->fields[CSV_COL_ALTITUDE],
            (double)row->fields[CSV_COL_AIR_TEMP]);
}

static void baro_log_summary(const struct sim_csv_row *first_row, 
                             const struct sim_csv_row *last_row,
                             size_t row_count)
{
    LOG_INF("Altitude range: %.2f to %.2f m", 
            (double)first_row->fields[CSV_COL_ALTITUDE],
            (double)last_row->fields[CSV_COL_ALTITUDE]);
}

static const struct sim_csv_config baro_csv_config = {
    .filename = DATA_FILE,
    .sensor_name = "BARO",
    .copy_to_sensor = baro_copy_to_sensor,
    .interpolate = baro_interpolate,
    .log_first_row = baro_log_first_row,
    .log_summary = baro_log_summary,
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
        
        LOG_INF("═══════════════════════════════════════════════");
        LOG_INF("  SIM_BARO INITIALIZATION");
        LOG_INF("═══════════════════════════════════════════════");
        
        // Try to load CSV data
        if (strlen(DATA_FILE) > 0) {
            LOG_INF("DATA_FILE defined: \"%s\"", DATA_FILE);
            if (sim_csv_load(&data->csv_ctx, &baro_csv_config) == 0) {
                sim_csv_init_playback(&data->csv_ctx, data, now);
            }
        } else {
            LOG_INF("DATA_FILE not defined (empty string)");
            LOG_INF("Mode: SYNTHETIC DATA MODE");
            LOG_INF("═══════════════════════════════════════════════");
            
            // Initialize synthetic values
            data->altitude_m = 0.0f;
            data->velocity_mps = 0.0f;
            data->pressure_hpa = 1013.25f;
            data->temperature_c = 20.0f;
        }
        
        return 0;
    }
    
    data->last_ms = now;
    if (dt <= 0) dt = 0.02f;

    if (data->csv_ctx.csv_loaded) {
        // Use CSV data
        sim_csv_update(&data->csv_ctx, data, now);
        
    } else {
        // Synthetic data mode
        data->velocity_mps += 0.1f * dt;
        data->altitude_m += data->velocity_mps * dt;

        float noise = ((int32_t)sys_rand32_get() % 1000) / 1000.0f - 0.5f;
        data->altitude_m += noise * 0.05f;

        float pressure_pa = 101325.0f *
            powf(1.0f - (data->altitude_m / 44330.0f), 5.255f);
        
        data->pressure_hpa = pressure_pa / 100.0f;
        data->temperature_c = 20.0f;
        
        if (data->csv_ctx.sample_count % 50 == 0) {
            LOG_INF("SYN: p=%.2f hPa | alt=%.2f m | v=%.2f m/s | T=%.2f C",
                    (double)data->pressure_hpa, (double)data->altitude_m,
                    (double)data->velocity_mps, (double)data->temperature_c);
        }
        data->csv_ctx.sample_count++;
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
        .pressure_hpa = 1013.25f,                           \
        .temperature_c = 20.0f,                             \
        .altitude_m = 0.0f,                                 \
        .velocity_mps = 0.0f,                               \
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