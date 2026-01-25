#define DT_DRV_COMPAT zephyr_sim_baro

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

LOG_MODULE_REGISTER(sim_baro, LOG_LEVEL_INF);

#ifndef DATA_FILE
#define DATA_FILE ""
#endif
#define MAX_LINE_LENGTH 512
#define MAX_CSV_ROWS 10000

struct csv_row {
    int64_t timestamp_ms;
    float pressure_mbar;  // mbar = hPa
    float temperature_c;
    float altitude_m;
};

struct sim_baro_data {
    float altitude_m;
    float velocity_mps;
    float pressure_hpa;
    float temperature_c;
    int64_t last_ms;
    
    // CSV playback data
    struct csv_row *csv_data;
    size_t csv_row_count;
    size_t csv_current_index;
    int64_t csv_start_time_ms;
    int64_t csv_first_timestamp;
    bool csv_loaded;
    
    // Logging control
    uint32_t sample_count;
};


static int parse_csv_line(const char *line, struct csv_row *row)
{
    char buffer[MAX_LINE_LENGTH];
    strncpy(buffer, line, MAX_LINE_LENGTH - 1);
    buffer[MAX_LINE_LENGTH - 1] = '\0';
    
    char *token;
    char *saveptr;
    int field = 0;
    
    token = strtok_r(buffer, ",", &saveptr);
    while (token != NULL) {
        switch (field) {
            case 0: // Timestamp (ms)
                row->timestamp_ms = (int64_t)atoll(token);
                break;
            case 2: // Altitude (m)
                row->altitude_m = atof(token);
                break;
            case 4: // Pressure (mbar)
                row->pressure_mbar = atof(token) / 100.0f; // Convert cmbar to mbar
                break;
            case 5: // Barom. Temp (0.01 C) - need to divide by 100
                row->temperature_c = atof(token) / 100.0f;
                break;
        }
        field++;
        token = strtok_r(NULL, ",", &saveptr);
    }
    
    // Validate we got the required fields
    return (field >= 6) ? 0 : -1;
}

static int load_csv_data(struct sim_baro_data *data)
{
    LOG_INF("═══════════════════════════════════════════════");
    LOG_INF("  CSV DATA LOADING");
    LOG_INF("═══════════════════════════════════════════════");
    LOG_INF("Attempting to open: %s", DATA_FILE);
    
    // In Zephyr native_sim, we can use standard C file I/O
    FILE *fp = fopen(DATA_FILE, "r");
    if (!fp) {
        LOG_ERR("❌ Failed to open CSV file: %s", DATA_FILE);
        LOG_WRN("Falling back to SYNTHETIC data mode");
        return -1;
    }
    
    LOG_INF("✓ File opened successfully");
    
    // Allocate memory for CSV data
    data->csv_data = malloc(sizeof(struct csv_row) * MAX_CSV_ROWS);
    if (!data->csv_data) {
        fclose(fp);
        LOG_ERR("❌ Failed to allocate memory for CSV data");
        LOG_WRN("Falling back to SYNTHETIC data mode");
        return -ENOMEM;
    }
    
    LOG_INF("✓ Memory allocated for %d rows", MAX_CSV_ROWS);
    
    char line[MAX_LINE_LENGTH];
    data->csv_row_count = 0;
    
    // Skip header line
    if (!fgets(line, sizeof(line), fp)) {
        fclose(fp);
        k_free(data->csv_data);
        LOG_ERR("❌ Empty CSV file");
        LOG_WRN("Falling back to SYNTHETIC data mode");
        return -1;
    }
    
    LOG_INF("✓ Header line read");
    
    // Read data rows
    while (data->csv_row_count < MAX_CSV_ROWS && fgets(line, sizeof(line), fp)) {
        struct csv_row *row = &data->csv_data[data->csv_row_count];
        
        if (parse_csv_line(line, row) == 0) {
            // Store the first timestamp for relative time calculation
            if (data->csv_row_count == 0) {
                data->csv_first_timestamp = row->timestamp_ms;
                LOG_INF("✓ First data point: t=%lld ms, p=%.2f mbar, alt=%.2f m, T=%.2f C",
                        row->timestamp_ms, 
                        (double)row->pressure_mbar,
                        (double)row->altitude_m,
                        (double)row->temperature_c);
            }
            data->csv_row_count++;
        }
    }
    
    fclose(fp);
    
    if (data->csv_row_count == 0) {
        k_free(data->csv_data);
        LOG_ERR("❌ No valid data rows in CSV");
        LOG_WRN("Falling back to SYNTHETIC data mode");
        return -1;
    }
    
    data->csv_loaded = true;
    
    struct csv_row *last_row = &data->csv_data[data->csv_row_count - 1];
    int64_t duration_ms = last_row->timestamp_ms - data->csv_first_timestamp;
    
    LOG_INF("═══════════════════════════════════════════════");
    LOG_INF("  CSV LOAD SUCCESSFUL");
    LOG_INF("═══════════════════════════════════════════════");
    LOG_INF("Rows loaded: %zu", data->csv_row_count);
    LOG_INF("Time range: %lld to %lld ms", 
            data->csv_data[0].timestamp_ms,
            last_row->timestamp_ms);
    LOG_INF("Duration: %.2f seconds", (double)duration_ms / 1000.0);
    LOG_INF("Altitude range: %.2f to %.2f m", 
            (double)data->csv_data[0].altitude_m,
            (double)last_row->altitude_m);
    LOG_INF("Mode: 📊 CSV PLAYBACK MODE");
    LOG_INF("═══════════════════════════════════════════════");
    
    return 0;
}

static void interpolate_csv_data(struct sim_baro_data *data, int64_t now_ms)
{
    if (!data->csv_loaded || data->csv_row_count == 0) {
        return;
    }
    
    // Calculate elapsed time since start (in simulation time)
    int64_t elapsed_ms = now_ms - data->csv_start_time_ms;
    
    // Map to CSV timestamp (relative to first CSV timestamp)
    int64_t target_timestamp = data->csv_first_timestamp + elapsed_ms;
    
    // Find the two surrounding data points
    size_t i = data->csv_current_index;
    
    // Advance index if needed
    while (i < data->csv_row_count - 1 && 
           data->csv_data[i + 1].timestamp_ms <= target_timestamp) {
        i++;
    }
    
    // Log when we advance to a new index
    if (i != data->csv_current_index) {
        LOG_DBG("CSV index advanced: %zu -> %zu (CSV time: %lld ms)", 
                data->csv_current_index, i, 
                data->csv_data[i].timestamp_ms);
    }
    
    data->csv_current_index = i;
    
    // Handle edge cases
    if (i == 0 && target_timestamp < data->csv_data[0].timestamp_ms) {
        // Before first data point, use first row
        struct csv_row *row = &data->csv_data[0];
        data->pressure_hpa = row->pressure_mbar;  // mbar = hPa
        data->temperature_c = row->temperature_c;
        data->altitude_m = row->altitude_m;
        
        if (data->sample_count % 100 == 0) {
            LOG_DBG("Using first CSV row (before start time)");
        }
        return;
    }
    
    if (i >= data->csv_row_count - 1) {
        // Past end of data, use last row
        struct csv_row *row = &data->csv_data[data->csv_row_count - 1];
        data->pressure_hpa = row->pressure_mbar;
        data->temperature_c = row->temperature_c;
        data->altitude_m = row->altitude_m;
        
        if (data->sample_count == 0 || (data->sample_count % 100 == 0)) {
            LOG_WRN("⚠️  End of CSV data reached - holding last values");
        }
        return;
    }
    
    // Interpolate between current and next point
    struct csv_row *curr = &data->csv_data[i];
    struct csv_row *next = &data->csv_data[i + 1];
    
    int64_t dt = next->timestamp_ms - curr->timestamp_ms;
    if (dt <= 0) dt = 1;
    
    float alpha = (float)(target_timestamp - curr->timestamp_ms) / (float)dt;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    // Linear interpolation
    data->pressure_hpa = curr->pressure_mbar + alpha * (next->pressure_mbar - curr->pressure_mbar);
    data->temperature_c = curr->temperature_c + alpha * (next->temperature_c - curr->temperature_c);
    data->altitude_m = curr->altitude_m + alpha * (next->altitude_m - curr->altitude_m);
    
    // Periodic detailed logging (every 50 samples)
    if (data->sample_count % 50 == 0) {
        LOG_INF("📊 CSV: idx=%zu/%zu | t=%lld ms | p=%.2f hPa | alt=%.2f m | T=%.2f C | α=%.3f",
                i, data->csv_row_count - 1,
                target_timestamp,
                (double)data->pressure_hpa,
                (double)data->altitude_m,
                (double)data->temperature_c,
                (double)alpha);
    }
}

static int sim_baro_sample_fetch(const struct device *dev,
                                 enum sensor_channel chan)
{
    struct sim_baro_data *data = dev->data;

    int64_t now = k_uptime_get();
    float dt = (now - data->last_ms) / 1000.0f;
    
    // Initialize on first call
    if (data->last_ms == 0) {
        data->last_ms = now;
        data->sample_count = 0;
        
        LOG_INF("═══════════════════════════════════════════════");
        LOG_INF("  SIM_BARO INITIALIZATION");
        LOG_INF("═══════════════════════════════════════════════");
        
        // Load CSV data if DATA_FILE is defined
        if (strlen(DATA_FILE) > 0) {
            LOG_INF("DATA_FILE defined: \"%s\"", DATA_FILE);
            if (load_csv_data(data) == 0) {
                data->csv_start_time_ms = now;
                data->csv_current_index = 0;
                
                // Initialize with first CSV values
                struct csv_row *first = &data->csv_data[0];
                data->pressure_hpa = first->pressure_mbar;
                data->temperature_c = first->temperature_c;
                data->altitude_m = first->altitude_m;
            }
        } else {
            LOG_INF("DATA_FILE not defined (empty string)");
            LOG_INF("Mode: 🔧 SYNTHETIC DATA MODE");
            LOG_INF("═══════════════════════════════════════════════");
            
            // Initialize synthetic values
            data->altitude_m = 0.0f;
            data->velocity_mps = 0.0f;
            data->pressure_hpa = 1013.25f;
            data->temperature_c = 20.0f;
        }
        
        return 0;  // Now returns with valid data
    }
    
    data->last_ms = now;
    data->sample_count++;

    if (dt <= 0) dt = 0.02f;

    if (strlen(DATA_FILE) > 0 && data->csv_loaded) {
        // Use CSV data
        interpolate_csv_data(data, now);
        
    } else {
        // Synthetic data
        data->velocity_mps += 0.1f * dt;
        data->altitude_m   += data->velocity_mps * dt;

        /* Add noise */
        float noise = ((int32_t)sys_rand32_get() % 1000) / 1000.0f - 0.5f;
        data->altitude_m += noise * 0.05f;

        /* Convert altitude → pressure (ISA approx), result in hPa */
        float pressure_pa = 101325.0f *
            powf(1.0f - (data->altitude_m / 44330.0f), 5.255f);
        
        data->pressure_hpa = pressure_pa / 100.0f;

        data->temperature_c = 20.0f;
        
        // Periodic logging for synthetic mode (every 50 samples)
        if (data->sample_count % 50 == 0) {
            LOG_INF("🔧 SYN: p=%.2f hPa | alt=%.2f m | v=%.2f m/s | T=%.2f C",
                    (double)data->pressure_hpa,
                    (double)data->altitude_m,
                    (double)data->velocity_mps,
                    (double)data->temperature_c);
        }
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
        .csv_loaded = false,                                \
        .sample_count = 0,                                  \
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