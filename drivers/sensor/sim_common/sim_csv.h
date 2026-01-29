#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


#ifndef DATA_FILE
#define DATA_FILE ""
#endif

#define CSV_NUM_COLUMNS 54

// Standardized OpenRocket CSV column indices
#define CSV_COL_TIMESTAMP              0
#define CSV_COL_ALTITUDE               1
#define CSV_COL_VERTICAL_VELO          2
#define CSV_COL_VERTICAL_ACCEL         3
#define CSV_COL_TOTAL_VELO             4
#define CSV_COL_TOTAL_ACCEL            5
#define CSV_COL_POS_EAST               6
#define CSV_COL_POS_NORTH              7
#define CSV_COL_GPS_LAT_DIST           8
#define CSV_COL_GPS_LAT_DIR            9
#define CSV_COL_GPS_LAT_VELO          10
#define CSV_COL_GPS_LAT_ACCEL         11
#define CSV_COL_LATITUDE              12
#define CSV_COL_LONGITUDE             13
#define CSV_COL_GRAVITY               14
#define CSV_COL_ANGLE_ATTACK          15
#define CSV_COL_ROLL_RATE             16
#define CSV_COL_PITCH_RATE            17
#define CSV_COL_YAW_RATE              18
#define CSV_COL_MASS                  19
#define CSV_COL_MOT_MASS              20
#define CSV_COL_LONG_MMOI             21
#define CSV_COL_ROT_MMOI              22
#define CSV_COL_CP_LOCATION           23
#define CSV_COL_CG_LOCATION           24
#define CSV_COL_STABILITY             25
#define CSV_COL_MACH_NUMBER           26
#define CSV_COL_REYNOLDS_NUMBER       27
#define CSV_COL_THRUST                28
#define CSV_COL_DRAG                  29
#define CSV_COL_DRAG_COEFF            30
#define CSV_COL_AXIAL_DRAG_COEFF      31
#define CSV_COL_FRIC_DRAG_COEFF       32
#define CSV_COL_PRESSURE_DRAG_COEFF   33
#define CSV_COL_BASE_DRAG_COEFF       34
#define CSV_COL_NORM_FORCE_COEFF      35
#define CSV_COL_PITCH_MOM_COEFF       36
#define CSV_COL_YAW_MOM_COEFF         37
#define CSV_COL_SIDE_FORCE_COEFF      38
#define CSV_COL_ROLL_MOM_COEFF        39
#define CSV_COL_ROLL_FORCING_COEFF    40
#define CSV_COL_ROLL_DAMPING_COEFF    41
#define CSV_COL_PITCH_DAMPING_COEFF   42
#define CSV_COL_CORIOLIS_ACCEL        43
#define CSV_COL_REF_LENGTH            44
#define CSV_COL_REF_AREA              45
#define CSV_COL_VERTICAL_ORIENT       46
#define CSV_COL_LATERAL_ORIENT        47
#define CSV_COL_WIND_SPEED            48
#define CSV_COL_AIR_TEMP              49
#define CSV_COL_AIR_PRESSURE          50
#define CSV_COL_SPEED_OF_SOUND        51
#define CSV_COL_SIM_TIMESTEP          52
#define CSV_COL_COMPUTATION_TIME      53

struct sim_csv_row {
    float fields[CSV_NUM_COLUMNS];
};

struct sim_csv_config {
    const char *filename;
    const char *sensor_name;
    
    /**
     * Copy data from CSV row to sensor data structure
     * @param sensor_data: Pointer to sensor's data structure
     * @param csv_row: Parsed CSV row (array of floats)
     */
    void (*copy_to_sensor)(void *sensor_data, const struct sim_csv_row *csv_row);
    
    /**
     * Interpolate between two CSV rows and write to sensor data
     * @param sensor_data: Pointer to sensor's data structure
     * @param row_curr: Current CSV row
     * @param row_next: Next CSV row
     * @param alpha: Interpolation factor [0.0, 1.0]
     */
    void (*interpolate)(void *sensor_data, const struct sim_csv_row *row_curr, 
                       const struct sim_csv_row *row_next, float alpha);
    
    /**
     * Optional: Log first data point details
     */
    void (*log_first_row)(const struct sim_csv_row *row);
    
    /**
     * Optional: Log data range/summary after scanning
     */
    void (*log_summary)(const struct sim_csv_row *first_row, 
                       const struct sim_csv_row *last_row, 
                       size_t row_count);
};

struct sim_csv_context {
    FILE *fp;
    
    struct sim_csv_row row_curr;    // Current CSV row
    struct sim_csv_row row_next;    // Next CSV row
    
    int64_t csv_start_time_ms;      // Simulation start time
    int64_t csv_first_timestamp;    // First timestamp in CSV (ms)
    int64_t csv_last_timestamp;     // Last timestamp in CSV (ms)
    
    size_t csv_row_count;           // Total rows in file
    size_t csv_current_index;       // Current position
    
    bool csv_loaded;
    bool end_of_file;
    
    const struct sim_csv_config *config;
    uint32_t sample_count;
};

int sim_csv_load(struct sim_csv_context *ctx, 
                 const struct sim_csv_config *config);
void sim_csv_update(struct sim_csv_context *ctx, 
                    void *sensor_data, 
                    int64_t now_ms);
void sim_csv_free(struct sim_csv_context *ctx);
void sim_csv_init_playback(struct sim_csv_context *ctx,
                           void *sensor_data,
                           int64_t now_ms);

/**
 * Helper: Get timestamp from CSV row (column 0, converted to ms)
 */
static inline int64_t sim_csv_get_timestamp_ms(const struct sim_csv_row *row)
{
    return (int64_t)(row->fields[CSV_COL_TIMESTAMP] * 1000.0f);
}