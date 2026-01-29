#include "sim_csv.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(sim_csv, LOG_LEVEL_INF);

#define MAX_LINE_LENGTH 4096

/**
 * Parse entire CSV line into float array using sscanf
 * Returns 0 on success, -1 on error
 */
static int parse_csv_row(const char *line, struct sim_csv_row *row)
{
    int result = sscanf(line,
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"  // 0-9
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"  // 10-19
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"  // 20-29
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"  // 30-39
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"  // 40-49
        "%f,%f,%f,%f",                     // 50-53
        &row->fields[0],  &row->fields[1],  &row->fields[2],  &row->fields[3],
        &row->fields[4],  &row->fields[5],  &row->fields[6],  &row->fields[7],
        &row->fields[8],  &row->fields[9],  &row->fields[10], &row->fields[11],
        &row->fields[12], &row->fields[13], &row->fields[14], &row->fields[15],
        &row->fields[16], &row->fields[17], &row->fields[18], &row->fields[19],
        &row->fields[20], &row->fields[21], &row->fields[22], &row->fields[23],
        &row->fields[24], &row->fields[25], &row->fields[26], &row->fields[27],
        &row->fields[28], &row->fields[29], &row->fields[30], &row->fields[31],
        &row->fields[32], &row->fields[33], &row->fields[34], &row->fields[35],
        &row->fields[36], &row->fields[37], &row->fields[38], &row->fields[39],
        &row->fields[40], &row->fields[41], &row->fields[42], &row->fields[43],
        &row->fields[44], &row->fields[45], &row->fields[46], &row->fields[47],
        &row->fields[48], &row->fields[49], &row->fields[50], &row->fields[51],
        &row->fields[52], &row->fields[53]
    );
    
    return (result == CSV_NUM_COLUMNS) ? 0 : -1;
}

/**
 * Read next CSV row from file
 * Returns 0 on success, -1 on EOF or error
 */
static int read_next_row(FILE *fp, struct sim_csv_row *row_out)
{
    char line[MAX_LINE_LENGTH];
    
    if (!fgets(line, sizeof(line), fp)) {
        return -1;  // EOF or error
    }
    
    return parse_csv_row(line, row_out);
}

/**
 * Scan entire file to get metadata
 */
static int scan_csv_metadata(FILE *fp, struct sim_csv_context *ctx)
{
    char line[MAX_LINE_LENGTH];
    struct sim_csv_row temp_row;
    
    ctx->csv_row_count = 0;
    bool first_row = true;
    
    // Skip header
    if (!fgets(line, sizeof(line), fp)) {
        return -EINVAL;
    }
    
    // Scan all data rows
    while (fgets(line, sizeof(line), fp)) {
        if (parse_csv_row(line, &temp_row) == 0) {
            int64_t timestamp = sim_csv_get_timestamp_ms(&temp_row);
            
            if (first_row) {
                ctx->csv_first_timestamp = timestamp;
                
                if (ctx->config->log_first_row) {
                    ctx->config->log_first_row(&temp_row);
                }
                
                first_row = false;
            }
            
            ctx->csv_last_timestamp = timestamp;
            ctx->csv_row_count++;
        }
    }
    
    // Rewind to start of file
    rewind(fp);
    
    // Skip header again
    fgets(line, sizeof(line), fp);
    
    return ctx->csv_row_count > 0 ? 0 : -EINVAL;
}

int sim_csv_load(struct sim_csv_context *ctx, 
                 const struct sim_csv_config *config)
{
    if (!ctx || !config) {
        return -EINVAL;
    }
    
    memset(ctx, 0, sizeof(*ctx));
    ctx->config = config;
    
    LOG_INF("═══════════════════════════════════════════════");
    LOG_INF("  CSV DATA LOADING - %s", config->sensor_name);
    LOG_INF("═══════════════════════════════════════════════");
    LOG_INF("Attempting to open: %s", config->filename);
    
    ctx->fp = fopen(config->filename, "r");
    if (!ctx->fp) {
        LOG_ERR("Failed to open CSV file: %s", config->filename);
        LOG_WRN("Falling back to SYNTHETIC data mode");
        return -ENOENT;
    }
    
    LOG_INF("File opened successfully");
    
    // Scan file for metadata
    LOG_INF("Scanning file for metadata...");
    if (scan_csv_metadata(ctx->fp, ctx) != 0) {
        fclose(ctx->fp);
        ctx->fp = NULL;
        LOG_ERR("No valid data rows in CSV");
        LOG_WRN("Falling back to SYNTHETIC data mode");
        return -EINVAL;
    }
    
    // Read first two rows into the sliding window
    if (read_next_row(ctx->fp, &ctx->row_curr) != 0) {
        fclose(ctx->fp);
        ctx->fp = NULL;
        LOG_ERR("Failed to read first row");
        return -EINVAL;
    }
    
    if (read_next_row(ctx->fp, &ctx->row_next) != 0) {
        // Only one row in file - that's OK
        ctx->end_of_file = true;
    }
    
    ctx->csv_loaded = true;
    ctx->csv_current_index = 0;
    
    int64_t duration_ms = ctx->csv_last_timestamp - ctx->csv_first_timestamp;
    
    LOG_INF("═══════════════════════════════════════════════");
    LOG_INF("  CSV LOAD SUCCESSFUL - %s", config->sensor_name);
    LOG_INF("═══════════════════════════════════════════════");
    LOG_INF("Rows in file: %zu", ctx->csv_row_count);
    LOG_INF("Time range: %lld to %lld ms", 
            ctx->csv_first_timestamp, ctx->csv_last_timestamp);
    LOG_INF("Duration: %.2f seconds", (double)duration_ms / 1000.0);
    LOG_INF("Memory usage: %zu bytes (2-row window)", 
            sizeof(struct sim_csv_row) * 2);
    
    // Optional: Sensor-specific summary
    if (config->log_summary) {
        // Need to get last row for summary
        struct sim_csv_row temp_first = ctx->row_curr;
        struct sim_csv_row temp_last;
        
        long saved_pos = ftell(ctx->fp);
        rewind(ctx->fp);
        char line[MAX_LINE_LENGTH];
        fgets(line, sizeof(line), ctx->fp); // skip header
        
        while (fgets(line, sizeof(line), ctx->fp)) {
            parse_csv_row(line, &temp_last);
        }
        
        config->log_summary(&temp_first, &temp_last, ctx->csv_row_count);
        
        // Restore file position
        fseek(ctx->fp, saved_pos, SEEK_SET);
    }
    
    LOG_INF("Mode: CSV PLAYBACK MODE (STREAMING)");
    LOG_INF("═══════════════════════════════════════════════");
    
    return 0;
}

void sim_csv_init_playback(struct sim_csv_context *ctx,
                           void *sensor_data,
                           int64_t now_ms)
{
    if (!ctx->csv_loaded) {
        return;
    }
    
    ctx->csv_start_time_ms = now_ms;
    ctx->sample_count = 0;
    
    // Initialize with first CSV values
    ctx->config->copy_to_sensor(sensor_data, &ctx->row_curr);
}

void sim_csv_update(struct sim_csv_context *ctx, 
                    void *sensor_data, 
                    int64_t now_ms)
{
    if (!ctx || !ctx->csv_loaded) {
        return;
    }
    
    ctx->sample_count++;
    
    // Calculate elapsed time since start
    int64_t elapsed_ms = now_ms - ctx->csv_start_time_ms;
    int64_t target_timestamp = ctx->csv_first_timestamp + elapsed_ms;
    
    // Get timestamps of current window
    int64_t curr_ts = sim_csv_get_timestamp_ms(&ctx->row_curr);
    int64_t next_ts = ctx->end_of_file ? 
                      ctx->csv_last_timestamp : 
                      sim_csv_get_timestamp_ms(&ctx->row_next);
    
    // Advance window if needed
    while (!ctx->end_of_file && next_ts < target_timestamp) {
        // Slide the window
        ctx->row_curr = ctx->row_next;
        ctx->csv_current_index++;
        
        // Read new next row
        if (read_next_row(ctx->fp, &ctx->row_next) != 0) {
            ctx->end_of_file = true;
            LOG_DBG("%s: Reached end of CSV file at index %zu",
                    ctx->config->sensor_name, ctx->csv_current_index);
            break;
        }
        
        curr_ts = sim_csv_get_timestamp_ms(&ctx->row_curr);
        next_ts = sim_csv_get_timestamp_ms(&ctx->row_next);
        
        LOG_DBG("%s: CSV window advanced to index %zu (CSV time: %lld ms)",
                ctx->config->sensor_name, ctx->csv_current_index, curr_ts);
    }
    
    // Handle edge case: before first data point
    if (target_timestamp < curr_ts) {
        ctx->config->copy_to_sensor(sensor_data, &ctx->row_curr);
        
        if (ctx->sample_count % 100 == 0) {
            LOG_DBG("%s: Using first CSV row (before start time)", 
                    ctx->config->sensor_name);
        }
        return;
    }
    
    // Handle edge case: past end of data
    if (ctx->end_of_file && target_timestamp >= next_ts) {
        ctx->config->copy_to_sensor(sensor_data, &ctx->row_curr);
        
        if (ctx->sample_count == 1 || (ctx->sample_count % 100 == 0)) {
            LOG_WRN("%s: End of CSV data reached - holding last values",
                    ctx->config->sensor_name);
        }
        return;
    }
    
    // Interpolate between current and next
    int64_t dt = next_ts - curr_ts;
    if (dt <= 0) dt = 1;
    
    float alpha = (float)(target_timestamp - curr_ts) / (float)dt;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    // Call sensor-specific interpolation
    ctx->config->interpolate(sensor_data, &ctx->row_curr, &ctx->row_next, alpha);
    
    // Periodic logging
    if (ctx->sample_count % 50 == 0) {
        LOG_INF("%s CSV: idx=%zu/%zu | t=%lld ms | α=%.3f",
                ctx->config->sensor_name,
                ctx->csv_current_index, ctx->csv_row_count - 1,
                target_timestamp,
                (double)alpha);
    }
}

void sim_csv_free(struct sim_csv_context *ctx)
{
    if (!ctx) {
        return;
    }
    
    if (ctx->fp) {
        fclose(ctx->fp);
        ctx->fp = NULL;
    }
    
    ctx->csv_loaded = false;
    ctx->csv_row_count = 0;
}