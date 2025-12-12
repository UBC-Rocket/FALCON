#ifndef DATA_H
#define DATA_H

#include <stdint.h>

// Data structures for sensor data
struct imu_data
{
    float accel[3];    // Acceleration in m/s²
    float gyro[3];     // Angular velocity in rad/s
    int64_t timestamp; // Timestamp in milliseconds
};

struct baro_data
{
    float pressure;    // Pressure in hPa (same as mBar)
    float temperature; // Temperature in °C
    float altitude;    // Altitude in meters
    int64_t timestamp; // Timestamp in milliseconds
};

// Global instances
extern struct imu_data g_imu_data;
extern struct baro_data g_baro_data;

// Getters and setters
void set_imu_data(const struct imu_data *src);
void get_imu_data(struct imu_data *dst);

void set_baro_data(const struct baro_data *src);
void get_baro_data(struct baro_data *dst);

#endif