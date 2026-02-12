#ifndef DATA_H
#define DATA_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FLIGHT_STATE_STANDBY = 0,
    FLIGHT_STATE_ASCENT,
    FLIGHT_STATE_MACH_LOCK,
    FLIGHT_STATE_DROGUE_DESCENT,
    FLIGHT_STATE_MAIN_DESCENT,
    FLIGHT_STATE_LANDED,
} flight_state_id_t;

// Data structures for sensor data
struct imu_data {
    float accel[3];    // Acceleration in m/s²
    float gyro[3];     // Angular velocity in rad/s
    int64_t timestamp; // Timestamp in milliseconds
};

// Per-barometer sensor data
struct baro_sensor_data {
    float pressure;    // Pressure in Pa
    float temperature; // Temperature in °C
    float altitude;    // Altitude in meters (from pressure + temp)
    float nis;         // Normalized innovation squared
    uint8_t faults;    // Accumulated fault count
    bool healthy;      // Health status (accepted/rejected)
};

// Combined barometer data (shared with other threads)
struct baro_data {
    struct baro_sensor_data baro0; // Barometer 0
    struct baro_sensor_data baro1; // Barometer 1

    float altitude;     // Kalman-filtered altitude estimate in meters (absolute)
    float altitude_agl; // Kalman-filtered altitude AGL (relative to ground) in meters
    float alt_variance; // Kalman filter variance (P)
    float velocity;     // Vertical velocity estimate (m/s)
    float vel_variance; // Velocity variance (P11)

    int64_t timestamp; // Timestamp in milliseconds
};

struct state_data {
    flight_state_id_t state;
    float ground_altitude;
    bool ground_calibrated;
    int64_t timestamp;
};

struct pyro_data {
    uint8_t status_byte;
    int64_t timestamp;
    bool drogue_fired;
    bool main_fired;
    bool drogue_fail;
    bool main_fail;
    bool drogue_cont_ok;
    bool main_cont_ok;
    bool drogue_fire_ack;
    bool main_fire_ack;
    bool drogue_fire_requested;
    bool main_fire_requested;
};

// Global instances
extern struct imu_data g_imu_data;
extern struct baro_data g_baro_data;
extern struct state_data g_state_data;
extern struct pyro_data g_pyro_data;

// Getters and setters
void set_imu_data(const struct imu_data *src);
void get_imu_data(struct imu_data *dst);

void set_baro_data(const struct baro_data *src);
void get_baro_data(struct baro_data *dst);

void set_state_data(const struct state_data *src);
void get_state_data(struct state_data *dst);

void set_pyro_data(const struct pyro_data *src);
void get_pyro_data(struct pyro_data *dst);

#endif
