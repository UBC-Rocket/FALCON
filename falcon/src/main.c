#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "sensors/imu_thread.h"
#include "data.h"

LOG_MODULE_REGISTER(falcon_main, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("Falcon application started");

    // Sensor sleep times in milliseconds
    uint32_t imu_sleep_time_ms = 50;

    // Start the threads
    start_imu_thread(&imu_sleep_time_ms);

    while (1) {
        struct imu_data imu_sample;

        // Retrieve the latest IMU data
        get_imu_data(&imu_sample);

        // Log the IMU data
        LOG_INF("IMU Data: Accel [X=%.3f, Y=%.3f, Z=%.3f] m/s^2, "
                "Gyro [X=%.3f, Y=%.3f, Z=%.3f] rad/s, "
                "Timestamp: %lld ms",
                imu_sample.accel[0], imu_sample.accel[1], imu_sample.accel[2],
                imu_sample.gyro[0], imu_sample.gyro[1], imu_sample.gyro[2],
                imu_sample.timestamp);

        // Sleep for 1 second
        k_sleep(K_SECONDS(1));
    }

    return 0;
}