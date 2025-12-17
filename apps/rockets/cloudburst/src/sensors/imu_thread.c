#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "../data.h"

LOG_MODULE_REGISTER(imu_thread, LOG_LEVEL_INF);

#define IMU_THREAD_STACK 2048
#define IMU_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(imu_stack, IMU_THREAD_STACK);
static struct k_thread imu_thread;

static void imu_thread_fn(void *sleep_time_ptr, void *p2, void *p3)
{
    const struct device *gyro_dev = DEVICE_DT_GET(DT_ALIAS(gyro0));
    const struct device *accel_dev = DEVICE_DT_GET(DT_ALIAS(accel0));

    if (!device_is_ready(accel_dev) || !device_is_ready(gyro_dev)) {
        LOG_ERR("BMI088 not ready");
        return;
    }

    // Retrieve the sleep time passed as a parameter
    uint32_t sleep_time_ms = *(uint32_t *)sleep_time_ptr;

    while (1) {
        struct sensor_value accel[3];
        struct sensor_value gyro[3];
        struct imu_data imu_sample;

        if (sensor_sample_fetch(accel_dev) < 0 ||
            sensor_sample_fetch(gyro_dev) < 0) {
            LOG_ERR("Failed to fetch samples from BMI088");
            k_sleep(K_MSEC(sleep_time_ms));
            continue;
        }

        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &accel[0]);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]);

        sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_X, &gyro[0]);
        sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_Y, &gyro[1]);
        sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_Z, &gyro[2]);

        imu_sample.accel[0] = sensor_value_to_double(&accel[0]);
        imu_sample.accel[1] = sensor_value_to_double(&accel[1]);
        imu_sample.accel[2] = sensor_value_to_double(&accel[2]);

        imu_sample.gyro[0] = sensor_value_to_double(&gyro[0]);
        imu_sample.gyro[1] = sensor_value_to_double(&gyro[1]);
        imu_sample.gyro[2] = sensor_value_to_double(&gyro[2]);

        imu_sample.timestamp = k_uptime_get();

        set_imu_data(&imu_sample);

        k_sleep(K_MSEC(sleep_time_ms));
    }
}

void start_imu_thread(uint32_t *sleep_time_ms)
{
    k_thread_create(
        &imu_thread,
        imu_stack,
        K_THREAD_STACK_SIZEOF(imu_stack),
        imu_thread_fn,
        sleep_time_ms, NULL, NULL,  // Pass sleep time as a parameter
        IMU_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );
}