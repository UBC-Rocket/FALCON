#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_test, LOG_LEVEL_INF);

// Define which sensors to test
#define TEST_BARO_SENSOR 1
#define TEST_GYRO_SENSOR 1
#define TEST_ACCEL_SENSOR 1

int main(void) {
  LOG_INF("Sensor test application started");

  // Initialize sensor devices
  const struct device *baro_dev = NULL;
  const struct device *gyro_dev = NULL;
  const struct device *accel_dev = NULL;

#if TEST_BARO_SENSOR
  baro_dev = DEVICE_DT_GET(DT_ALIAS(baro0));
  if (!device_is_ready(baro_dev)) {
    LOG_ERR("Barometer sensor (baro0) is not ready");
    baro_dev = NULL;
  } else {
    LOG_INF("Barometer sensor initialized");
  }
#endif

#if TEST_GYRO_SENSOR
  gyro_dev = DEVICE_DT_GET(DT_ALIAS(gyro0));
  if (!device_is_ready(gyro_dev)) {
    LOG_ERR("Gyroscope (gyro0) is not ready");
    gyro_dev = NULL;
  } else {
    LOG_INF("Gyroscope initialized");
  }
#endif

#if TEST_ACCEL_SENSOR
  accel_dev = DEVICE_DT_GET(DT_ALIAS(accel0));
  if (!device_is_ready(accel_dev)) {
    LOG_ERR("Accelerometer (accel0) is not ready");
    accel_dev = NULL;
  } else {
    LOG_INF("Accelerometer initialized");
  }
#endif

  // Sensor values
  struct sensor_value pressure, temperature;
  struct sensor_value gyro_x, gyro_y, gyro_z;
  struct sensor_value accel_x, accel_y, accel_z;

  // Main polling loop
  while (1) {
    LOG_INF("--------------------------------------------------");
#if TEST_BARO_SENSOR
    if (baro_dev) {
      if (sensor_sample_fetch_chan(baro_dev, SENSOR_CHAN_ALL) == 0) {
        sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &pressure);
        sensor_channel_get(baro_dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);

        LOG_INF("Pressure: %.2f mPa, Temperature: %.2f degC",
                sensor_value_to_double(&pressure),
                sensor_value_to_double(&temperature));
      } else {
        LOG_ERR("Failed to fetch data from barometer sensor");
      }
    }
#endif

#if TEST_GYRO_SENSOR
    if (gyro_dev) {
      if (sensor_sample_fetch(gyro_dev) == 0) {
        sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
        sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
        sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

        LOG_INF("Gyroscope (rad/s): X=%.2f, Y=%.2f, Z=%.2f",
                sensor_value_to_double(&gyro_x),
                sensor_value_to_double(&gyro_y),
                sensor_value_to_double(&gyro_z));
      } else {
        LOG_ERR("Failed to fetch data from gyroscope");
      }
    }
#endif

#if TEST_ACCEL_SENSOR
    if (accel_dev) {
      if (sensor_sample_fetch(accel_dev) == 0) {
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

        LOG_INF("Acceleration (m/s^2): X=%.2f, Y=%.2f, Z=%.2f",
                sensor_value_to_double(&accel_x),
                sensor_value_to_double(&accel_y),
                sensor_value_to_double(&accel_z));
      } else {
        LOG_ERR("Failed to fetch data from accelerometer");
      }
    }
#endif

    // Sleep for 1 second before polling again
    k_sleep(K_MSEC(1000));
  }
  return 0;
}