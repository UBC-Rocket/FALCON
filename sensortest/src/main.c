#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

int main(void)
{
    const struct device *accel = DEVICE_DT_GET(DT_ALIAS(accel0));
    const struct device *gyro  = DEVICE_DT_GET(DT_ALIAS(gyro0));

    if (!device_is_ready(accel)) {
        printk("Accel not ready (check DT node, properties, status, Kconfig)\n");
    }
    if (!device_is_ready(gyro)) {
        printk("Gyro not ready (check DT node, properties, status, Kconfig)\n");
    }
    if (!device_is_ready(accel) || !device_is_ready(gyro)) {
        return -1;
    }

    printk("Polling BMI088 (no interrupts)...\n");

    while (1) {
        if (sensor_sample_fetch(accel) == 0) {
            struct sensor_value ax, ay, az;
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &ax);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &ay);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &az);
            printk("ACCEL X=%d.%06d Y=%d.%06d Z=%d.%06d g\n",
                   ax.val1, ax.val2, ay.val1, ay.val2, az.val1, az.val2);
        } else {
            printk("Accel fetch error\n");
        }

        if (sensor_sample_fetch(gyro) == 0) {
            struct sensor_value gx, gy, gz;
            sensor_channel_get(gyro, SENSOR_CHAN_GYRO_X, &gx);
            sensor_channel_get(gyro, SENSOR_CHAN_GYRO_Y, &gy);
            sensor_channel_get(gyro, SENSOR_CHAN_GYRO_Z, &gz);
            printk("GYRO  X=%d.%06d Y=%d.%06d Z=%d.%06d rad/s\n",
                   gx.val1, gx.val2, gy.val1, gy.val2, gz.val1, gz.val2);
        } else {
            printk("Gyro fetch error\n");
        }

        k_msleep(10); /* Adjust: 10 ms ≈ 100 Hz effective read rate */
    }
    return 0;
}