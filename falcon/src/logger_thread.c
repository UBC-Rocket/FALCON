#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include "data.h"
#include "log_format.h"

LOG_MODULE_REGISTER(logger_thread, LOG_LEVEL_INF);

#define LOGGER_THREAD_STACK_SIZE 2048
#define LOGGER_THREAD_PRIORITY   7
#define LOGGER_THREAD_PERIOD_MS  1000

K_THREAD_STACK_DEFINE(logger_stack, LOGGER_THREAD_STACK_SIZE);
static struct k_thread logger_thread;

static void logger_thread_fn(void *p1, void *p2, void *p3)
{
    struct log_frame frame;

    while (1) {
        k_sleep(K_MSEC(LOGGER_THREAD_PERIOD_MS));

        frame.log_timestamp = k_uptime_get();

        get_imu_data(&frame.imu);
        get_baro_data(&frame.baro);

        //temporary logging output
        LOG_INF("Time=%lld Accel[0]=%.3f",
        frame.log_timestamp,
        (double)frame.imu.accel[0]);
    }
}

void start_logger_thread()
{
    k_thread_create(
        &logger_thread,
        logger_stack,
        K_THREAD_STACK_SIZEOF(logger_stack),
        logger_thread_fn,
        NULL, NULL, NULL,
        LOGGER_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );
}
