#include "data.h"
#include "logger_thread.h"
#include "sensors/baro_thread.h"
#include "sensors/imu_thread.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(falcon_main, LOG_LEVEL_INF);

int main(void) {
  LOG_INF("Falcon application started");

  // Start the threads
  start_imu_thread();
  start_logger_thread();
  start_baro_thread();

  return 0;
}
