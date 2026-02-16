#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "sensors/imu_thread.h"
#include "sensors/baro_thread.h"
#include "logger_thread.h"
#include "state_machine/state_machine.h"
#include "pyro/pyro_thread.h"
#include "radio/radio_thread.h"
#include "gps/gps_thread.h"
#include "data.h"

LOG_MODULE_REGISTER(falcon_main, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("Falcon application started");

    // Start the threads
    start_imu_thread();
    start_logger_thread();
    start_baro_thread();
    start_pyro_thread();
    start_state_machine_thread();
    start_radio_thread();
    start_gps_thread();

    /* Simple timed pyro sequence for manual/full-test runs:
     * - fire drogue after 30 seconds
     * - fire main after 60 seconds (30s after drogue)
     */
    LOG_INF("Waiting 30 seconds to fire drogue...");
    k_sleep(K_SECONDS(30));
    int rc = pyro_fire_drogue();
    LOG_INF("pyro_fire_drogue() returned %d", rc);

    /* Wait 5 seconds and check pyro status for drogue */
    LOG_INF("Waiting 5 seconds to check drogue status...");
    k_sleep(K_SECONDS(5));
    struct pyro_data pyro_status;
    get_pyro_data(&pyro_status);
    LOG_INF("Drogue status -- ACK: %d, Fired: %d, Fail: %d",
            pyro_status.drogue_fire_ack ? 1 : 0,
            pyro_status.drogue_fired ? 1 : 0,
            pyro_status.drogue_fail ? 1 : 0);

    LOG_INF("Waiting additional 30 seconds to fire main...");
    k_sleep(K_SECONDS(30));
    rc = pyro_fire_main();
    LOG_INF("pyro_fire_main() returned %d", rc);

    /* Wait 5 seconds and check pyro status for main */
    LOG_INF("Waiting 5 seconds to check main status...");
    k_sleep(K_SECONDS(5));
    get_pyro_data(&pyro_status);
    LOG_INF("Main status  -- ACK: %d, Fired: %d, Fail: %d",
            pyro_status.main_fire_ack ? 1 : 0,
            pyro_status.main_fired ? 1 : 0,
            pyro_status.main_fail ? 1 : 0);

    /* Keep main thread alive */
    while (1) {
        k_sleep(K_SECONDS(60));
    }
}
