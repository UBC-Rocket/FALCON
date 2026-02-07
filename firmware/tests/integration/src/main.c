#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/logging/log.h>

#include "state_machine/state_machine.h"
#include "state_machine/state_machine_internal.h"
#include "state_machine/state_machine_config.h"
#include "pyro/pyro_thread.h"
#include "data.h"

LOG_MODULE_REGISTER(integration_test, LOG_LEVEL_INF);

/* Fake barometer data generator */
static struct {
    float altitude_m;
    float velocity_mps;
    int64_t timestamp_ms;
    flight_state_id_t target_state;
} test_scenario;

static void inject_test_data(float altitude, float velocity, bool log)
{
    test_scenario.altitude_m = altitude;
    test_scenario.velocity_mps = velocity;
    test_scenario.timestamp_ms = k_uptime_get();

    /* Update the global baro data that state machine reads */
    struct baro_data baro_update = {
        .baro0 = {
            .altitude = altitude,
            .pressure = 101325.0f,  // Dummy pressure
            .temperature = 20.0f,   // Dummy temperature
            .nis = 0.0f,
            .faults = 0,
            .healthy = true,
        },
        .baro1 = {
            .altitude = altitude,
            .pressure = 101325.0f,
            .temperature = 20.0f,
            .nis = 0.0f,
            .faults = 0,
            .healthy = true,
        },
        .altitude = altitude,
        .velocity = velocity,
        .timestamp = test_scenario.timestamp_ms,
        .alt_variance = 1.0f,  // Small variance for confident estimates
        .vel_variance = 1.0f,
    };
    set_baro_data(&baro_update);
    
    if (log) {
        LOG_INF("Injected: alt=%.2f m, vel=%.2f m/s", altitude, velocity);
    }
}

/* Continuously inject test data with live timestamps for a duration */
static void inject_and_wait(float altitude, float velocity, int duration_ms)
{
    int64_t end_time = k_uptime_get() + duration_ms;
    bool logged_once = false;
    
    while (k_uptime_get() < end_time) {
        inject_test_data(altitude, velocity, false);
        
        if (!logged_once) {
            LOG_INF("Continuously injecting: alt=%.2f m, vel=%.2f m/s for %d ms", 
                    altitude, velocity, duration_ms);
            logged_once = true;
        }
        
        k_sleep(K_MSEC(50));  /* Update at 20Hz to keep timestamp fresh */
    }
}

ZTEST(integration, test_full_flight_sequence)
{
    bool test_failed = false;
    pyro_status_t pyro_status;
    
    LOG_INF("========================================");
    LOG_INF("Starting Full Flight Integration Test");
    LOG_INF("========================================");

    /* Start threads */
    LOG_INF("\nStarting state machine and pyro threads...");
    start_state_machine_thread();
    start_pyro_thread();
    
    /* Wait for initialization */
    k_sleep(K_SECONDS(2));

    // Standby
    LOG_INF("\n=== PHASE 1: STANDBY ===");
    inject_test_data(0.0, 0.0, true);
    k_sleep(K_SECONDS(5));

    // Ascent
    LOG_INF("\n=== PHASE 2: ASCENT ===");
    LOG_INF("Injecting altitude > %.2f m and velocity > %.2f m/s", 
            ASCENT_ALTITUDE_THRESHOLD_M, ASCENT_VELOCITY_THRESHOLD_MPS);
    inject_test_data(ASCENT_ALTITUDE_THRESHOLD_M + 1.0, 
                     ASCENT_VELOCITY_THRESHOLD_MPS + 1.0, true);
    k_sleep(K_SECONDS(5));

    // Mach lock
    LOG_INF("\n=== PHASE 3: MACH LOCK ===");
    LOG_INF("Injecting velocity > %.2f m/s", MACH_LOCK_VELOCITY_THRESHOLD_MPS);
    inject_test_data(500.0, MACH_LOCK_VELOCITY_THRESHOLD_MPS + 1.0, true);
    k_sleep(K_SECONDS(5));

    // Mach unlock
    LOG_INF("\n=== PHASE 4: MACH UNLOCK ===");
    LOG_INF("Injecting velocity < %.2f m/s", MACH_LOCK_VELOCITY_THRESHOLD_MPS);
    inject_test_data(1000.0, MACH_LOCK_VELOCITY_THRESHOLD_MPS - 1.0, true);
    k_sleep(K_SECONDS(5));

    // Drogue descent
    LOG_INF("\n=== PHASE 5: DROGUE DESCENT ===");
    LOG_INF("Injecting velocity < %.2f m/s", DROGUE_DEPLOY_VELOCITY_THRESHOLD_MPS);
    LOG_INF("Drogue should fire %d ms after state entry...", DROGUE_DEPLOY_DELAY_MS);
    inject_and_wait(1500.0, DROGUE_DEPLOY_VELOCITY_THRESHOLD_MPS - 1.0, 
                    DROGUE_DEPLOY_DELAY_MS + 2000);  /* 3s delay + 2s buffer */

    /* Check drogue pyro status */
    k_sleep(K_SECONDS(2));  /* Give more time for pyro to react and send acknowledgment */
    pyro_get_status(&pyro_status);
    if (pyro_status.drogue_fired && !pyro_status.drogue_fail && pyro_status.drogue_fire_ack) {
        LOG_INF("Drogue fired successfully with ACK");
    } else {
        if (pyro_status.drogue_fail) {
            LOG_ERR("Drogue fire FAILED");
        }
        if (!pyro_status.drogue_fired) {
            LOG_ERR("Drogue did not fire");
        }
        if (!pyro_status.drogue_fire_ack) {
            LOG_ERR("Drogue no ACK received");
        }
        test_failed = true;
    }

    // Main descent
    LOG_INF("\n=== PHASE 6: MAIN DESCENT ===");
    LOG_INF("Injecting altitude < %.2f m AGL", MAIN_DEPLOY_ALTITUDE_M);
    LOG_INF("Main should fire immediately...");
    inject_test_data(MAIN_DEPLOY_ALTITUDE_M - 1.0, -8.0, true);
    k_sleep(K_SECONDS(5));
    
    /* Check main pyro status */
    k_sleep(K_SECONDS(2));  /* Give more time for pyro to react and send acknowledgment */
    pyro_get_status(&pyro_status);
    if (pyro_status.main_fired && !pyro_status.main_fail && pyro_status.main_fire_ack) {
        LOG_INF("Main fired successfully with ACK");
    } else {
        if (pyro_status.main_fail) {
            LOG_ERR("Main fire FAILED");
        } else if (!pyro_status.main_fired) {
            LOG_ERR("Main did not fire");
        } else if (!pyro_status.main_fire_ack) {
            LOG_ERR("Main fired but no ACK received");
        }
        test_failed = true;
    }

    // Landed
    LOG_INF("\n=== PHASE 7: LANDED ===");
    LOG_INF("Injecting velocity < %.2f m/s", LANDED_VELOCITY_THRESHOLD_MPS);
    
    inject_and_wait(3.0, LANDED_VELOCITY_THRESHOLD_MPS - 1.0, 
                    LANDED_CHECKS * LANDED_CHECK_INTERVAL_MS + 1.0);
    
    k_sleep(K_SECONDS(1));
    
    /* Final test result */
    LOG_INF("\n========================================");
    if (test_failed) {
        LOG_ERR("INTEGRATION TEST FAILED");
        LOG_ERR("One or more pyro commands did not execute successfully");
        LOG_INF("========================================");
        zassert_false(test_failed, "Pyro command(s) failed during flight sequence");
    } else {
        LOG_INF("✓ INTEGRATION TEST PASSED");
        LOG_INF("All pyro commands executed successfully");
        LOG_INF("========================================");
    }
}

ZTEST_SUITE(integration, NULL, NULL, NULL, NULL, NULL);
