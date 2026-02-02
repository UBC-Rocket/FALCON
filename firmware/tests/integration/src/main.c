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
    LOG_INF("========================================");
    LOG_INF("Starting Full Flight Integration Test");
    LOG_INF("========================================");
    LOG_INF("Using constants from state_machine_config.h:");
    LOG_INF("  ASCENT_ALTITUDE_THRESHOLD_M = %.2f", ASCENT_ALTITUDE_THRESHOLD_M);
    LOG_INF("  ASCENT_VELOCITY_THRESHOLD_MPS = %.2f", ASCENT_VELOCITY_THRESHOLD_MPS);
    LOG_INF("  MACH_LOCK_VELOCITY_THRESHOLD_MPS = %.2f", MACH_LOCK_VELOCITY_THRESHOLD_MPS);
    LOG_INF("  DROGUE_DEPLOY_VELOCITY_THRESHOLD_MPS = %.2f", DROGUE_DEPLOY_VELOCITY_THRESHOLD_MPS);
    LOG_INF("  DROGUE_DEPLOY_DELAY_MS = %d", DROGUE_DEPLOY_DELAY_MS);
    LOG_INF("  MAIN_DEPLOY_ALTITUDE_M = %.2f", MAIN_DEPLOY_ALTITUDE_M);
    LOG_INF("  LANDED_VELOCITY_THRESHOLD_MPS = %.2f", LANDED_VELOCITY_THRESHOLD_MPS);

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



    // Main descent
    LOG_INF("\n=== PHASE 6: MAIN DESCENT ===");
    LOG_INF("Injecting altitude < %.2f m AGL", MAIN_DEPLOY_ALTITUDE_M);
    LOG_INF("Main should fire immediately...");
    inject_test_data(MAIN_DEPLOY_ALTITUDE_M - 1.0, -8.0, true);
    k_sleep(K_SECONDS(5));

    // Landed
    LOG_INF("\n=== PHASE 7: LANDED ===");
    LOG_INF("Injecting velocity < %.2f m/s", LANDED_VELOCITY_THRESHOLD_MPS);
    
    inject_and_wait(3.0, LANDED_VELOCITY_THRESHOLD_MPS - 1.0, 
                    LANDED_CHECKS * LANDED_CHECK_INTERVAL_MS + 1.0);
    
    k_sleep(K_SECONDS(1));
}

ZTEST_SUITE(integration, NULL, NULL, NULL, NULL, NULL);
