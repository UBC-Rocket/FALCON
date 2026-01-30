#include <zephyr/logging/log.h>
#include <zephyr/ztest.h>

#include "data.h"
#include "state_machine_config.h"
#include "state_machine_test.h"

LOG_MODULE_REGISTER(state_machine_test, LOG_LEVEL_INF);

/**
 * @brief Helper to advance through standby warmup and ground averaging.
 * @return timestamp after ground averaging is complete
 */
static int64_t complete_standby_setup(float ground_altitude)
{
    int64_t t = 0;

    // Step through warmup period - state machine won't collect samples during warmup
    for (int i = 0; i < GROUND_WARMUP_MS / 100; i++) {
        state_machine_test_step(ground_altitude, 0.0f, t);
        t += 100;
    }

    // Now collect ground samples (warmup is complete)
    for (int i = 0; i < GROUND_AVERAGE_SAMPLES; i++) {
        state_machine_test_step(ground_altitude, 0.0f, t);
        t += 100;
    }

    // Verify ground altitude is calculated
    zassert_within(state_machine_test_get_ground_altitude(), ground_altitude, 0.001f,
                   "ground altitude should match average");
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_STANDBY,
                  "should still be in standby after ground averaging");

    return t;
}

/**
 * @brief Transition from standby to ascent state.
 * Assumes state machine is already in STANDBY state with ground altitude set.
 * @param ground_altitude Ground altitude for relative altitude calculations
 * @param start_t Starting timestamp for the transition
 * @return timestamp after entering ascent
 */
static int64_t transition_to_ascent(float ground_altitude, int64_t start_t)
{
    int64_t t = start_t;

    // Transition to ascent: need relative altitude > threshold AND velocity > threshold
    float ascent_alt = ground_altitude + ASCENT_ALTITUDE_THRESHOLD_M + 1.0f;
    float ascent_vel = ASCENT_VELOCITY_THRESHOLD_MPS + 1.0f;

    for (int i = 0; i < ASCENT_CHECKS; i++) {
        state_machine_test_step(ascent_alt, ascent_vel, t);
        t += 100;
    }

    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_ASCENT,
                  "expected ascent after ascent checks");

    return t;
}

/**
 * @brief Transition from ascent to drogue descent state.
 * Assumes state machine is already in ASCENT state.
 * @param ground_altitude Ground altitude for relative altitude calculations
 * @param start_t Starting timestamp for the transition
 * @return timestamp after entering drogue descent
 */
static int64_t transition_to_drogue_descent(float ground_altitude, int64_t start_t)
{
    int64_t t = start_t;

    // Transition to drogue descent: velocity must be below threshold
    float current_alt = ground_altitude + ASCENT_ALTITUDE_THRESHOLD_M + 1.0f;
    float drogue_vel = DROGUE_DEPLOY_VELOCITY_THRESHOLD_MPS - 1.0f;

    for (int i = 0; i < DROGUE_DEPLOY_CHECKS; i++) {
        state_machine_test_step(current_alt, drogue_vel, t);
        t += 100;
    }

    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_DROGUE_DESCENT,
                  "expected drogue descent after drogue checks");
    zassert_false(state_machine_test_get_drogue_fire_triggered(),
                  "drogue should not fire immediately upon entry");

    return t;
}

/**
 * @brief Transition from ascent to mach lock state.
 * Assumes state machine is already in ASCENT state.
 * @param ground_altitude Ground altitude for relative altitude calculations
 * @param start_t Starting timestamp for the transition
 * @return timestamp after entering mach lock
 */
static int64_t transition_to_mach_lock(float ground_altitude, int64_t start_t)
{
    int64_t t = start_t;
    float current_alt = ground_altitude + ASCENT_ALTITUDE_THRESHOLD_M + 1.0f;

    // Enter mach lock
    float mach_vel = MACH_LOCK_VELOCITY_THRESHOLD_MPS + 1.0f;
    for (int i = 0; i < MACH_LOCK_CHECKS; i++) {
        state_machine_test_step(current_alt, mach_vel, t);
        t += 100;
    }
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_MACH_LOCK,
                  "expected mach lock after high velocity");

    return t;
}

/**
 * @brief Transition from mach lock back to ascent state.
 * Assumes state machine is already in MACH_LOCK state.
 * @param ground_altitude Ground altitude for relative altitude calculations
 * @param start_t Starting timestamp for the transition
 * @return timestamp after exiting mach lock back to ascent
 */
static int64_t transition_from_mach_lock(float ground_altitude, int64_t start_t)
{
    int64_t t = start_t;
    float current_alt = ground_altitude + ASCENT_ALTITUDE_THRESHOLD_M + 1.0f;

    // Exit mach lock
    float unlock_vel = MACH_UNLOCK_VELOCITY_THRESHOLD_MPS - 1.0f;
    for (int i = 0; i < MACH_UNLOCK_CHECKS; i++) {
        state_machine_test_step(current_alt, unlock_vel, t);
        t += 100;
    }
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_ASCENT,
                  "expected ascent after mach unlock");

    return t;
}

/**
 * @brief Transition from drogue descent to main descent state.
 * Assumes state machine is already in DROGUE_DESCENT state.
 * @param ground_altitude Ground altitude for relative altitude calculations
 * @param start_t Starting timestamp (should be drogue entry time)
 * @return timestamp after entering main descent
 */
static int64_t transition_to_main_descent(float ground_altitude, int64_t start_t)
{
    int64_t t = start_t;
    // Transition to main descent: relative altitude must be below threshold
    float main_alt = ground_altitude + MAIN_DEPLOY_ALTITUDE_M - 1.0f;
    for (int i = 0; i < MAIN_DEPLOY_CHECKS; i++) {
        state_machine_test_step(main_alt, 0.0f, t);
        t += 100;
    }

    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_MAIN_DESCENT,
                  "expected main descent after main deploy checks");

    return t;
}

/**
 * @brief Transition from main descent to landed state.
 * Assumes state machine is already in MAIN_DESCENT state.
 * @param ground_altitude Ground altitude for relative altitude calculations
 * @param start_t Starting timestamp (should be main descent entry time)
 * @return timestamp after entering landed state
 */
static int64_t transition_to_landed(float ground_altitude, int64_t start_t)
{
    int64_t t = start_t;
    int64_t main_entry_time = start_t;

    // The main descent entry sets last_landed_check_ms to entry time
    // We need to wait for the first interval, then do LANDED_CHECKS spaced checks
    float slow_vel = LANDED_VELOCITY_THRESHOLD_MPS - 1.0f;
    float near_ground_alt = ground_altitude + 1.0f;

    // First check happens after interval
    t = main_entry_time + LANDED_CHECK_INTERVAL_MS;
    state_machine_test_step(near_ground_alt, slow_vel, t);

    // Remaining checks, each spaced by interval
    for (int i = 1; i < LANDED_CHECKS; i++) {
        t += LANDED_CHECK_INTERVAL_MS;
        state_machine_test_step(near_ground_alt, slow_vel, t);
    }

    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_LANDED,
                  "expected landed after spaced checks");

    return t;
}

ZTEST(state_machine, test_standby_ground_averaging)
{
    float ground_altitude = 100.0f;

    state_machine_test_reset(0);
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_STANDBY, "should start in standby");

    complete_standby_setup(ground_altitude);

    // Verify shared state data
    struct state_data shared = {0};
    get_state_data(&shared);
    zassert_equal(shared.state, FLIGHT_STATE_STANDBY, "shared state should match state machine");
    zassert_within(shared.ground_altitude, ground_altitude, 0.001f,
                   "shared ground altitude should match average");
}

ZTEST(state_machine, test_standby_to_ascent)
{
    float ground_altitude = 100.0f;
    int64_t t = 0;

    state_machine_test_reset(0);
    t = complete_standby_setup(ground_altitude);
    transition_to_ascent(ground_altitude, t);

    // Verify shared state data
    struct state_data shared = {0};
    get_state_data(&shared);
    zassert_equal(shared.state, FLIGHT_STATE_ASCENT, "shared state should match state machine");
    zassert_within(shared.ground_altitude, ground_altitude, 0.001f,
                   "shared ground altitude should match average");
}

ZTEST(state_machine, test_mach_lock)
{
    float ground_altitude = 100.0f;
    int64_t t = 0;

    state_machine_test_setup_state(FLIGHT_STATE_ASCENT, ground_altitude, t);
    t = transition_to_mach_lock(ground_altitude, t);
    transition_from_mach_lock(ground_altitude, t);
}

ZTEST(state_machine, test_mach_lock_blocks_drogue)
{
    float ground_altitude = 100.0f;
    int64_t t = 0;

    state_machine_test_setup_state(FLIGHT_STATE_ASCENT, ground_altitude, t);
    t = transition_to_mach_lock(ground_altitude, t);

    // Verify we're in mach lock
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_MACH_LOCK, "should be in mach lock");

    // Try to trigger drogue descent with low velocity
    // This should NOT work - must exit mach lock first
    float current_alt = ground_altitude + ASCENT_ALTITUDE_THRESHOLD_M + 1.0f;
    float drogue_vel = DROGUE_DEPLOY_VELOCITY_THRESHOLD_MPS - 1.0f;

    for (int i = 0; i < DROGUE_DEPLOY_CHECKS; i++) {
        state_machine_test_step(current_alt, drogue_vel, t);
        t += 100;
    }

    // Should still be in mach lock, NOT drogue descent
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_MACH_LOCK,
                  "should remain in mach lock, cannot transition to drogue");
}

ZTEST(state_machine, test_ascent_to_drogue_descent)
{
    float ground_altitude = 100.0f;
    int64_t t = 0;

    state_machine_test_setup_state(FLIGHT_STATE_ASCENT, ground_altitude, t);
    transition_to_drogue_descent(ground_altitude, t);
}

ZTEST(state_machine, test_drogue_delay)
{
    float ground_altitude = 100.0f;
    int64_t t = 0;
    int64_t drogue_entry_time = t;

    state_machine_test_setup_state(FLIGHT_STATE_DROGUE_DESCENT, ground_altitude, drogue_entry_time);

    // Before delay, drogue should not fire
    t = drogue_entry_time + (DROGUE_DEPLOY_DELAY_MS / 2);
    state_machine_test_step(ground_altitude + 1.0f, 0.0f, t);
    zassert_false(state_machine_test_get_drogue_fire_triggered(),
                  "drogue should not fire before delay");

    // After delay, drogue should fire
    t = drogue_entry_time + DROGUE_DEPLOY_DELAY_MS;
    state_machine_test_step(ground_altitude + 1.0f, 0.0f, t);
    zassert_true(state_machine_test_get_drogue_fire_triggered(), "drogue should fire after delay");
}

ZTEST(state_machine, test_drogue_to_main_descent)
{
    float ground_altitude = 100.0f;
    int64_t t = 0;

    state_machine_test_setup_state(FLIGHT_STATE_DROGUE_DESCENT, ground_altitude, t);
    transition_to_main_descent(ground_altitude, t);

    // Verify we're in main descent
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_MAIN_DESCENT,
                  "expected main descent after main deploy checks");
}

ZTEST(state_machine, test_main_to_landed)
{
    float ground_altitude = 100.0f;
    int64_t t = 0;

    state_machine_test_setup_state(FLIGHT_STATE_MAIN_DESCENT, ground_altitude, t);
    transition_to_landed(ground_altitude, t);
}

ZTEST(state_machine, test_full_flight_sequence)
{
    float ground_altitude = 100.0f;
    int64_t t = 0;

    state_machine_test_reset(t);

    // Standby: ground averaging
    t = complete_standby_setup(ground_altitude);
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_STANDBY, "standby");

    // Standby -> Ascent
    t = transition_to_ascent(ground_altitude, t);
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_ASCENT, "ascent");

    // Ascent -> Mach Lock
    t = transition_to_mach_lock(ground_altitude, t);
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_MACH_LOCK, "mach lock");

    // Mach Lock -> Ascent
    t = transition_from_mach_lock(ground_altitude, t);
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_ASCENT, "ascent after mach");

    // Ascent -> Drogue Descent
    t = transition_to_drogue_descent(ground_altitude, t);
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_DROGUE_DESCENT, "drogue");

    // Drogue Descent -> Main Descent
    t = transition_to_main_descent(ground_altitude, t);
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_MAIN_DESCENT, "main");

    // Main Descent -> Landed
    t = transition_to_landed(ground_altitude, t);
    zassert_equal(state_machine_test_get_state(), FLIGHT_STATE_LANDED, "landed");
}

ZTEST_SUITE(state_machine, NULL, NULL, NULL, NULL, NULL);
