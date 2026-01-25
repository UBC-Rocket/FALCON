#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>

#include "state_machine.h"
#include "state_machine_internal.h"
#include "state_machine_states.h"

LOG_MODULE_REGISTER(state_machine, LOG_LEVEL_INF);

#define STATE_THREAD_STACK_SIZE 2048
#define STATE_THREAD_PRIORITY   5
#define STATE_THREAD_PERIOD_MS  20

static K_THREAD_STACK_DEFINE(state_stack, STATE_THREAD_STACK_SIZE);
static struct k_thread state_thread;
static struct flight_sm state_machine;

static const struct smf_state flight_states[];
static void state_machine_reset(int64_t start_ms);

/**
 * @brief Transition the SMF context to a new state with logging.
 */
void transition_to(struct flight_sm *sm, flight_state_id_t next_state)
{
    if (next_state == sm->current_id) {
        return;
    }

    LOG_INF("State change: %s -> %s",
            flight_state_to_string(sm->current_id),
            flight_state_to_string(next_state));
    smf_set_state(SMF_CTX(sm), &flight_states[next_state]);
}

static const struct smf_state flight_states[] = {
    [FLIGHT_STATE_STANDBY] =
        SMF_CREATE_STATE(state_standby_entry, state_standby_run, NULL, NULL, NULL),
    [FLIGHT_STATE_ASCENT] =
        SMF_CREATE_STATE(state_ascent_entry, state_ascent_run, NULL, NULL, NULL),
    [FLIGHT_STATE_MACH_LOCK] =
        SMF_CREATE_STATE(state_mach_lock_entry, state_mach_lock_run, NULL, NULL, NULL),
    [FLIGHT_STATE_DROGUE_DESCENT] =
        SMF_CREATE_STATE(state_drogue_descent_entry, state_drogue_descent_run, NULL, NULL, NULL),
    [FLIGHT_STATE_MAIN_DESCENT] =
        SMF_CREATE_STATE(state_main_descent_entry, state_main_descent_run, NULL, NULL, NULL),
    [FLIGHT_STATE_LANDED] =
        SMF_CREATE_STATE(state_landed_entry, state_landed_run, NULL, NULL, NULL),
};

/**
 * @brief State machine thread loop that drives SMF with baro samples.
 */
static void state_machine_thread_fn(void *p1, void *p2, void *p3)
{
    struct baro_data baro;

    while (1) {
        get_baro_data(&baro);
        int64_t now_ms = (baro.timestamp > 0) ? baro.timestamp : k_uptime_get();

        state_machine.sample.altitude_m = baro.altitude;
        state_machine.sample.velocity_mps = baro.velocity;
        state_machine.sample.timestamp_ms = now_ms;

        smf_run_state(SMF_CTX(&state_machine));
        flight_state_id_t current = state_machine.current_id;

        struct state_data data = {
            .state = current,
            .ground_altitude = state_machine.ground_altitude_m,
            .timestamp = now_ms,
        };
        set_state_data(&data);

        k_sleep(K_MSEC(STATE_THREAD_PERIOD_MS));
    }
}

/**
 * @brief Start the state machine thread and initialize SMF.
 */
void start_state_machine_thread(void)
{
    state_machine_reset(k_uptime_get());

    k_thread_create(
        &state_thread,
        state_stack,
        K_THREAD_STACK_SIZEOF(state_stack),
        state_machine_thread_fn,
        NULL, NULL, NULL,
        STATE_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );
}

#if defined(CONFIG_ZTEST)
void state_machine_test_reset(int64_t start_ms)
{
    state_machine_reset(start_ms);
}

void state_machine_test_step(float altitude_m, float velocity_mps, int64_t timestamp_ms)
{
    state_machine.sample.altitude_m = altitude_m;
    state_machine.sample.velocity_mps = velocity_mps;
    state_machine.sample.timestamp_ms = timestamp_ms;
    smf_run_state(SMF_CTX(&state_machine));

    struct state_data data = {
        .state = state_machine.current_id,
        .ground_altitude = state_machine.ground_altitude_m,
        .timestamp = timestamp_ms,
    };
    set_state_data(&data);
}

void state_machine_test_setup_state(flight_state_id_t state, float ground_altitude_m, int64_t timestamp_ms)
{
    state_machine_reset(timestamp_ms);
    state_machine.ground_altitude_m = ground_altitude_m;
    state_machine.ground_ready = true;
    state_machine.sample.timestamp_ms = timestamp_ms;
    transition_to(&state_machine, state);
}

flight_state_id_t state_machine_test_get_state(void)
{
    return state_machine.current_id;
}

float state_machine_test_get_ground_altitude(void)
{
    return state_machine.ground_altitude_m;
}

bool state_machine_test_get_drogue_fire_triggered(void)
{
    return state_machine.drogue_fire_triggered;
}
#endif

/**
 * @brief Reset and initialize the state machine context.
 */
static void state_machine_reset(int64_t start_ms)
{
    memset(&state_machine, 0, sizeof(state_machine));
    state_machine.sample.timestamp_ms = start_ms;
    smf_set_initial(SMF_CTX(&state_machine), &flight_states[FLIGHT_STATE_STANDBY]);
}
