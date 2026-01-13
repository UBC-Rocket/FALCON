#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>

#include "state_machine.h"

LOG_MODULE_REGISTER(state_machine, LOG_LEVEL_INF);

#define STATE_THREAD_STACK_SIZE 2048
#define STATE_THREAD_PRIORITY   5
#define STATE_THREAD_PERIOD_MS  20

// Standby baseline
#define GROUND_AVERAGE_SAMPLES             10

// Ascent detection
#define ASCENT_ALTITUDE_THRESHOLD_M        25.0f
#define ASCENT_VELOCITY_THRESHOLD_MPS      5.0f
#define ASCENT_CHECKS                      5

// Mach lock
#define MACH_LOCK_VELOCITY_THRESHOLD_MPS   150.0f
#define MACH_LOCK_CHECKS                   10
#define MACH_UNLOCK_VELOCITY_THRESHOLD_MPS 150.0f
#define MACH_UNLOCK_CHECKS                 10

// Drogue deployment
#define DROGUE_DEPLOY_VELOCITY_THRESHOLD_MPS  5.0f
#define DROGUE_DEPLOY_CHECKS                 5
#define DROGUE_DEPLOY_DELAY_MS               3000

// Main deployment
#define MAIN_DEPLOY_ALTITUDE_M             488.0f
#define MAIN_DEPLOY_CHECKS                 5

// Landing detection
#define LANDED_VELOCITY_THRESHOLD_MPS      4.0f
#define LANDED_CHECKS                      6
#define LANDED_CHECK_INTERVAL_MS           10000

typedef struct
{
    uint8_t count;
} repeated_check_t;

typedef struct
{
    float altitude_m;
    float velocity_mps;
    int64_t timestamp_ms;
} state_sample_t;

struct flight_sm
{
    struct smf_ctx ctx;
    flight_state_id_t current_id;
    int64_t entry_time_ms;
    state_sample_t sample;
    float ground_altitude_m;
    float ground_sum_m;
    uint8_t ground_samples;
    bool ground_ready;
    repeated_check_t standby_check;
    repeated_check_t mach_lock_check;
    repeated_check_t mach_unlock_check;
    repeated_check_t drogue_main_check;
    repeated_check_t landed_check;
    int64_t last_landed_check_ms;
    bool drogue_fired;
};

static K_THREAD_STACK_DEFINE(state_stack, STATE_THREAD_STACK_SIZE);
static struct k_thread state_thread;
static struct flight_sm state_machine;

static const struct smf_state flight_states[];

/**
 * @brief Update a repeated-check counter and report when it reaches the threshold.
 */
static bool repeated_check_update(repeated_check_t *check, bool condition, uint8_t required)
{
    if (condition) {
        if (check->count < 255) {
            check->count++;
        }
    } else {
        check->count = 0;
    }

    return check->count >= required;
}

/**
 * @brief Clear a repeated-check counter.
 */
static void reset_repeated_check(repeated_check_t *check)
{
    check->count = 0;
}

/**
 * @brief Reset ground altitude averaging state.
 */
static void reset_ground_average(struct flight_sm *sm)
{
    sm->ground_altitude_m = 0.0f;
    sm->ground_sum_m = 0.0f;
    sm->ground_samples = 0;
    sm->ground_ready = false;
}

/**
 * @brief Convert an absolute altitude to altitude relative to ground baseline.
 */
static float get_relative_altitude(const struct flight_sm *sm, float altitude_m)
{
    return altitude_m - sm->ground_altitude_m;
}

/**
 * @brief Trigger drogue deployment action.
 */
static void state_action_fire_drogue(void)
{
    LOG_INF("Drogue deployment triggered");
    // TODO: Signal drogue deployment here
}

/**
 * @brief Trigger main parachute deployment action.
 */
static void state_action_fire_main(void)
{
    LOG_INF("Main deployment triggered");
    // TODO: Signal main deployment here
}

/**
 * @brief Trigger landed action.
 */
static void state_action_landed(void)
{
    LOG_INF("The rocket has landed");
    // TODO: Need to do anything upon landing?
}

/**
 * @brief Return a human-readable string for a flight state.
 */
const char *flight_state_to_string(flight_state_id_t state)
{
    switch (state) {
    case FLIGHT_STATE_STANDBY:
        return "STANDBY";
    case FLIGHT_STATE_ASCENT:
        return "ASCENT";
    case FLIGHT_STATE_MACH_LOCK:
        return "MACH_LOCK";
    case FLIGHT_STATE_DROGUE_DESCENT:
        return "DROGUE_DESCENT";
    case FLIGHT_STATE_MAIN_DESCENT:
        return "MAIN_DESCENT";
    case FLIGHT_STATE_LANDED:
        return "LANDED";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief Common entry bookkeeping shared by all states.
 */
static void state_entry_common(struct flight_sm *sm, flight_state_id_t state)
{
    sm->current_id = state;
    sm->entry_time_ms = sm->sample.timestamp_ms;
}

/**
 * @brief Evaluate transitions while in standby (includes ground averaging).
 */
static flight_state_id_t update_standby(struct flight_sm *sm, const state_sample_t *sample)
{
    if (!sm->ground_ready) {
        sm->ground_sum_m += sample->altitude_m;
        sm->ground_samples++;
        if (sm->ground_samples >= GROUND_AVERAGE_SAMPLES) {
            sm->ground_altitude_m = sm->ground_sum_m / (float)sm->ground_samples;
            sm->ground_ready = true;
        }
        return FLIGHT_STATE_STANDBY;
    }

    float rel_altitude = get_relative_altitude(sm, sample->altitude_m);
    bool ascent_condition = (rel_altitude > ASCENT_ALTITUDE_THRESHOLD_M) &&
                            (sample->velocity_mps > ASCENT_VELOCITY_THRESHOLD_MPS);

    if (repeated_check_update(&sm->standby_check, ascent_condition, ASCENT_CHECKS)) {
        return FLIGHT_STATE_ASCENT;
    }

    return FLIGHT_STATE_STANDBY;
}

/**
 * @brief Evaluate transitions while in ascent.
 */
static flight_state_id_t update_ascent(struct flight_sm *sm, const state_sample_t *sample)
{
    bool mach_lock = sample->velocity_mps > MACH_LOCK_VELOCITY_THRESHOLD_MPS;

    if (repeated_check_update(&sm->mach_lock_check, mach_lock, MACH_LOCK_CHECKS)) {
        return FLIGHT_STATE_MACH_LOCK;
    }

    bool drogue = sample->velocity_mps < DROGUE_DEPLOY_VELOCITY_THRESHOLD_MPS;
    if (repeated_check_update(&sm->drogue_main_check, drogue, DROGUE_DEPLOY_CHECKS)) {
        return FLIGHT_STATE_DROGUE_DESCENT;
    }

    return FLIGHT_STATE_ASCENT;
}

/**
 * @brief Evaluate transitions while in mach lock.
 */
static flight_state_id_t update_mach_lock(struct flight_sm *sm, const state_sample_t *sample)
{
    bool below_unlock = sample->velocity_mps < MACH_UNLOCK_VELOCITY_THRESHOLD_MPS;

    if (repeated_check_update(&sm->mach_unlock_check, below_unlock, MACH_UNLOCK_CHECKS)) {
        return FLIGHT_STATE_ASCENT;
    }

    return FLIGHT_STATE_MACH_LOCK;
}

/**
 * @brief Evaluate transitions while in drogue descent.
 */
static flight_state_id_t update_drogue_descent(struct flight_sm *sm, const state_sample_t *sample)
{
    if (!sm->drogue_fired) {
        return FLIGHT_STATE_DROGUE_DESCENT;
    }

    float rel_altitude = get_relative_altitude(sm, sample->altitude_m);
    bool below_main_alt = rel_altitude < MAIN_DEPLOY_ALTITUDE_M;

    if (repeated_check_update(&sm->drogue_main_check, below_main_alt, MAIN_DEPLOY_CHECKS)) {
        return FLIGHT_STATE_MAIN_DESCENT;
    }

    return FLIGHT_STATE_DROGUE_DESCENT;
}

/**
 * @brief Evaluate transitions while in main descent.
 */
static flight_state_id_t update_main_descent(struct flight_sm *sm, const state_sample_t *sample)
{
    bool landed = fabsf(sample->velocity_mps) < LANDED_VELOCITY_THRESHOLD_MPS;

    if (landed) {
        int64_t elapsed = sample->timestamp_ms - sm->last_landed_check_ms;
        if (elapsed >= LANDED_CHECK_INTERVAL_MS) {
            sm->last_landed_check_ms = sample->timestamp_ms;
            if (repeated_check_update(&sm->landed_check, true, LANDED_CHECKS)) {
                return FLIGHT_STATE_LANDED;
            }
        }
    } else {
        repeated_check_update(&sm->landed_check, false, LANDED_CHECKS);
        sm->last_landed_check_ms = sample->timestamp_ms;
    }

    return FLIGHT_STATE_MAIN_DESCENT;
}

/**
 * @brief Maintain the terminal landed state.
 */
static flight_state_id_t update_landed(void)
{
    return FLIGHT_STATE_LANDED;
}

/**
 * @brief Transition the SMF context to a new state with logging.
 */
static void transition_to(struct flight_sm *sm, flight_state_id_t next_state)
{
    if (next_state == sm->current_id) {
        return;
    }

    LOG_INF("State change: %s -> %s",
            flight_state_to_string(sm->current_id),
            flight_state_to_string(next_state));
    smf_set_state(SMF_CTX(sm), &flight_states[next_state]);
}

/**
 * @brief SMF entry handler for standby state.
 */
static void state_standby_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_STANDBY);
    reset_repeated_check(&sm->standby_check);
    reset_ground_average(sm);
}

/**
 * @brief SMF run handler for standby state.
 */
static enum smf_state_result state_standby_run(void *obj)
{
    struct flight_sm *sm = obj;

    transition_to(sm, update_standby(sm, &sm->sample));
    return SMF_EVENT_HANDLED;
}

/**
 * @brief SMF entry handler for ascent state.
 */
static void state_ascent_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_ASCENT);
    reset_repeated_check(&sm->mach_lock_check);
    reset_repeated_check(&sm->drogue_main_check);
}

/**
 * @brief SMF run handler for ascent state.
 */
static enum smf_state_result state_ascent_run(void *obj)
{
    struct flight_sm *sm = obj;

    transition_to(sm, update_ascent(sm, &sm->sample));
    return SMF_EVENT_HANDLED;
}

/**
 * @brief SMF entry handler for mach lock state.
 */
static void state_mach_lock_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_MACH_LOCK);
    reset_repeated_check(&sm->mach_unlock_check);
}

/**
 * @brief SMF run handler for mach lock state.
 */
static enum smf_state_result state_mach_lock_run(void *obj)
{
    struct flight_sm *sm = obj;

    transition_to(sm, update_mach_lock(sm, &sm->sample));
    return SMF_EVENT_HANDLED;
}

/**
 * @brief SMF entry handler for drogue descent state.
 */
static void state_drogue_descent_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_DROGUE_DESCENT);
    reset_repeated_check(&sm->drogue_main_check);
    sm->drogue_fired = false;
}

/**
 * @brief SMF run handler for drogue descent state.
 */
static enum smf_state_result state_drogue_descent_run(void *obj)
{
    struct flight_sm *sm = obj;

    if (!sm->drogue_fired &&
        (sm->sample.timestamp_ms - sm->entry_time_ms) >= DROGUE_DEPLOY_DELAY_MS) {
        state_action_fire_drogue();
        sm->drogue_fired = true;
    }

    transition_to(sm, update_drogue_descent(sm, &sm->sample));
    return SMF_EVENT_HANDLED;
}

/**
 * @brief SMF entry handler for main descent state.
 */
static void state_main_descent_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_MAIN_DESCENT);
    reset_repeated_check(&sm->landed_check);
    sm->last_landed_check_ms = sm->sample.timestamp_ms;
    state_action_fire_main();
}

/**
 * @brief SMF run handler for main descent state.
 */
static enum smf_state_result state_main_descent_run(void *obj)
{
    struct flight_sm *sm = obj;

    transition_to(sm, update_main_descent(sm, &sm->sample));
    return SMF_EVENT_HANDLED;
}

/**
 * @brief SMF entry handler for landed state.
 */
static void state_landed_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_LANDED);
    state_action_landed();
}

/**
 * @brief SMF run handler for landed state.
 */
static enum smf_state_result state_landed_run(void *obj)
{
    struct flight_sm *sm = obj;

    transition_to(sm, update_landed());
    return SMF_EVENT_HANDLED;
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
    state_machine.sample.timestamp_ms = k_uptime_get();
    smf_set_initial(SMF_CTX(&state_machine), &flight_states[FLIGHT_STATE_STANDBY]);

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
