#include <math.h>

#include "state_machine_internal.h"
#include "state_machine_states.h"

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
 * @brief SMF entry handler for main descent state.
 */
void state_main_descent_entry(void *obj)
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
enum smf_state_result state_main_descent_run(void *obj)
{
    struct flight_sm *sm = obj;

    transition_to(sm, update_main_descent(sm, &sm->sample));
    return SMF_EVENT_HANDLED;
}
