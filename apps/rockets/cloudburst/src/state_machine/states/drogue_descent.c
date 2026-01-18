#include "state_machine_internal.h"
#include "state_machine_states.h"

/**
 * @brief Evaluate transitions while in drogue descent.
 */
static flight_state_id_t update_drogue_descent(struct flight_sm *sm, const state_sample_t *sample)
{
    float rel_altitude = get_relative_altitude(sm, sample->altitude_m);
    bool below_main_alt = rel_altitude < MAIN_DEPLOY_ALTITUDE_M;

    if (repeated_check_update(&sm->drogue_main_check, below_main_alt, MAIN_DEPLOY_CHECKS)) {
        return FLIGHT_STATE_MAIN_DESCENT;
    }

    return FLIGHT_STATE_DROGUE_DESCENT;
}

/**
 * @brief SMF entry handler for drogue descent state.
 */
void state_drogue_descent_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_DROGUE_DESCENT);
    reset_repeated_check(&sm->drogue_main_check);
    sm->drogue_fire_triggered = false;
}

/**
 * @brief SMF run handler for drogue descent state.
 */
enum smf_state_result state_drogue_descent_run(void *obj)
{
    struct flight_sm *sm = obj;

    if (!sm->drogue_fire_triggered &&
        (sm->sample.timestamp_ms - sm->entry_time_ms) >= DROGUE_DEPLOY_DELAY_MS) {
        state_action_fire_drogue();
        sm->drogue_fire_triggered = true;
    }

    transition_to(sm, update_drogue_descent(sm, &sm->sample));
    return SMF_EVENT_HANDLED;
}
