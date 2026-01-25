#include "state_machine_internal.h"
#include "state_machine_states.h"

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
 * @brief SMF entry handler for ascent state.
 */
void state_ascent_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_ASCENT);
    reset_repeated_check(&sm->mach_lock_check);
    reset_repeated_check(&sm->drogue_main_check);
}

/**
 * @brief SMF run handler for ascent state.
 */
enum smf_state_result state_ascent_run(void *obj)
{
    struct flight_sm *sm = obj;

    transition_to(sm, update_ascent(sm, &sm->sample));
    return SMF_EVENT_HANDLED;
}
