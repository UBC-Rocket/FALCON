#include "state_machine_internal.h"
#include "state_machine_states.h"

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
 * @brief SMF entry handler for mach lock state.
 */
void state_mach_lock_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_MACH_LOCK);
    reset_repeated_check(&sm->mach_unlock_check);
}

/**
 * @brief SMF run handler for mach lock state.
 */
enum smf_state_result state_mach_lock_run(void *obj)
{
    struct flight_sm *sm = obj;

    transition_to(sm, update_mach_lock(sm, &sm->sample));
    return SMF_EVENT_HANDLED;
}
