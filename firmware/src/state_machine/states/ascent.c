#include "state_machine_internal.h"
#include "state_machine_states.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(state_ascent, LOG_LEVEL_DBG);

/**
 * @brief Evaluate transitions while in ascent.
 */
static flight_state_id_t update_ascent(struct flight_sm *sm, const state_sample_t *sample)
{
    bool mach_lock = sample->velocity_mps > MACH_LOCK_VELOCITY_THRESHOLD_MPS;

    if (repeated_check_update(&sm->mach_lock_check, mach_lock, MACH_LOCK_CHECKS)) {
        return FLIGHT_STATE_MACH_LOCK;
    }

    if (mach_lock && sm->mach_lock_check.count > 0) {
        LOG_WRN("Mach lock condition MET but waiting for checks: %d/%d", sm->mach_lock_check.count,
                MACH_LOCK_CHECKS);
    }

    bool drogue = sample->velocity_mps < DROGUE_DEPLOY_VELOCITY_THRESHOLD_MPS;
    if (repeated_check_update(&sm->drogue_main_check, drogue, DROGUE_DEPLOY_CHECKS)) {
        return FLIGHT_STATE_DROGUE_DESCENT;
    }

    if (drogue && sm->drogue_main_check.count > 0) {
        LOG_WRN("Drogue deploy condition MET but waiting for checks: %d/%d",
                sm->drogue_main_check.count, DROGUE_DEPLOY_CHECKS);
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
