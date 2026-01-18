#include "state_machine_internal.h"
#include "state_machine_states.h"

/**
 * @brief Maintain the terminal landed state.
 */
static flight_state_id_t update_landed(void)
{
    return FLIGHT_STATE_LANDED;
}

/**
 * @brief SMF entry handler for landed state.
 */
void state_landed_entry(void *obj)
{
    struct flight_sm *sm = obj;

    state_entry_common(sm, FLIGHT_STATE_LANDED);
    state_action_landed();
}

/**
 * @brief SMF run handler for landed state.
 */
enum smf_state_result state_landed_run(void *obj)
{
    struct flight_sm *sm = obj;

    transition_to(sm, update_landed());
    return SMF_EVENT_HANDLED;
}
