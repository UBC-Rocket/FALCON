#include <zephyr/logging/log.h>

#include "state_machine_internal.h"
#include "../pyro/pyro_thread.h"

LOG_MODULE_DECLARE(state_machine);

/**
 * @brief Update a repeated-check counter and report when it reaches the threshold.
 */
bool repeated_check_update(repeated_check_t *check, bool condition, uint8_t required)
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
void reset_repeated_check(repeated_check_t *check)
{
    check->count = 0;
}

/**
 * @brief Reset ground altitude averaging state.
 */
void reset_ground_average(struct flight_sm *sm)
{
    sm->ground_altitude_m = 0.0f;
    sm->ground_sum_m = 0.0f;
    sm->ground_samples = 0;
    sm->ground_ready = false;
    sm->ground_warmup_start_ms = sm->sample.timestamp_ms;
}

/**
 * @brief Convert an absolute altitude to altitude relative to ground baseline.
 */
float get_relative_altitude(const struct flight_sm *sm, float altitude_m)
{
    return altitude_m - sm->ground_altitude_m;
}

/**
 * @brief Trigger drogue deployment action.
 */
void state_action_fire_drogue(void)
{
    LOG_INF("Drogue deployment triggered");
    int ret = pyro_fire_drogue();
    if (ret != 0) {
        LOG_ERR("Failed to fire drogue: %d", ret);
    }
}

/**
 * @brief Trigger main parachute deployment action.
 */
void state_action_fire_main(void)
{
    LOG_INF("Main deployment triggered");
    int ret = pyro_fire_main();
    if (ret != 0) {
        LOG_ERR("Failed to fire main: %d", ret);
    }
}

/**
 * @brief Trigger landed action.
 */
void state_action_landed(void)
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
void state_entry_common(struct flight_sm *sm, flight_state_id_t state)
{
    sm->current_id = state;
    sm->entry_time_ms = sm->sample.timestamp_ms;
}
